// vc_fiesta — ScrollFiesta operations on tifxyz segments from the command
// line: topology audit, cleanup, and sheet detangling. The batch counterpart
// of the VC3D GUI actions; both go through vc::fiesta (which dlopen-loads
// scrollfiesta.dll/.so at runtime — it must sit next to this executable or
// be reachable via SCROLLFIESTA_DLL / the default library search).
//
// Exit codes: 0 ok, 1 usage, 2 failure, 3 cancelled.

#include "vc/core/fiesta/FiestaOps.hpp"
#include "vc/core/util/QuadSurface.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace vc::fiesta;

namespace
{

int usage(const char* argv0)
{
    std::cout
        << "usage: " << argv0 << " <command> <tifxyz_dir> [out_dir] [options]\n"
        << "\n"
        << "commands:\n"
        << "  audit      topology report (read-only)\n"
        << "  clean      manifold repair + pinhole fill (+ optional extras)\n"
        << "  detangle   split fused sheets (depth peel / dev cut / bridge cut / overlap)\n"
        << "  survey     batch audit (+ optional detangle) over a tree of segments\n"
        << "\n"
        << "options:\n"
        << "  --roi=X,Y,W,H   operate on a grid-space sub-rectangle only\n"
        << "  --json=FILE     also write the report as JSON\n"
        << "  --progress      print PROGRESS <pct> <stage> lines\n"
        << "\n"
        << "clean writes the result to <out_dir> (default <in>_fiesta_clean);\n"
        << "detangle writes one segment per piece to <out_dir>/<id>_s<k>.\n"
        << "\n"
        << "survey options: --detangle  --csv=FILE  --max-cells=N (default 6000000)\n"
        << "  stage toggles: --no-peel --no-dev --no-bridge --no-overlap\n"
        << "  Walks <tifxyz_dir> recursively for materialized tifxyz segments\n"
        << "  (meta.json + x.tif), audits each, optionally detangles (pieces\n"
        << "  land in [out_dir], default <in>/_fiesta_survey), and writes a\n"
        << "  CSV summary.\n";
    return 1;
}

bool parseRoi(const std::string& s, cv::Rect& roi)
{
    int x, y, w, h;
    if (std::sscanf(s.c_str(), "%d,%d,%d,%d", &x, &y, &w, &h) != 4)
        return false;
    roi = cv::Rect(x, y, w, h);
    return true;
}

ProgressFn makeProgress(bool enabled)
{
    if (!enabled)
        return {};
    return [](float fraction, const char* stage) {
        std::cout << "PROGRESS " << static_cast<int>(fraction * 100.f) << " "
                  << (stage ? stage : "") << std::endl;
        return true;
    };
}

void printWriteback(const vc::core::util::WriteBackStats& st)
{
    std::cout << "  grid rect: " << st.rect.x << "," << st.rect.y << " "
              << st.rect.width << "x" << st.rect.height
              << "  valid cells: " << st.validCells
              << " (exact " << st.provenancedCells << ", rasterized "
              << st.rasterizedCells << ")\n";
    if (st.conflictCells || st.unplacedVertices || st.droppedTriangles)
        std::cout << "  warnings: conflicts " << st.conflictCells
                  << ", unplaced verts " << st.unplacedVertices
                  << ", dropped triangles " << st.droppedTriangles << "\n";
}

double secondsSince(const std::chrono::steady_clock::time_point& t0)
{
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now() - t0)
        .count();
}

int cmdSurvey(
    const fs::path& root, fs::path outDir, bool detangle,
    fs::path csvPath, std::uint64_t maxCells,
    const sf_detangle_config& detCfg)
{
    // Collect materialized tifxyz segment dirs (meta.json + x.tif).
    std::vector<fs::path> segDirs;
    auto isSegmentDir = [](const fs::path& d) {
        return fs::exists(d / "meta.json") && fs::exists(d / "x.tif");
    };
    if (isSegmentDir(root)) {
        segDirs.push_back(root);
    } else {
        for (auto it = fs::recursive_directory_iterator(root);
             it != fs::recursive_directory_iterator(); ++it) {
            if (it->is_directory() && isSegmentDir(it->path())) {
                segDirs.push_back(it->path());
                it.disable_recursion_pending();
            }
        }
    }
    std::sort(segDirs.begin(), segDirs.end());
    if (segDirs.empty()) {
        std::cerr << "error: no materialized tifxyz segments under " << root
                  << " (need meta.json + x.tif; did you run vc_opendata pull?)\n";
        return 2;
    }

    if (outDir.empty())
        outDir = root / "_fiesta_survey";
    fs::create_directories(outDir);
    if (csvPath.empty())
        csvPath = outDir / "survey.csv";
    std::ofstream csv(csvPath);
    csv << "segment,cells,verts,faces,components,boundary_loops,nm_edges,"
           "retained,"
           "genus,is_disk,audit_s,pieces,peel,dev,bridge,overlap,detangle_s,"
           "wb_conflicts,wb_unplaced,wb_dropped,status\n";

    std::cout << "\nsurveying " << segDirs.size() << " segment(s); output "
              << outDir << "\n\n";

    int ok = 0, skipped = 0, failed = 0;
    for (const auto& dir : segDirs) {
        const std::string id = dir.filename().string();
        std::string status = "ok";
        std::uint64_t cells = 0;
        AuditReport audit;
        double audit_s = 0, det_s = 0;
        size_t pieces = 0;
        sf_detangle_report drep{};
        int wb_conflicts = 0, wb_unplaced = 0, wb_dropped = 0;
        double retained = 1.0;

        try {
            auto surf = load_quad_from_tifxyz(dir);
            const cv::Mat_<cv::Vec3f> P = surf->rawPoints();
            cells = static_cast<std::uint64_t>(P.rows) *
                    static_cast<std::uint64_t>(P.cols);
            if (cells > maxCells) {
                status = "skipped:too-large";
                ++skipped;
            } else {
                auto t0 = std::chrono::steady_clock::now();
                audit = auditQuadSurface(*surf);
                audit_s = secondsSince(t0);

                if (detangle) {
                    auto t1 = std::chrono::steady_clock::now();
                    // Research harness: disable the retention guard (0.0) so we
                    // measure what the splitters actually do, rather than
                    // refusing. The GUI / `vc_fiesta detangle` keep the guard.
                    DetangleResult det =
                        detangleQuadSurface(*surf, {}, &detCfg, {}, 0.0);
                    det_s = secondsSince(t1);
                    pieces = det.pieces.size();
                    drep = det.report;
                    retained = det.retainedFraction;
                    int k = 0;
                    for (auto& piece : det.pieces) {
                        wb_conflicts += piece.writeback.conflictCells;
                        wb_unplaced += piece.writeback.unplacedVertices;
                        wb_dropped += piece.writeback.droppedTriangles;
                        if (pieces > 1) {
                            const fs::path out =
                                outDir / (id + "_s" + std::to_string(k++));
                            piece.surface->save(
                                out.string(), out.filename().string());
                        }
                    }
                }
                ++ok;
            }
        } catch (const std::exception& e) {
            status = std::string("fail:") + e.what();
            ++failed;
        }

        std::cout << std::left << std::setw(28) << id << " cells=" << cells
                  << " comps=" << audit.topo.n_components
                  << " loops=" << audit.topo.n_boundary_loops
                  << " nm=" << audit.topo.n_nonmanifold_edges
                  << " genus=" << audit.topo.genus
                  << " disk=" << (audit.topo.is_disk ? "y" : "n");
        if (detangle && status == "ok")
            std::cout << " | pieces=" << pieces << " (p" << drep.peel_splits
                      << "/d" << drep.dev_splits << "/b" << drep.bridge_splits
                      << "/o" << drep.overlap_splits << ") kept " << std::fixed
                      << std::setprecision(1) << (retained * 100.0) << "% "
                      << det_s << "s";
        if (status != "ok")
            std::cout << " [" << status << "]";
        std::cout << "\n";

        csv << id << "," << cells << "," << audit.topo.n_verts << ","
            << audit.topo.n_faces << "," << audit.topo.n_components << ","
            << audit.topo.n_boundary_loops << ","
            << audit.topo.n_nonmanifold_edges << "," << retained << ","
            << audit.topo.genus << ","
            << audit.topo.is_disk << "," << audit_s << "," << pieces << ","
            << drep.peel_splits << "," << drep.dev_splits << ","
            << drep.bridge_splits << "," << drep.overlap_splits << ","
            << det_s << "," << wb_conflicts << "," << wb_unplaced << ","
            << wb_dropped << ",\"" << status << "\"\n";
    }

    std::cout << "\nsurvey: " << ok << " ok, " << skipped << " skipped, "
              << failed << " failed -> " << csvPath << "\n";
    return failed ? 2 : 0;
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc < 3)
        return usage(argv[0]);

    const std::string command = argv[1];
    const fs::path in_path = argv[2];
    fs::path out_path;
    cv::Rect roi;
    fs::path json_path;
    bool progress = false;
    bool survey_detangle = false;
    fs::path csv_path;
    std::uint64_t max_cells = 6000000ull;
    bool no_peel = false, no_dev = false, no_bridge = false, no_overlap = false;

    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--detangle") {
            survey_detangle = true;
        } else if (a == "--no-peel") {
            no_peel = true;
        } else if (a == "--no-dev") {
            no_dev = true;
        } else if (a == "--no-bridge") {
            no_bridge = true;
        } else if (a == "--no-overlap") {
            no_overlap = true;
        } else if (a.rfind("--csv=", 0) == 0) {
            csv_path = a.substr(6);
        } else if (a.rfind("--max-cells=", 0) == 0) {
            max_cells = std::stoull(a.substr(12));
        } else if (a.rfind("--roi=", 0) == 0) {
            if (!parseRoi(a.substr(6), roi)) {
                std::cerr << "error: bad --roi (expected X,Y,W,H)\n";
                return 1;
            }
        } else if (a.rfind("--json=", 0) == 0) {
            json_path = a.substr(7);
        } else if (a == "--progress") {
            progress = true;
        } else if (a.rfind("--", 0) == 0) {
            std::cerr << "error: unknown option '" << a << "'\n";
            return 1;
        } else if (out_path.empty()) {
            out_path = a;
        } else {
            std::cerr << "error: unexpected argument '" << a << "'\n";
            return 1;
        }
    }

    auto& runtime = FiestaRuntime::instance();
    if (!runtime.available()) {
        std::cerr << "error: " << runtime.unavailableReason() << "\n";
        return 2;
    }
    std::cout << "scrollfiesta " << runtime.versionString() << "\n";

    if (command == "survey") {
        const sf_api* api = FiestaRuntime::instance().api();
        sf_detangle_config detCfg = api->detangle_config_default();
        if (no_peel) detCfg.enable_peel = 0;
        if (no_dev) detCfg.enable_dev = 0;
        if (no_bridge) detCfg.enable_bridge = 0;
        if (no_overlap) detCfg.enable_overlap = 0;
        try {
            return cmdSurvey(in_path, out_path, survey_detangle, csv_path,
                             max_cells, detCfg);
        } catch (const std::exception& e) {
            std::cerr << "error: " << e.what() << "\n";
            return 2;
        }
    }

    std::unique_ptr<QuadSurface> surf;
    try {
        surf = load_quad_from_tifxyz(in_path);
    } catch (...) {
        std::cerr << "error: could not load tifxyz: " << in_path << "\n";
        return 2;
    }
    const std::string in_id = in_path.filename().string();

    try {
        if (command == "audit") {
            AuditReport report =
                auditQuadSurface(*surf, roi, makeProgress(progress));
            std::cout << report.pretty();
            if (!json_path.empty())
                std::ofstream(json_path) << report.json() << "\n";
            return 0;
        }

        if (command == "clean") {
            CleanupResult result =
                cleanupQuadSurfaceRoi(*surf, roi, nullptr,
                                      makeProgress(progress));
            std::cout << "before: components " << result.before.topo.n_components
                      << ", non-manifold edges "
                      << result.before.topo.n_nonmanifold_edges
                      << ", boundary loops "
                      << result.before.topo.n_boundary_loops << "\n";
            std::cout << "after:  components " << result.after.topo.n_components
                      << ", non-manifold edges "
                      << result.after.topo.n_nonmanifold_edges
                      << ", boundary loops "
                      << result.after.topo.n_boundary_loops << "\n";
            printWriteback(result.writeback);

            fs::path out = out_path.empty()
                               ? in_path.parent_path() / (in_id + "_fiesta_clean")
                               : out_path;
            result.surface->save(out.string(), out.filename().string());
            std::cout << "wrote " << out << "\n";
            if (!json_path.empty())
                std::ofstream(json_path)
                    << "{\"before\":" << result.before.json()
                    << ",\"after\":" << result.after.json() << "}\n";
            return 0;
        }

        if (command == "detangle") {
            DetangleResult result =
                detangleQuadSurface(*surf, roi, nullptr,
                                    makeProgress(progress));
            std::cout << "pieces: " << result.pieces.size()
                      << " (peel " << result.report.peel_splits
                      << ", dev " << result.report.dev_splits
                      << ", bridge " << result.report.bridge_splits
                      << ", overlap " << result.report.overlap_splits << ")\n";

            const fs::path out_dir = out_path.empty()
                                         ? in_path.parent_path()
                                         : out_path;
            int k = 0;
            for (auto& piece : result.pieces) {
                const std::string id =
                    in_id + "_fiesta_s" + std::to_string(k++);
                printWriteback(piece.writeback);
                piece.surface->save((out_dir / id).string(), id);
                std::cout << "wrote " << (out_dir / id) << "\n";
            }
            return 0;
        }
    } catch (const FiestaCancelled&) {
        std::cerr << "cancelled\n";
        return 3;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 2;
    }

    std::cerr << "error: unknown command '" << command << "'\n";
    return usage(argv[0]);
}
