// vc_fiesta — ScrollFiesta operations on tifxyz segments from the command
// line: topology audit, cleanup, and sheet detangling. The batch counterpart
// of the VC3D GUI actions; both go through vc::fiesta (which dlopen-loads
// scrollfiesta.dll/.so at runtime — it must sit next to this executable or
// be reachable via SCROLLFIESTA_DLL / the default library search).
//
// Exit codes: 0 ok, 1 usage, 2 failure, 3 cancelled.

#include "vc/core/fiesta/FiestaOps.hpp"
#include "vc/core/util/QuadSurface.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

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
        << "\n"
        << "options:\n"
        << "  --roi=X,Y,W,H   operate on a grid-space sub-rectangle only\n"
        << "  --json=FILE     also write the report as JSON\n"
        << "  --progress      print PROGRESS <pct> <stage> lines\n"
        << "\n"
        << "clean writes the result to <out_dir> (default <in>_fiesta_clean);\n"
        << "detangle writes one segment per piece to <out_dir>/<id>_s<k>.\n";
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

    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind("--roi=", 0) == 0) {
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
