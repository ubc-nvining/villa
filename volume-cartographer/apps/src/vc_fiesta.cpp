// vc_fiesta — ScrollFiesta operations on tifxyz segments from the command
// line: topology audit, cleanup, sheet detangling, and spiral-hint generation.
// The batch counterpart
// of the VC3D GUI actions; both go through vc::fiesta (which dlopen-loads
// scrollfiesta.dll/.so at runtime — it must sit next to this executable or
// be reachable via SCROLLFIESTA_DLL / the default library search).
//
// Exit codes: 0 ok, 1 usage, 2 failure, 3 cancelled.

#include "vc/core/fiesta/FiestaOps.hpp"
#include "vc/core/util/QuadSurface.hpp"
#include "utils/Json.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using namespace vc::fiesta;

namespace
{

struct Vec3d
{
    double z = 0.0;
    double y = 0.0;
    double x = 0.0;
};

struct HintOptions
{
    std::string idPrefix;
    Vec3d axisPoint;
    Vec3d axisDirection{1.0, 0.0, 0.0};
    double wrapSpacing = 0.0;
    std::string volumeId;
    std::string volumeLocation;
    double voxelSize = 0.0;
    std::vector<std::string> volumeTags;
    fs::path cancelFile;
};

int usage(const char* argv0)
{
    std::cout
        << "usage: " << argv0 << " <command> <tifxyz_dir> [out_dir] [options]\n"
        << "\n"
        << "commands:\n"
        << "  audit      topology report (read-only)\n"
        << "  clean      manifold repair + pinhole fill (+ optional extras)\n"
        << "  detangle   split fused sheets (depth peel / dev cut / bridge cut / overlap)\n"
        << "  hints      generate new unverified spiral-hint segments from ROI(s)\n"
        << "  survey     batch audit (+ optional detangle) over a tree of segments\n"
        << "\n"
        << "options:\n"
        << "  --roi=X,Y,W,H   operate on a grid-space sub-rectangle (repeat for hints)\n"
        << "  --json=FILE     also write the report as JSON\n"
        << "  --progress      print PROGRESS <pct> <stage> lines\n"
        << "\n"
        << "clean writes the result to <out_dir> (default <in>_fiesta_clean);\n"
        << "detangle writes one segment per piece to <out_dir>/<id>_s<k>.\n"
        << "hints requires <out_dir> and one or more --roi values. Options:\n"
        << "  --id-prefix=ID --axis-point-zyx=Z,Y,X --axis-direction-zyx=Z,Y,X\n"
        << "  --wrap-spacing=N --volume-id=ID --volume-location=PATH\n"
        << "  --voxel-size=N --volume-tag=TAG --cancel-file=FILE\n"
        << "  --manifold=0|1 --pinholes=0|1 --sliver=0|1 --cull-frac=N\n"
        << "Every hint is tagged scroll-hint + unverified. The source is never\n"
        << "modified, existing outputs are never overwritten, and --json is a\n"
        << "vc3d-scrollfiesta-hints-v1 manifest.\n"
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

bool parseVec3d(const std::string& s, Vec3d& out)
{
    return std::sscanf(s.c_str(), "%lf,%lf,%lf", &out.z, &out.y, &out.x) == 3 &&
           std::isfinite(out.z) && std::isfinite(out.y) && std::isfinite(out.x);
}

bool cancelRequested(const fs::path& cancelFile)
{
    if (cancelFile.empty())
        return false;
    std::error_code ec;
    return fs::exists(cancelFile, ec);
}

ProgressFn makeProgress(bool enabled, const fs::path& cancelFile = {})
{
    if (!enabled && cancelFile.empty())
        return {};
    return [enabled, cancelFile](float fraction, const char* stage) {
        if (cancelRequested(cancelFile))
            return false;
        if (enabled) {
            const int pct = std::clamp(
                static_cast<int>(fraction * 100.f), 0, 100);
            std::cout << "PROGRESS " << pct << " "
                      << (stage ? stage : "") << std::endl;
        }
        return !cancelRequested(cancelFile);
    };
}

std::string utcTimestamp()
{
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
#ifdef _WIN32
    gmtime_s(&utc, &now);
#else
    gmtime_r(&now, &utc);
#endif
    std::ostringstream out;
    out << std::put_time(&utc, "%Y%m%dT%H%M%SZ");
    return out.str();
}

utils::Json vec3Json(const Vec3d& v)
{
    auto out = utils::Json::array();
    out.push_back(v.z);
    out.push_back(v.y);
    out.push_back(v.x);
    return out;
}

utils::Json roiJson(const cv::Rect& roi)
{
    auto out = utils::Json::object();
    out["x"] = roi.x;
    out["y"] = roi.y;
    out["width"] = roi.width;
    out["height"] = roi.height;
    return out;
}

utils::Json writebackJson(const vc::core::util::WriteBackStats& st)
{
    auto out = utils::Json::object();
    out["rect"] = roiJson(st.rect);
    out["valid_cells"] = static_cast<std::uint64_t>(st.validCells);
    out["provenanced_cells"] =
        static_cast<std::uint64_t>(st.provenancedCells);
    out["rasterized_cells"] = static_cast<std::uint64_t>(st.rasterizedCells);
    out["conflict_cells"] = static_cast<std::uint64_t>(st.conflictCells);
    out["unplaced_vertices"] =
        static_cast<std::uint64_t>(st.unplacedVertices);
    out["dropped_triangles"] =
        static_cast<std::uint64_t>(st.droppedTriangles);
    return out;
}

void tagSpiralHint(
    QuadSurface& surface, const std::string& parentId, int roiIndex,
    const cv::Rect& roi, const HintOptions& options, const std::string& runId)
{
    if (!surface.meta.is_object())
        surface.meta = utils::Json::object();
    if (!surface.meta.contains("tags") || !surface.meta["tags"].is_object())
        surface.meta["tags"] = utils::Json::object();
    surface.meta["tags"]["scroll-hint"] = true;
    surface.meta["tags"]["unverified"] = true;

    auto fiesta = utils::Json::object();
    fiesta["op"] = "generate-spiral-hint";
    fiesta["generator"] = "vc_fiesta hints";
    fiesta["parent"] = parentId;
    fiesta["roi_index"] = roiIndex;
    fiesta["roi"] = roiJson(roi);
    fiesta["role"] = "spiral-hint";
    fiesta["trust"] = "unverified";
    fiesta["source_untouched"] = true;
    fiesta["run_id"] = runId;
    surface.meta["fiesta"] = std::move(fiesta);

    auto geometry = utils::Json::object();
    geometry["coordinate_order"] = "zyx";
    geometry["axis_point"] = vec3Json(options.axisPoint);
    geometry["axis_direction"] = vec3Json(options.axisDirection);
    geometry["wrap_spacing"] = options.wrapSpacing;
    surface.meta["scroll_geometry"] = std::move(geometry);

    auto volume = utils::Json::object();
    volume["id"] = options.volumeId;
    volume["location"] = options.volumeLocation;
    volume["voxel_size"] = options.voxelSize;
    auto tags = utils::Json::array();
    for (const auto& tag : options.volumeTags)
        tags.push_back(tag);
    volume["tags"] = std::move(tags);
    surface.meta["source_volume"] = std::move(volume);
}

class CreatedOutputsGuard
{
public:
    void reserve(size_t count) { _paths.reserve(count); }

    ~CreatedOutputsGuard()
    {
        if (_committed)
            return;
        for (auto it = _paths.rbegin(); it != _paths.rend(); ++it) {
            std::error_code ec;
            fs::remove_all(*it, ec);
        }
    }

    void add(const fs::path& path) { _paths.push_back(path); }
    void commit() { _committed = true; }

private:
    std::vector<fs::path> _paths;
    bool _committed = false;
};

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

int cmdHints(
    QuadSurface& source, const fs::path& sourcePath, fs::path outRoot,
    const std::vector<cv::Rect>& rois, const fs::path& jsonPath,
    bool showProgress, const sf_cleanup_config& cleanupCfg,
    HintOptions options)
{
    if (outRoot.empty()) {
        std::cerr << "error: hints requires <out_dir>\n";
        return 1;
    }
    if (rois.empty()) {
        std::cerr << "error: hints requires at least one --roi=X,Y,W,H\n";
        return 1;
    }
    if (jsonPath.empty()) {
        std::cerr << "error: hints requires --json=FILE for its import manifest\n";
        return 1;
    }
    for (const auto& roi : rois) {
        if (roi.width <= 0 || roi.height <= 0) {
            std::cerr << "error: hint ROIs must have positive width and height\n";
            return 1;
        }
    }

    const std::string sourceId = sourcePath.filename().string();
    if (options.idPrefix.empty())
        options.idPrefix = sourceId + "_scroll_hint_" + utcTimestamp();
    const fs::path prefixPath(options.idPrefix);
    if (options.idPrefix.empty() || prefixPath.filename() != prefixPath ||
        options.idPrefix == "." || options.idPrefix == "..") {
        std::cerr << "error: --id-prefix must be one directory-safe name\n";
        return 1;
    }
    if (!std::isfinite(options.wrapSpacing) || options.wrapSpacing < 0.0 ||
        !std::isfinite(options.voxelSize) || options.voxelSize < 0.0) {
        std::cerr << "error: wrap spacing and voxel size must be finite and non-negative\n";
        return 1;
    }
    if (std::hypot(
            options.axisDirection.z, options.axisDirection.y,
            options.axisDirection.x) <= 0.0) {
        std::cerr << "error: axis direction must be non-zero\n";
        return 1;
    }

    std::error_code ec;
    fs::create_directories(outRoot, ec);
    if (ec || !fs::is_directory(outRoot)) {
        std::cerr << "error: cannot create output root " << outRoot << ": "
                  << ec.message() << "\n";
        return 2;
    }
    if (fs::exists(jsonPath)) {
        std::cerr << "error: manifest already exists (refusing overwrite): "
                  << jsonPath << "\n";
        return 2;
    }

    std::vector<fs::path> finalPaths;
    finalPaths.reserve(rois.size());
    for (size_t k = 0; k < rois.size(); ++k) {
        const std::string id = options.idPrefix + "_r" + std::to_string(k);
        const fs::path out = outRoot / id;
        if (fs::exists(out)) {
            std::cerr << "error: output already exists (refusing overwrite): "
                      << out << "\n";
            return 2;
        }
        finalPaths.push_back(out);
    }

    const std::string runId = options.idPrefix;
    auto manifest = utils::Json::object();
    manifest["schema"] = "vc3d-scrollfiesta-hints-v1";
    manifest["generator"] = "vc_fiesta hints";
    manifest["scrollfiesta_version"] =
        FiestaRuntime::instance().versionString();
    manifest["created_utc"] = utcTimestamp();
    manifest["run_id"] = runId;
    auto sourceJson = utils::Json::object();
    sourceJson["id"] = sourceId;
    sourceJson["path"] = fs::absolute(sourcePath).string();
    sourceJson["untouched"] = true;
    manifest["source"] = std::move(sourceJson);
    auto geometry = utils::Json::object();
    geometry["coordinate_order"] = "zyx";
    geometry["axis_point"] = vec3Json(options.axisPoint);
    geometry["axis_direction"] = vec3Json(options.axisDirection);
    geometry["wrap_spacing"] = options.wrapSpacing;
    manifest["scroll_geometry"] = std::move(geometry);
    auto outputs = utils::Json::array();

    const ProgressFn overall = makeProgress(showProgress, options.cancelFile);
    CreatedOutputsGuard guard;
    guard.reserve(rois.size() + 2);
    for (size_t k = 0; k < rois.size(); ++k) {
        const float begin = static_cast<float>(k) /
                            static_cast<float>(rois.size());
        const float span = 1.0f / static_cast<float>(rois.size());
        ProgressFn roiProgress;
        if (overall) {
            roiProgress = [overall, begin, span](float f, const char* stage) {
                return overall(begin + span * std::clamp(f, 0.0f, 0.90f), stage);
            };
            if (!overall(begin, "preparing hint ROI"))
                throw FiestaCancelled();
        }

        CleanupResult clean = cleanupQuadSurfaceRoi(
            source, rois[k], &cleanupCfg, roiProgress);
        const fs::path& out = finalPaths[k];
        const std::string id = out.filename().string();
        tagSpiralHint(
            *clean.surface, sourceId, static_cast<int>(k), rois[k], options,
            runId);
        if (overall && !overall(begin + span * 0.92f, "writing hint atomically"))
            throw FiestaCancelled();
        // QuadSurface::save writes to a sibling temporary directory and then
        // renames it into place, so a failed save cannot expose half a tifxyz.
        clean.surface->save(out, id);
        guard.add(out);
        if (cancelRequested(options.cancelFile))
            throw FiestaCancelled();

        auto entry = utils::Json::object();
        entry["id"] = id;
        entry["path"] = fs::absolute(out).string();
        entry["roi_index"] = static_cast<int>(k);
        entry["roi"] = roiJson(rois[k]);
        entry["trust"] = "unverified";
        entry["before"] = utils::Json::parse(clean.before.json());
        entry["after"] = utils::Json::parse(clean.after.json());
        entry["writeback"] = writebackJson(clean.writeback);
        entry["retained_fraction"] = clean.retainedFraction;
        outputs.push_back(std::move(entry));
        if (overall && !overall(begin + span, "hint written"))
            throw FiestaCancelled();
    }
    manifest["outputs"] = std::move(outputs);

    if (!jsonPath.parent_path().empty()) {
        fs::create_directories(jsonPath.parent_path(), ec);
        if (ec)
            throw std::runtime_error(
                "could not create manifest directory: " + ec.message());
    }
    fs::path tmpManifest = jsonPath;
    tmpManifest += ".partial";
    if (fs::exists(tmpManifest))
        throw std::runtime_error(
            "manifest staging path already exists (refusing overwrite)");
    guard.add(tmpManifest);
    {
        std::ofstream json(tmpManifest, std::ios::binary | std::ios::trunc);
        if (!json)
            throw std::runtime_error("could not open manifest for writing");
        json << manifest.dump(2) << "\n";
        if (!json)
            throw std::runtime_error("failed while writing manifest");
    }
    if (cancelRequested(options.cancelFile)) {
        throw FiestaCancelled();
    }
    fs::rename(tmpManifest, jsonPath);
    guard.add(jsonPath);
    if (cancelRequested(options.cancelFile))
        throw FiestaCancelled();

    if (overall && !overall(1.0f, "complete"))
        throw FiestaCancelled();
    guard.commit();
    std::cout << "generated " << finalPaths.size()
              << " unverified spiral hint(s); source untouched\n";
    for (const auto& out : finalPaths)
        std::cout << "wrote " << out << "\n";
    std::cout << "manifest " << jsonPath << "\n";
    return 0;
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
    std::vector<cv::Rect> rois;
    fs::path json_path;
    bool progress = false;
    bool survey_detangle = false;
    fs::path csv_path;
    std::uint64_t max_cells = 6000000ull;
    bool no_peel = false, no_dev = false, no_bridge = false, no_overlap = false;
    std::optional<int> cleanup_manifold;
    std::optional<int> cleanup_pinholes;
    std::optional<int> cleanup_sliver;
    std::optional<double> cleanup_cull_frac;
    HintOptions hint_options;

    try {
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
                cv::Rect roi;
                if (!parseRoi(a.substr(6), roi)) {
                    std::cerr << "error: bad --roi (expected X,Y,W,H)\n";
                    return 1;
                }
                rois.push_back(roi);
            } else if (a.rfind("--json=", 0) == 0) {
                json_path = a.substr(7);
            } else if (a == "--progress") {
                progress = true;
            } else if (a.rfind("--id-prefix=", 0) == 0) {
                hint_options.idPrefix = a.substr(12);
            } else if (a.rfind("--axis-point-zyx=", 0) == 0) {
                if (!parseVec3d(a.substr(17), hint_options.axisPoint)) {
                    std::cerr << "error: bad --axis-point-zyx (expected Z,Y,X)\n";
                    return 1;
                }
            } else if (a.rfind("--axis-direction-zyx=", 0) == 0) {
                if (!parseVec3d(a.substr(21), hint_options.axisDirection)) {
                    std::cerr << "error: bad --axis-direction-zyx (expected Z,Y,X)\n";
                    return 1;
                }
            } else if (a.rfind("--wrap-spacing=", 0) == 0) {
                hint_options.wrapSpacing = std::stod(a.substr(15));
            } else if (a.rfind("--volume-id=", 0) == 0) {
                hint_options.volumeId = a.substr(12);
            } else if (a.rfind("--volume-location=", 0) == 0) {
                hint_options.volumeLocation = a.substr(18);
            } else if (a.rfind("--voxel-size=", 0) == 0) {
                hint_options.voxelSize = std::stod(a.substr(13));
            } else if (a.rfind("--volume-tag=", 0) == 0) {
                hint_options.volumeTags.push_back(a.substr(13));
            } else if (a.rfind("--cancel-file=", 0) == 0) {
                hint_options.cancelFile = a.substr(14);
            } else if (a.rfind("--manifold=", 0) == 0) {
                cleanup_manifold = std::stoi(a.substr(11));
            } else if (a.rfind("--pinholes=", 0) == 0) {
                cleanup_pinholes = std::stoi(a.substr(11));
            } else if (a.rfind("--sliver=", 0) == 0) {
                cleanup_sliver = std::stoi(a.substr(9));
            } else if (a.rfind("--cull-frac=", 0) == 0) {
                cleanup_cull_frac = std::stod(a.substr(12));
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
    } catch (const std::exception& e) {
        std::cerr << "error: invalid option value: " << e.what() << "\n";
        return 1;
    }

    const auto validSwitch = [](const std::optional<int>& value) {
        return !value || *value == 0 || *value == 1;
    };
    if (!validSwitch(cleanup_manifold) || !validSwitch(cleanup_pinholes) ||
        !validSwitch(cleanup_sliver)) {
        std::cerr << "error: cleanup switches must be 0 or 1\n";
        return 1;
    }
    if (cleanup_cull_frac &&
        (!std::isfinite(*cleanup_cull_frac) || *cleanup_cull_frac < 0.0 ||
         *cleanup_cull_frac > 1.0)) {
        std::cerr << "error: --cull-frac must be finite and in [0,1]\n";
        return 1;
    }
    if (command != "hints" && rois.size() > 1) {
        std::cerr << "error: repeated --roi is supported only by hints\n";
        return 1;
    }
    const cv::Rect roi = rois.empty() ? cv::Rect{} : rois.front();

    auto& runtime = FiestaRuntime::instance();
    if (!runtime.available()) {
        std::cerr << "error: " << runtime.unavailableReason() << "\n";
        return 2;
    }
    std::cout << "scrollfiesta " << runtime.versionString() << "\n";

    sf_cleanup_config cleanupCfg = runtime.api()->cleanup_config_default();
    if (cleanup_manifold) {
        cleanupCfg.manifold_repair = *cleanup_manifold;
        cleanupCfg.reorient = *cleanup_manifold;
    }
    if (cleanup_pinholes)
        cleanupCfg.fill_pinholes = *cleanup_pinholes;
    if (cleanup_sliver)
        cleanupCfg.sliver_cleanup = *cleanup_sliver;
    if (cleanup_cull_frac)
        cleanupCfg.cull_min_area_frac = static_cast<float>(*cleanup_cull_frac);

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
        if (command == "hints") {
            return cmdHints(
                *surf, in_path, out_path, rois, json_path, progress,
                cleanupCfg, std::move(hint_options));
        }

        if (command == "audit") {
            AuditReport report =
                auditQuadSurface(
                    *surf, roi,
                    makeProgress(progress, hint_options.cancelFile));
            std::cout << report.pretty();
            if (!json_path.empty())
                std::ofstream(json_path) << report.json() << "\n";
            return 0;
        }

        if (command == "clean") {
            CleanupResult result =
                cleanupQuadSurfaceRoi(
                    *surf, roi, &cleanupCfg,
                    makeProgress(progress, hint_options.cancelFile));
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
                                    makeProgress(progress, hint_options.cancelFile));
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
