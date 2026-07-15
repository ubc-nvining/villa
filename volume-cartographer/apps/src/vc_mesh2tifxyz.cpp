// vc_mesh2tifxyz — turn an arbitrary triangle mesh (OBJ) into a tifxyz
// segment, without requiring UVs in the file.
//
// Two routes, picked automatically:
//  1. Provenance write-back: when the OBJ has a .griduv sidecar (written by
//     vc_tifxyz2obj / TriMeshBridge) mapping vertices to source grid cells,
//     positions are written straight back into those cells — bit-exact where
//     the mesh kept them — and covered holes are raster-filled. Use
//     --source=<tifxyz> to inherit scale and carry channels.
//  2. Flatten + rasterize: otherwise the mesh is ABF++/LSCM-parameterized to
//     one chart and rasterized into a fresh grid (the mesh must be a manifold
//     disk — clean/cut it first, e.g. with vc_fiesta or ScrollFiesta).
//
// This replaces the external flatten->vc_obj2tifxyz detour for meshes that
// have no UVs (e.g. ScrollFiesta's welded.obj — NB its tool OBJs are
// "v z y x"; pass --zyx for those).

#include "vc/core/util/QuadSurface.hpp"
#include "vc/core/util/TriMeshBridge.hpp"
#include "vc/flattening/MeshFlatten.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;
using vc::core::util::TriMesh;

namespace
{

int usage(const char* argv0)
{
    std::cout
        << "usage: " << argv0 << " <in.obj> <out_tifxyz_dir> [options]\n"
        << "  --source=<tifxyz>  inherit scale (and channels, with a sidecar)\n"
        << "  --scale=SX,SY      output grid cells per voxel (default 0.05)\n"
        << "  --flatten          force the flatten route even with a sidecar\n"
        << "  --lscm-only        skip ABF++ (faster, slightly more distortion)\n"
        << "  --zyx              input OBJ uses ScrollFiesta tool order (v z y x)\n";
    return 1;
}

bool loadObj(const fs::path& path, bool zyx, TriMesh& mesh)
{
    std::ifstream file(path);
    if (!file.is_open())
        return false;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream iss(line);
        std::string tag;
        iss >> tag;
        if (tag == "v") {
            float a, b, c;
            if (iss >> a >> b >> c) {
                if (zyx)
                    mesh.vertices.emplace_back(c, b, a);
                else
                    mesh.vertices.emplace_back(a, b, c);
            }
        } else if (tag == "f") {
            cv::Vec3i f;
            std::string tok;
            int k = 0;
            while (iss >> tok && k < 3) {
                // "v", "v/vt", "v/vt/vn", "v//vn" — the vertex index leads.
                f[k++] = std::stoi(tok) - 1;
            }
            if (k == 3)
                mesh.faces.push_back(f);
        }
    }
    return !mesh.vertices.empty() && !mesh.faces.empty();
}

// "<obj>.griduv": one "col row" line per vertex, in OBJ vertex order.
bool loadGridUvSidecar(
    const fs::path& objPath, size_t nVerts, std::vector<cv::Vec2i>& cells)
{
    std::ifstream file(objPath.string() + ".griduv");
    if (!file.is_open())
        return false;
    float col = 0, row = 0;
    while (file >> col >> row)
        cells.emplace_back(
            static_cast<int>(std::lround(row)),
            static_cast<int>(std::lround(col)));
    if (cells.size() != nVerts) {
        std::cerr << "warning: .griduv has " << cells.size()
                  << " entries but OBJ has " << nVerts
                  << " vertices; ignoring sidecar\n";
        cells.clear();
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc < 3)
        return usage(argv[0]);

    const fs::path in_path = argv[1];
    const fs::path out_path = argv[2];
    fs::path source_path;
    cv::Vec2f scale(0.f, 0.f);
    bool force_flatten = false;
    bool lscm_only = false;
    bool zyx = false;

    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind("--source=", 0) == 0) {
            source_path = a.substr(9);
        } else if (a.rfind("--scale=", 0) == 0) {
            if (std::sscanf(a.c_str() + 8, "%f,%f", &scale[0], &scale[1]) != 2) {
                std::cerr << "error: bad --scale (expected SX,SY)\n";
                return 1;
            }
        } else if (a == "--flatten") {
            force_flatten = true;
        } else if (a == "--lscm-only") {
            lscm_only = true;
        } else if (a == "--zyx") {
            zyx = true;
        } else {
            std::cerr << "error: unknown option '" << a << "'\n";
            return 1;
        }
    }

    TriMesh mesh;
    if (!loadObj(in_path, zyx, mesh)) {
        std::cerr << "error: could not load OBJ: " << in_path << "\n";
        return 2;
    }
    std::cout << "loaded " << mesh.vertices.size() << " vertices, "
              << mesh.faces.size() << " faces\n";

    std::unique_ptr<QuadSurface> source;
    if (!source_path.empty()) {
        try {
            source = load_quad_from_tifxyz(source_path);
            if (scale[0] <= 0.f)
                scale = source->scale();
        } catch (...) {
            std::cerr << "warning: could not load --source tifxyz: "
                      << source_path << "\n";
        }
    }

    std::unique_ptr<QuadSurface> result;
    std::vector<cv::Vec2i> cells;
    if (!force_flatten && loadGridUvSidecar(in_path, mesh.vertices.size(), cells)) {
        std::cout << "route: provenance write-back (.griduv sidecar)\n";
        vc::core::util::GridMeshProvenance prov;
        prov.scale = scale[0] > 0.f ? scale : cv::Vec2f(0.05f, 0.05f);
        vc::core::util::WriteBackOptions opt;
        opt.channelSource = source.get();
        vc::core::util::WriteBackStats stats;
        result = vc::core::util::writeBackToGrid(mesh, cells, prov, opt, &stats);
        if (result) {
            std::cout << "grid " << stats.rect.width << "x" << stats.rect.height
                      << " at (" << stats.rect.x << "," << stats.rect.y
                      << "), valid " << stats.validCells << " (exact "
                      << stats.provenancedCells << ", rasterized "
                      << stats.rasterizedCells << ")\n";
            if (stats.conflictCells || stats.unplacedVertices ||
                stats.droppedTriangles)
                std::cout << "warnings: conflicts " << stats.conflictCells
                          << ", unplaced " << stats.unplacedVertices
                          << ", dropped tris " << stats.droppedTriangles << "\n";
        }
    } else {
        std::cout << "route: flatten + rasterize (ABF++"
                  << (lscm_only ? " off, LSCM only" : "/LSCM") << ")\n";
        vc::ABFConfig cfg;
        cfg.useABF = !lscm_only;
        vc::MeshFlattenResult flat =
            vc::meshToQuadSurfaceByFlattening(mesh, scale, cfg);
        if (!flat.surface) {
            std::cerr << "error: flatten failed: "
                      << flat.diagnostics.failureReason << "\n";
            return 2;
        }
        std::cout << "valid cells: " << flat.validCells
                  << ", UV-flipped triangles: " << flat.flippedTriangles << "\n";
        if (flat.flippedTriangles > 0)
            std::cout << "warning: the chart folds over itself in "
                      << flat.flippedTriangles
                      << " triangle(s); inspect before trusting\n";
        result = std::move(flat.surface);
    }

    if (!result) {
        std::cerr << "error: no output surface could be produced\n";
        return 2;
    }
    const std::string id = out_path.filename().string();
    try {
        result->save(out_path.string(), id);
    } catch (const std::exception& e) {
        std::cerr << "error saving tifxyz: " << e.what() << "\n";
        return 2;
    }
    std::cout << "wrote " << out_path << "\n";
    return 0;
}
