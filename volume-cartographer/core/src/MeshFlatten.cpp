#include "vc/flattening/MeshFlatten.hpp"

#include "vc/core/util/GridTriangleRaster.hpp"
#include "vc/core/util/QuadSurface.hpp"

#include <OpenABF/OpenABF.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace vc
{

namespace
{

using HalfEdgeMesh = OpenABF::detail::ABF::Mesh<double>;
using ABF = OpenABF::ABFPlusPlus<double>;
using LSCM = OpenABF::AngleBasedLSCM<double, HalfEdgeMesh>;

double triArea3D(const cv::Vec3f& a, const cv::Vec3f& b, const cv::Vec3f& c)
{
    const cv::Vec3f ab = b - a, ac = c - a;
    return 0.5 * cv::norm(ab.cross(ac));
}

double triAreaUVSigned(
    const cv::Vec2f& a, const cv::Vec2f& b, const cv::Vec2f& c)
{
    return 0.5 * (static_cast<double>(b[0] - a[0]) * (c[1] - a[1]) -
                  static_cast<double>(b[1] - a[1]) * (c[0] - a[0]));
}

void fail(ABFDiagnostics* diag, const std::string& reason)
{
    if (diag) {
        diag->success = false;
        diag->failureReason = reason;
    }
    std::cerr << "MeshFlatten: " << reason << std::endl;
}

}  // namespace

std::vector<cv::Vec2f> abfFlattenMesh(
    const core::util::TriMesh& mesh, const ABFConfig& cfg,
    ABFDiagnostics* diag, std::size_t lscmOnlyFaceThreshold)
{
    if (diag)
        *diag = ABFDiagnostics{};
    if (mesh.vertices.size() < 3 || mesh.faces.empty()) {
        fail(diag, "mesh is empty");
        return {};
    }

    auto hem = HalfEdgeMesh::New();
    try {
        for (const auto& v : mesh.vertices) {
            OpenABF::Vec3d p;
            p[0] = v[0];
            p[1] = v[1];
            p[2] = v[2];
            hem->insert_vertex(p);
        }
        for (const auto& f : mesh.faces) {
            std::vector<std::size_t> face = {
                static_cast<std::size_t>(f[0]),
                static_cast<std::size_t>(f[1]),
                static_cast<std::size_t>(f[2])};
            hem->insert_face(face);
        }
        hem->update_boundary();
    } catch (const OpenABF::MeshException& e) {
        fail(diag, std::string("mesh not parameterizable as-is (run cleanup/"
                               "cut-to-disk first): ") +
                       e.what());
        return {};
    }
    if (!OpenABF::IsManifold(hem)) {
        fail(diag, "mesh is not manifold (run cleanup first)");
        return {};
    }

    const bool useABF = cfg.useABF && mesh.faces.size() <= lscmOnlyFaceThreshold;
    if (cfg.useABF && !useABF) {
        std::cout << "MeshFlatten: " << mesh.faces.size()
                  << " faces > " << lscmOnlyFaceThreshold
                  << " — using LSCM only" << std::endl;
    }
    if (useABF) {
        std::size_t iters = 0;
        double grad = 0;
        try {
            ABF::Compute(hem, iters, grad, cfg.maxIterations);
            if (diag) {
                diag->abfIterations = iters;
                diag->abfGradient = grad;
            }
        } catch (const OpenABF::SolverException& e) {
            std::cerr << "MeshFlatten: ABF solver failed (" << e.what()
                      << "), falling back to LSCM only" << std::endl;
            if (diag)
                diag->failureReason =
                    std::string("ABF solver failed; used LSCM fallback: ") +
                    e.what();
        }
    }
    try {
        LSCM::Compute(hem);
    } catch (const std::exception& e) {
        fail(diag, std::string("LSCM failed: ") + e.what());
        return {};
    }

    std::vector<cv::Vec2f> uvs(mesh.vertices.size(), cv::Vec2f(0.f, 0.f));
    for (const auto& v : hem->vertices()) {
        uvs[v->idx] = cv::Vec2f(
            static_cast<float>(v->pos[0]), static_cast<float>(v->pos[1]));
    }

    // Metric scaling: total UV area == total 3D area, so UV units are voxels
    // and a grid at `scale` cells/voxel reproduces segment density.
    if (cfg.scaleToOriginalArea) {
        double a3 = 0.0, a2 = 0.0;
        for (const auto& f : mesh.faces) {
            a3 += triArea3D(
                mesh.vertices[(size_t)f[0]], mesh.vertices[(size_t)f[1]],
                mesh.vertices[(size_t)f[2]]);
            a2 += std::abs(triAreaUVSigned(
                uvs[(size_t)f[0]], uvs[(size_t)f[1]], uvs[(size_t)f[2]]));
        }
        if (a2 > 1e-12 && a3 > 0.0) {
            const float s = static_cast<float>(std::sqrt(a3 / a2));
            for (auto& uv : uvs)
                uv *= s;
        }
    }

    // Distortion diagnostics: count faces whose UV winding disagrees with
    // the majority (flips = the chart folds over itself somewhere).
    if (diag) {
        int pos = 0, neg = 0;
        for (const auto& f : mesh.faces) {
            const double a = triAreaUVSigned(
                uvs[(size_t)f[0]], uvs[(size_t)f[1]], uvs[(size_t)f[2]]);
            if (a > 0)
                ++pos;
            else if (a < 0)
                ++neg;
        }
        diag->flippedTriangles = std::min(pos, neg);
        diag->validUvCount = static_cast<int>(uvs.size());
        diag->success = true;
    }
    return uvs;
}

MeshFlattenResult meshToQuadSurfaceByFlattening(
    const core::util::TriMesh& mesh, cv::Vec2f targetScale,
    const ABFConfig& cfg)
{
    MeshFlattenResult result;
    std::vector<cv::Vec2f> uvs = abfFlattenMesh(mesh, cfg, &result.diagnostics);
    if (uvs.empty())
        return result;
    result.flippedTriangles = result.diagnostics.flippedTriangles;

    if (targetScale[0] <= 0.f || targetScale[1] <= 0.f)
        targetScale = cv::Vec2f(0.05f, 0.05f);

    // Grid dims from UV bounds: gridW = uvRange * scale + 1 (UVs are voxels
    // when scaleToOriginalArea is on, so this reproduces segment density).
    float umin = std::numeric_limits<float>::max(), vmin = umin;
    float umax = std::numeric_limits<float>::lowest(), vmax = umax;
    for (const auto& uv : uvs) {
        umin = std::min(umin, uv[0]);
        umax = std::max(umax, uv[0]);
        vmin = std::min(vmin, uv[1]);
        vmax = std::max(vmax, uv[1]);
    }
    const int gw = std::max(
        2, static_cast<int>(std::lround((umax - umin) * targetScale[0])) + 1);
    const int gh = std::max(
        2, static_cast<int>(std::lround((vmax - vmin) * targetScale[1])) + 1);
    try {
        core::util::checkRasterGridSize(cv::Size(gw, gh));
    } catch (const std::exception& e) {
        result.diagnostics.success = false;
        result.diagnostics.failureReason = e.what();
        return result;
    }

    cv::Mat_<cv::Vec3f> points(gh, gw, cv::Vec3f(-1.f, -1.f, -1.f));
    const double ru = std::max(1e-12, static_cast<double>(umax - umin));
    const double rv = std::max(1e-12, static_cast<double>(vmax - vmin));

    for (const auto& f : mesh.faces) {
        cv::Vec2d g[3];
        cv::Vec3f pos[3];
        for (int k = 0; k < 3; ++k) {
            const cv::Vec2f uv = uvs[(size_t)f[k]];
            g[k][0] = (static_cast<double>(uv[0]) - umin) / ru * (gw - 1);
            g[k][1] = (static_cast<double>(uv[1]) - vmin) / rv * (gh - 1);
            pos[k] = mesh.vertices[(size_t)f[k]];
        }
        core::util::rasterizeTriangleExact(
            cv::Size(gw, gh), g,
            [&](int y, int x, float b0, float b1, float b2) {
                if (points(y, x)[0] != -1.f)
                    return;  // first triangle wins
                points(y, x) = b0 * pos[0] + b1 * pos[1] + b2 * pos[2];
            });
    }

    for (int r = 0; r < gh; ++r)
        for (int c = 0; c < gw; ++c)
            if (points(r, c)[0] != -1.f)
                ++result.validCells;
    if (result.validCells == 0) {
        result.diagnostics.success = false;
        result.diagnostics.failureReason = "rasterization covered no cells";
        return result;
    }

    result.surface = std::make_unique<QuadSurface>(points, targetScale);
    return result;
}

}  // namespace vc
