// MeshFlatten: a developable cylinder sector must flatten without flips and
// rasterize into a dense, geometrically-faithful grid.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "vc/core/util/QuadSurface.hpp"
#include "vc/flattening/MeshFlatten.hpp"

#include <cmath>

using namespace vc;
using vc::core::util::TriMesh;

namespace
{

// Cylinder sector: radius 50 vox, 1.2 rad arc, 100 vox tall — developable by
// construction, like a clean scroll wrap.
TriMesh makeCylinderSector(int nTheta, int nH)
{
    TriMesh mesh;
    const float radius = 50.f;
    for (int j = 0; j < nH; ++j) {
        for (int i = 0; i < nTheta; ++i) {
            const float theta = 1.2f * i / (nTheta - 1);
            mesh.vertices.emplace_back(
                radius * std::cos(theta), radius * std::sin(theta),
                100.f * j / (nH - 1));
        }
    }
    for (int j = 0; j + 1 < nH; ++j) {
        for (int i = 0; i + 1 < nTheta; ++i) {
            const int v00 = j * nTheta + i, v10 = j * nTheta + i + 1;
            const int v01 = (j + 1) * nTheta + i, v11 = (j + 1) * nTheta + i + 1;
            mesh.faces.emplace_back(v00, v10, v01);
            mesh.faces.emplace_back(v10, v11, v01);
        }
    }
    return mesh;
}

}  // namespace

TEST_CASE("cylinder sector flattens flip-free and rasterizes faithfully")
{
    TriMesh mesh = makeCylinderSector(24, 20);

    SUBCASE("UVs are metric and flip-free")
    {
        ABFDiagnostics diag;
        auto uvs = abfFlattenMesh(mesh, {}, &diag);
        REQUIRE(!uvs.empty());
        CHECK(diag.success);
        CHECK(diag.flippedTriangles == 0);
        // Metric UVs: the unrolled sector is (arc 60) x (height 100) voxels.
        float umin = 1e9f, umax = -1e9f, vmin = 1e9f, vmax = -1e9f;
        for (const auto& uv : uvs) {
            umin = std::min(umin, uv[0]);
            umax = std::max(umax, uv[0]);
            vmin = std::min(vmin, uv[1]);
            vmax = std::max(vmax, uv[1]);
        }
        const float extents[2] = {umax - umin, vmax - vmin};
        const float lo = std::min(extents[0], extents[1]);
        const float hi = std::max(extents[0], extents[1]);
        CHECK(lo == doctest::Approx(60.f).epsilon(0.08));
        CHECK(hi == doctest::Approx(100.f).epsilon(0.08));
    }

    SUBCASE("rasterized surface lies on the cylinder")
    {
        MeshFlattenResult result =
            meshToQuadSurfaceByFlattening(mesh, cv::Vec2f(0.5f, 0.5f));
        REQUIRE(result.surface);
        CHECK(result.flippedTriangles == 0);
        CHECK(result.validCells > 500);   // ~31 x 51 grid, mostly covered

        const cv::Mat_<cv::Vec3f> P = result.surface->rawPoints();
        int checked = 0;
        for (int r = 0; r < P.rows; ++r) {
            for (int c = 0; c < P.cols; ++c) {
                const cv::Vec3f& p = P(r, c);
                if (p[0] == -1.f)
                    continue;
                const float radial = std::sqrt(p[0] * p[0] + p[1] * p[1]);
                CHECK(radial == doctest::Approx(50.f).epsilon(0.02));
                CHECK(p[2] >= -1.f);
                CHECK(p[2] <= 101.f);
                ++checked;
            }
        }
        CHECK(checked > 500);
    }
}
