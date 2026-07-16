// End-to-end FiestaBridge/FiestaOps test: loads the runtime-built
// scrollfiesta library (staged next to the test executable), runs audit /
// cleanup / detangle on a synthetic QuadSurface, and checks the round trip.
// Skips (with a message) when the library is absent — its presence is
// guaranteed in VC_WITH_SCROLLFIESTA builds via the vc_fiesta dependency.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "vc/core/fiesta/FiestaOps.hpp"
#include "vc/core/util/QuadSurface.hpp"

#include <cstdlib>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

using namespace vc::fiesta;

namespace
{

// Gently curved sheet, grid step ~20 voxels (scale 0.05) like real segments.
cv::Mat_<cv::Vec3f> makeSheet(int rows, int cols)
{
    cv::Mat_<cv::Vec3f> P(rows, cols);
    for (int j = 0; j < rows; ++j)
        for (int i = 0; i < cols; ++i)
            P(j, i) = cv::Vec3f(
                1000.f + i * 20.f, 2000.f + j * 20.f,
                500.f + 0.4f * i + 0.2f * j + 0.02f * i * j);
    return P;
}

}  // namespace

TEST_CASE("fiesta runtime + ops end to end")
{
    auto& rt = FiestaRuntime::instance();
    if (!rt.available()) {
        std::cout << "scrollfiesta library unavailable - skipping: "
                  << rt.unavailableReason() << "\n";
        return;
    }
    std::cout << "loaded " << rt.versionString() << "\n";

    cv::Mat_<cv::Vec3f> P = makeSheet(12, 12);
    QuadSurface surf(P, cv::Vec2f(0.05f, 0.05f));

    SUBCASE("audit of a clean sheet")
    {
        AuditReport report = auditQuadSurface(surf);
        CHECK(report.topo.n_components == 1);
        CHECK(report.topo.n_nonmanifold_edges == 0);
        CHECK(report.topo.n_boundary_loops == 1);
        CHECK(report.topo.is_disk == 1);
        CHECK(!report.pretty().empty());
        CHECK(report.json().front() == '{');
    }

    SUBCASE("cleanup round trip keeps a clean sheet intact")
    {
        CleanupResult result = cleanupQuadSurfaceRoi(surf);
        REQUIRE(result.surface);
        CHECK(result.writeback.validCells > 0);
        CHECK(result.writeback.conflictCells == 0);
        CHECK(result.after.topo.n_nonmanifold_edges == 0);

        // Quad-supported cells must be bit-identical after the round trip.
        const cv::Mat_<cv::Vec3f> O = result.surface->rawPoints();
        const auto& rect = result.writeback.rect;
        int checked = 0;
        for (int r = 0; r < O.rows; ++r) {
            for (int c = 0; c < O.cols; ++c) {
                if (O(r, c)[0] == -1.f)
                    continue;
                const int sr = rect.y + r;
                const int sc = rect.x + c;
                REQUIRE(sr >= 0);
                REQUIRE(sc >= 0);
                REQUIRE(sr < P.rows);
                REQUIRE(sc < P.cols);
                CHECK(std::memcmp(
                          O(r, c).val, P(sr, sc).val, sizeof(float) * 3) == 0);
                ++checked;
            }
        }
        CHECK(checked > 0);
    }

    SUBCASE("detangle self-gates a single flat sheet to one piece")
    {
        DetangleResult result = detangleQuadSurface(surf);
        CHECK(result.pieces.size() == 1);
        REQUIRE(result.pieces[0].surface);
        CHECK(result.pieces[0].writeback.validCells > 0);
    }

    SUBCASE("detangle mass-retention guard")
    {
        // A clean self-gating sheet retains all its mass and does not trip the
        // guard; retainedFraction is reported.
        DetangleResult kept = detangleQuadSurface(surf, {}, nullptr, {}, 0.5);
        CHECK(kept.retainedFraction == doctest::Approx(1.0));

        // The guard fires whenever the result keeps less than the requested
        // fraction of the input. Demand an impossible >100% retention to
        // exercise the throw path deterministically (independent of what the
        // splitters happen to do to any given synthetic mesh): a normal
        // 100%-retained result is below the 1.5 threshold, so it must throw
        // FiestaError rather than proceed. This is the exact code path that
        // refuses the real-data case where detangle keeps ~0.1% of a segment.
        CHECK_THROWS_AS(
            detangleQuadSurface(surf, {}, nullptr, {}, 1.5), FiestaError);
    }

    SUBCASE("progress callback cancellation")
    {
        CHECK_THROWS_AS(
            cleanupQuadSurfaceRoi(
                surf, {}, nullptr,
                [](float, const char*) { return false; }),
            FiestaCancelled);
    }

    SUBCASE("ROI restricts the operation")
    {
        AuditReport full = auditQuadSurface(surf);
        AuditReport part = auditQuadSurface(surf, cv::Rect(2, 2, 6, 6));
        CHECK(part.topo.n_verts < full.topo.n_verts);
        CHECK(part.topo.n_components == 1);
    }
}

// Not a check — a fixture generator for manual CLI smoke tests. Runs only
// when VC_FIESTA_SAVE_FIXTURE=<dir> is set in the environment.
TEST_CASE("save synthetic tifxyz fixture")
{
    const char* out = std::getenv("VC_FIESTA_SAVE_FIXTURE");
    if (!out)
        return;  // silent no-op in normal test runs
    cv::Mat_<cv::Vec3f> P = makeSheet(24, 24);
    QuadSurface surf(P, cv::Vec2f(0.05f, 0.05f));
    surf.save(out, "fiesta_fixture");
    std::cout << "wrote " << out << "\n";
}
