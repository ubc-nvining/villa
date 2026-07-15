#include "vc/core/util/GridTriangleRaster.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace vc::core::util
{

namespace
{

// Twice the signed area of triangle (A, B, P): (B-A) x (P-A). Exact in int64.
inline long long edgeFn(
    long long ax, long long ay, long long bx, long long by, long long px,
    long long py)
{
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

// Top-left edge predicate for the fill rule. Antisymmetric under edge
// reversal, so the two triangles sharing an edge make opposite choices and
// each on-edge pixel is claimed exactly once.
inline bool isTopLeft(long long dx, long long dy)
{
    return dy < 0 || (dy == 0 && dx < 0);
}

}  // namespace

void checkRasterGridSize(cv::Size gridSize)
{
    // Edge functions are evaluated on fixed-point coordinates up to
    // (grid-1)*kRasterSubpixel; keep 2*L^2 below INT64_MAX so the signed-area
    // computation cannot overflow.
    const long long max_dim = std::max(gridSize.width, gridSize.height);
    if (max_dim * kRasterSubpixel >= 2'000'000'000LL) {
        throw std::runtime_error(
            "grid dimension too large for exact int64 rasterization: " +
            std::to_string(max_dim));
    }
}

void rasterizeTriangleExact(
    cv::Size gridSize, const cv::Vec2d g[3],
    const std::function<void(int, int, float, float, float)>& visit)
{
    // Snap each corner to the fixed-point sub-pixel lattice.
    long long X[3], Y[3];
    for (int k = 0; k < 3; ++k) {
        X[k] = std::llround(g[k][0] * kRasterSubpixel);
        Y[k] = std::llround(g[k][1] * kRasterSubpixel);
    }

    // Twice the signed area. Zero -> degenerate (covers nothing). Normalize
    // to CCW so the top-left rule has a consistent orientation; perm maps the
    // (possibly swapped) internal corner order back to the caller's.
    int perm[3] = {0, 1, 2};
    long long area2 = edgeFn(X[0], Y[0], X[1], Y[1], X[2], Y[2]);
    if (area2 == 0) {
        return;
    }
    if (area2 < 0) {
        std::swap(X[1], X[2]);
        std::swap(Y[1], Y[2]);
        std::swap(perm[1], perm[2]);
        area2 = -area2;
    }

    // Top-left bias per edge: a pixel exactly on an edge (w == 0) is included
    // only when that edge is top-left; folding the bias into the comparison
    // turns the rule into a single (w + bias) >= 0 test.
    const long long bias0 = isTopLeft(X[2] - X[1], Y[2] - Y[1]) ? 0 : -1;
    const long long bias1 = isTopLeft(X[0] - X[2], Y[0] - Y[2]) ? 0 : -1;
    const long long bias2 = isTopLeft(X[1] - X[0], Y[1] - Y[0]) ? 0 : -1;

    // Bounding box on the pixel grid.
    const int min_x = std::max(
        0, static_cast<int>(std::floor(std::min({g[0][0], g[1][0], g[2][0]}))));
    const int max_x = std::min(
        gridSize.width - 1,
        static_cast<int>(std::ceil(std::max({g[0][0], g[1][0], g[2][0]}))));
    const int min_y = std::max(
        0, static_cast<int>(std::floor(std::min({g[0][1], g[1][1], g[2][1]}))));
    const int max_y = std::min(
        gridSize.height - 1,
        static_cast<int>(std::ceil(std::max({g[0][1], g[1][1], g[2][1]}))));

    const double inv_area = 1.0 / static_cast<double>(area2);
    float b[3];
    for (int y = min_y; y <= max_y; y++) {
        const long long Py = static_cast<long long>(y) * kRasterSubpixel;
        for (int x = min_x; x <= max_x; x++) {
            const long long Px = static_cast<long long>(x) * kRasterSubpixel;
            const long long w0 = edgeFn(X[1], Y[1], X[2], Y[2], Px, Py);
            const long long w1 = edgeFn(X[2], Y[2], X[0], Y[0], Px, Py);
            const long long w2 = edgeFn(X[0], Y[0], X[1], Y[1], Px, Py);
            if ((w0 + bias0) < 0 || (w1 + bias1) < 0 || (w2 + bias2) < 0) {
                continue;
            }
            // w_k is the weight of internal corner k; map back to the
            // caller's corner order through perm.
            b[perm[0]] = static_cast<float>(w0 * inv_area);
            b[perm[1]] = static_cast<float>(w1 * inv_area);
            b[perm[2]] = static_cast<float>(w2 * inv_area);
            visit(y, x, b[0], b[1], b[2]);
        }
    }
}

}  // namespace vc::core::util
