#pragma once

// Exact triangle scan-conversion onto a regular grid.
//
// Extracted from vc_obj2tifxyz's ObjToTifxyzConverter::rasterizeTriangle so
// the same crack-free coverage is available in-process (TriMeshBridge write-
// back, flatten-to-grid) and to the CLI. Vertices are snapped to a fixed-
// point sub-pixel lattice and inclusion is decided by integer edge functions
// with the top-left fill rule: no float rounding, identical on every
// architecture, and a pixel on a shared edge is claimed by exactly one of
// the two adjacent triangles — a grid-aligned mesh tiles without gaps or
// double-claims.

#include <opencv2/core.hpp>

#include <functional>

namespace vc::core::util
{

// Sub-pixel resolution for fixed-point vertex snapping (8 fractional bits).
inline constexpr long long kRasterSubpixel = 256;

// Throws std::runtime_error if a grid dimension is too large for the exact
// int64 edge functions (~8.4M px per axis — far above any real tifxyz grid).
void checkRasterGridSize(cv::Size gridSize);

// Rasterize one triangle. g[k] is corner k's position in grid space
// (g[k][0] = x/col, g[k][1] = y/row). For every covered pixel the visitor is
// called as visit(row, col, b0, b1, b2) with barycentric weights that match
// the CALLER's corner order (winding normalization for the fill rule is
// internal). Degenerate (zero-area) triangles cover nothing. The visitor
// decides what to write and how conflicts resolve (e.g. first-wins).
void rasterizeTriangleExact(
    cv::Size gridSize, const cv::Vec2d g[3],
    const std::function<void(int, int, float, float, float)>& visit);

}  // namespace vc::core::util
