#pragma once

// FiestaOps — QuadSurface-level ScrollFiesta operations.
//
// Composes TriMeshBridge (grid <-> triangle mesh with per-vertex provenance)
// with the runtime-loaded ScrollFiesta table: crop the grid ROI, hand the
// triangles to ScrollFiesta, and write results back into the source grid
// frame. Splits return one cropped QuadSurface per piece (rects may overlap —
// pieces are meant to become separate segments). All functions throw
// FiestaError / FiestaCancelled on failure and require the library to be
// loaded (check FiestaRuntime::instance().available() first for UI flows).

#include "vc/core/fiesta/FiestaBridge.hpp"

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

class QuadSurface;

namespace vc::fiesta
{

struct AuditReport {
    sf_topology_report topo{};
    // Human-readable multi-line summary for dialogs/console.
    std::string pretty() const;
    // Compact JSON object (hand-rolled; no dependency).
    std::string json() const;
};

// Read-only topology audit of a surface (ROI empty = whole grid).
AuditReport auditQuadSurface(
    QuadSurface& surf, const cv::Rect& roi = {}, ProgressFn progress = {});

struct CleanupResult {
    std::unique_ptr<QuadSurface> surface;   // the cleaned ROI, source frame
    core::util::WriteBackStats writeback;
    sf_cleanup_report report{};
    AuditReport before, after;
};

// Run ScrollFiesta's cleanup bundle on a grid ROI. cfg = nullptr uses the
// library defaults. The result surface lives in the source grid frame
// (writeback.rect says where); the caller decides in-place commit vs new
// segment.
CleanupResult cleanupQuadSurfaceRoi(
    QuadSurface& surf, const cv::Rect& roi = {},
    const sf_cleanup_config* cfg = nullptr, ProgressFn progress = {});

struct SplitPiece {
    std::unique_ptr<QuadSurface> surface;
    core::util::WriteBackStats writeback;
};

struct DetangleResult {
    std::vector<SplitPiece> pieces;
    sf_detangle_report report{};
};

// Run the detangle cascade (depth peel -> gated developability cut -> bridge
// cut -> overlap separation) on a grid ROI. Each output piece becomes its own
// QuadSurface in the source frame. cfg = nullptr uses library defaults.
DetangleResult detangleQuadSurface(
    QuadSurface& surf, const cv::Rect& roi = {},
    const sf_detangle_config* cfg = nullptr, ProgressFn progress = {});

}  // namespace vc::fiesta
