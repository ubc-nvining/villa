#pragma once

// MeshFlatten — ABF++/LSCM parameterization of an arbitrary triangle mesh,
// and the composition that turns a parameterized mesh into a QuadSurface.
//
// This is the no-provenance fallback of the ScrollFiesta integration: when a
// mesh was RECONSTRUCTED (BPA, the volume pipeline, an external weld) there
// are no source grid cells to write back to, so the mesh is flattened to a
// single low-distortion chart and rasterized into a fresh regular grid —
// which is exactly what the tifxyz representation is: a UV-grid sampling.
//
// The input must be a manifold disk-topology sheet (one connected component,
// one boundary loop, genus 0). ScrollFiesta's cleanup + cut_to_disk ops
// produce exactly that; on unsuitable input these functions fail with a
// reason in ABFDiagnostics::failureReason rather than trying to repair.

#include "vc/core/util/TriMeshBridge.hpp"
#include "vc/flattening/ABFFlattening.hpp"

#include <memory>
#include <vector>

class QuadSurface;

namespace vc
{

// Per-vertex UVs for `mesh` (ABF++ then LSCM per config; cfg.useABF = false
// or a face count above lscmOnlyFaceThreshold -> LSCM only). Empty on
// failure, with diag->failureReason set when diag is provided. When
// cfg.scaleToOriginalArea is set (default) the UVs are metric: total UV area
// equals total 3D area, so UV units are voxels.
std::vector<cv::Vec2f> abfFlattenMesh(
    const core::util::TriMesh& mesh, const ABFConfig& cfg = {},
    ABFDiagnostics* diag = nullptr,
    std::size_t lscmOnlyFaceThreshold = 300000);

struct MeshFlattenResult {
    std::unique_ptr<QuadSurface> surface;
    ABFDiagnostics diagnostics;
    int validCells = 0;
    int flippedTriangles = 0;   // UV-flipped faces (distortion red flag)
};

// Flatten `mesh` and rasterize it into a new QuadSurface at `targetScale`
// grid cells per voxel (<=0 -> 0.05, the segment default). Returns a null
// surface on failure (see diagnostics.failureReason).
MeshFlattenResult meshToQuadSurfaceByFlattening(
    const core::util::TriMesh& mesh, cv::Vec2f targetScale = {0.f, 0.f},
    const ABFConfig& cfg = {});

}  // namespace vc
