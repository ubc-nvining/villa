#pragma once

// TriMeshBridge — QuadSurface <-> triangle-mesh conversion with per-vertex
// grid provenance.
//
// VC3D's native surface is a regular grid of 3D points (QuadSurface); external
// mesh processors (ScrollFiesta) operate on indexed triangle meshes. Because
// the triangles we hand out are generated FROM the grid, every vertex carries
// its source (row,col); after an operation, surviving vertices write their
// (possibly moved) positions straight back into their cells, newly created
// vertices are placed by neighbor relaxation, covered-but-unclaimed cells are
// filled by exact triangle rasterization, and never-claimed cells become
// invalid — which is precisely how deletions (culled components, cut seam
// bands) are represented on a grid. The triangulation rule and UV convention
// match vc_tifxyz2obj (two triangles per fully-valid 2x2 quad, u = col/scale,
// v = row/scale), so meshes produced here interoperate with the existing OBJ
// tooling.

#include <opencv2/core.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class QuadSurface;

namespace vc::core::util
{

// "No source cell" marker for provenance entries (newly created vertices).
inline const cv::Vec2i kNoCell{-1, -1};

// Indexed triangle mesh in volume space (x,y,z voxel units).
struct TriMesh {
    std::vector<cv::Vec3f> vertices;
    std::vector<cv::Vec3i> faces;    // 0-based; winding matches vc_tifxyz2obj
    std::vector<cv::Vec3f> normals;  // optional (empty ok)
    std::vector<cv::Vec2f> uvs;      // optional; u = col/scale.x, v = row/scale.y
};

// Where a grid-born mesh came from: per-vertex source cell in ABSOLUTE grid
// coordinates of the source surface, the rect it was built from, and the
// source scale (grid cells per voxel).
struct GridMeshProvenance {
    std::vector<cv::Vec2i> cell;  // per-vertex (row,col); kNoCell = synthesized
    cv::Rect roi;                 // source rect (x=col0, y=row0, w, h)
    cv::Vec2f scale{0.f, 0.f};
};

struct MeshFromGridOptions {
    cv::Rect roi{};           // empty = full grid
    bool withNormals = false; // face-accumulated per-vertex normals
    bool withUVs = true;      // grid-metric UVs (vc_tifxyz2obj convention)
};

// QuadSurface (ROI) -> triangle mesh + provenance. Two triangles per fully-
// valid 2x2 quad (loc_valid rule), vertices exactly at cell positions.
TriMesh meshFromQuadSurface(
    QuadSurface& surf, GridMeshProvenance* prov,
    const MeshFromGridOptions& opt = {});

// Same, over a raw points grid. gridOrigin is the grid's absolute offset
// (so provenance/UVs stay in the source frame when points is a crop).
TriMesh meshFromGridPoints(
    const cv::Mat_<cv::Vec3f>& points, const cv::Vec2f& scale,
    cv::Point gridOrigin, GridMeshProvenance* prov,
    const MeshFromGridOptions& opt = {});

struct WriteBackOptions {
    cv::Vec2f scale{0.f, 0.f};  // <=0: inherit provenance scale
    int padCells = 2;           // rect padding for interpolated new geometry
    // Reject triangles whose 3D size disagrees with their grid-space size by
    // more than this factor (stops bad provenance from smearing across the
    // grid): max3DEdge / (maxGridEdge * nominalStep) > maxStretch -> dropped.
    float maxStretch = 4.f;
    // Position spread (voxels) tolerated between multiple provenance claims
    // on one cell before it counts as a conflict.
    float conflictEps = 1e-3f;
    // Gauss-Seidel iterations for placing provenance-free vertices.
    int placementIters = 200;
    // Resample these channels from channelSource onto the output (nearest,
    // same absolute grid frame). Ignored when channelSource is null.
    const QuadSurface* channelSource = nullptr;
    std::vector<std::string> channels{"approval", "d"};
};

struct WriteBackStats {
    cv::Rect rect{};           // output grid's rect in the SOURCE grid frame
    int validCells = 0;        // valid cells in the output grid
    int provenancedCells = 0;  // written exactly from a provenanced vertex
    int rasterizedCells = 0;   // filled by triangle rasterization
    int conflictCells = 0;     // >1 claim with position spread > conflictEps
    int unplacedVertices = 0;  // synthesized verts with no anchored island
    int droppedTriangles = 0;  // stretch-gate / unplaced-corner rejections
    float maxNeighborStep = 0.f;   // max 3D distance between adjacent cells
    float meanNeighborStep = 0.f;
};

// One op-result piece -> QuadSurface. vertexCell[i] = ABSOLUTE source
// (row,col) of piece vertex i, or kNoCell. The output grid rect is the bbox
// of the assigned cells (+pad), so the piece stays in the source's row/col
// frame (comparable/overlayable with the source segment). Returns nullptr
// when nothing could be placed.
std::unique_ptr<QuadSurface> writeBackToGrid(
    const TriMesh& piece, const std::vector<cv::Vec2i>& vertexCell,
    const GridMeshProvenance& sourceProv, const WriteBackOptions& opt = {},
    WriteBackStats* stats = nullptr);

// Compose an op's vertex map with the input's provenance:
// out[i] = vmap[i] < 0 ? kNoCell : cellIn[vmap[i]].
std::vector<cv::Vec2i> composeProvenance(
    const std::vector<cv::Vec2i>& cellIn, const std::vector<int32_t>& vmap);

// Recover provenance for ops that provide no vertex map but copy surviving
// positions bit-verbatim (all ScrollFiesta split/cleanup ops do): exact
// float-bit position join of piece vertices against the source mesh.
// Duplicate positions are consumed in ascending source order; unmatched
// vertices get kNoCell.
std::vector<cv::Vec2i> recoverProvenanceByPosition(
    const TriMesh& piece, const TriMesh& sourceMesh,
    const GridMeshProvenance& sourceProv);

// Cheap QC: sampled distance from valid output cells to the piece mesh.
struct MeshGridAudit {
    float maxDist = 0.f;
    float meanDist = 0.f;
    int samples = 0;
};
MeshGridAudit auditGridAgainstMesh(
    const QuadSurface& s, const TriMesh& m, int samples = 5000);

}  // namespace vc::core::util
