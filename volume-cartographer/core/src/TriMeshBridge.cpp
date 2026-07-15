#include "vc/core/util/TriMeshBridge.hpp"

#include "vc/core/util/Geometry.hpp"
#include "vc/core/util/GridTriangleRaster.hpp"
#include "vc/core/util/QuadSurface.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_map>

namespace vc::core::util
{

namespace
{

inline bool cellValid(const cv::Vec3f& p) { return p[0] != -1.f; }

// Port of vc_tifxyz2obj's build_vertex_normals_from_faces: accumulate the two
// per-quad face normals onto their corners, then normalize.
cv::Mat_<cv::Vec3f> gridVertexNormals(const cv::Mat_<cv::Vec3f>& P)
{
    cv::Mat_<cv::Vec3f> nsum(P.size(), cv::Vec3f(0, 0, 0));
    cv::Mat_<int> ncnt(P.size(), 0);

    for (int j = 0; j < P.rows - 1; ++j) {
        for (int i = 0; i < P.cols - 1; ++i) {
            if (!loc_valid(P, cv::Vec2d(j, i)))
                continue;
            const cv::Vec3f p00 = P(j, i);
            const cv::Vec3f p01 = P(j, i + 1);
            const cv::Vec3f p10 = P(j + 1, i);
            const cv::Vec3f p11 = P(j + 1, i + 1);
            // Winding matches the emitted faces: (c10,c00,c01), (c10,c01,c11).
            const cv::Vec3f n1 = (p00 - p10).cross(p01 - p10);
            const cv::Vec3f n2 = (p01 - p10).cross(p11 - p10);
            auto add = [&](int y, int x, const cv::Vec3f& n) {
                nsum(y, x) += n;
                ncnt(y, x) += 1;
            };
            add(j + 1, i, n1); add(j, i, n1); add(j, i + 1, n1);
            add(j + 1, i, n2); add(j, i + 1, n2); add(j + 1, i + 1, n2);
        }
    }
    for (int y = 0; y < P.rows; ++y) {
        for (int x = 0; x < P.cols; ++x) {
            cv::Vec3f n = nsum(y, x);
            const float L2 = n.dot(n);
            if (ncnt(y, x) > 0 && std::isfinite(L2) && L2 > 1e-20f)
                nsum(y, x) = n / std::sqrt(L2);
            else
                nsum(y, x) = cv::Vec3f(0, 0, 1);
        }
    }
    return nsum;
}

// Exact-bit key for a 3-float position.
struct PosKey {
    uint32_t k[3];
    bool operator==(const PosKey& o) const
    {
        return k[0] == o.k[0] && k[1] == o.k[1] && k[2] == o.k[2];
    }
};
struct PosKeyHash {
    size_t operator()(const PosKey& p) const
    {
        size_t h = 1469598103934665603ull;
        for (uint32_t v : p.k) {
            h ^= v;
            h *= 1099511628211ull;
        }
        return h;
    }
};
inline PosKey posKey(const cv::Vec3f& v)
{
    PosKey p;
    static_assert(sizeof(p.k) == sizeof(cv::Vec3f));
    std::memcpy(p.k, v.val, sizeof(p.k));
    return p;
}

// Squared distance from point p to triangle (a,b,c). Ericson, RTCD 5.1.5.
float pointTriangleDistSq(
    const cv::Vec3f& p, const cv::Vec3f& a, const cv::Vec3f& b,
    const cv::Vec3f& c)
{
    const cv::Vec3f ab = b - a, ac = c - a, ap = p - a;
    const float d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0.f && d2 <= 0.f) {
        const cv::Vec3f d = p - a;
        return d.dot(d);
    }
    const cv::Vec3f bp = p - b;
    const float d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0.f && d4 <= d3) {
        const cv::Vec3f d = p - b;
        return d.dot(d);
    }
    const float vc_ = d1 * d4 - d3 * d2;
    if (vc_ <= 0.f && d1 >= 0.f && d3 <= 0.f) {
        const float v = d1 / (d1 - d3);
        const cv::Vec3f d = p - (a + v * ab);
        return d.dot(d);
    }
    const cv::Vec3f cp = p - c;
    const float d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0.f && d5 <= d6) {
        const cv::Vec3f d = p - c;
        return d.dot(d);
    }
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
        const float w = d2 / (d2 - d6);
        const cv::Vec3f d = p - (a + w * ac);
        return d.dot(d);
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        const cv::Vec3f d = p - (b + w * (c - b));
        return d.dot(d);
    }
    const float denom = 1.f / (va + vb + vc_);
    const float v = vb * denom, w = vc_ * denom;
    const cv::Vec3f d = p - (a + ab * v + ac * w);
    return d.dot(d);
}

}  // namespace

TriMesh meshFromGridPoints(
    const cv::Mat_<cv::Vec3f>& points, const cv::Vec2f& scale,
    cv::Point gridOrigin, GridMeshProvenance* prov,
    const MeshFromGridOptions& opt)
{
    TriMesh mesh;
    if (prov) {
        prov->cell.clear();
        prov->roi = cv::Rect(gridOrigin, points.size());
        prov->scale = scale;
    }
    if (points.rows < 2 || points.cols < 2)
        return mesh;

    const float uv_fac_x =
        (std::isfinite(scale[0]) && scale[0] > 0.f) ? 1.f / scale[0] : 1.f;
    const float uv_fac_y =
        (std::isfinite(scale[1]) && scale[1] > 0.f) ? 1.f / scale[1] : 1.f;

    cv::Mat_<cv::Vec3f> normals;
    if (opt.withNormals)
        normals = gridVertexNormals(points);

    cv::Mat_<int> idxs(points.size(), -1);
    auto addVertex = [&](int j, int i) -> int {
        int& id = idxs(j, i);
        if (id == -1) {
            id = static_cast<int>(mesh.vertices.size());
            mesh.vertices.push_back(points(j, i));
            if (opt.withUVs)
                mesh.uvs.emplace_back(
                    static_cast<float>(gridOrigin.x + i) * uv_fac_x,
                    static_cast<float>(gridOrigin.y + j) * uv_fac_y);
            if (opt.withNormals)
                mesh.normals.push_back(normals(j, i));
            if (prov)
                prov->cell.emplace_back(gridOrigin.y + j, gridOrigin.x + i);
        }
        return id;
    };

    // Two triangles per fully-valid 2x2 quad, exactly like vc_tifxyz2obj's
    // surf_write_obj (same validity rule, same winding).
    for (int j = 0; j < points.rows - 1; ++j) {
        for (int i = 0; i < points.cols - 1; ++i) {
            if (!loc_valid(points, cv::Vec2d(j, i)))
                continue;
            const int c00 = addVertex(j, i);
            const int c01 = addVertex(j, i + 1);
            const int c10 = addVertex(j + 1, i);
            const int c11 = addVertex(j + 1, i + 1);
            mesh.faces.emplace_back(c10, c00, c01);
            mesh.faces.emplace_back(c10, c01, c11);
        }
    }
    return mesh;
}

TriMesh meshFromQuadSurface(
    QuadSurface& surf, GridMeshProvenance* prov, const MeshFromGridOptions& opt)
{
    const cv::Mat_<cv::Vec3f> points = surf.rawPoints();
    cv::Rect roi = opt.roi;
    const cv::Rect full(0, 0, points.cols, points.rows);
    if (roi.width <= 0 || roi.height <= 0)
        roi = full;
    else
        roi &= full;

    MeshFromGridOptions local = opt;
    local.roi = cv::Rect();
    TriMesh mesh =
        meshFromGridPoints(points(roi), surf.scale(), roi.tl(), prov, local);
    if (prov)
        prov->roi = roi;
    return mesh;
}

std::vector<cv::Vec2i> composeProvenance(
    const std::vector<cv::Vec2i>& cellIn, const std::vector<int32_t>& vmap)
{
    std::vector<cv::Vec2i> out(vmap.size(), kNoCell);
    for (size_t i = 0; i < vmap.size(); ++i) {
        const int32_t v = vmap[i];
        if (v >= 0 && static_cast<size_t>(v) < cellIn.size())
            out[i] = cellIn[v];
    }
    return out;
}

std::vector<cv::Vec2i> recoverProvenanceByPosition(
    const TriMesh& piece, const TriMesh& sourceMesh,
    const GridMeshProvenance& sourceProv)
{
    std::vector<cv::Vec2i> out(piece.vertices.size(), kNoCell);
    if (sourceProv.cell.size() != sourceMesh.vertices.size())
        return out;

    // source position -> ascending list of source-vertex indices + cursor
    struct Bucket {
        std::vector<int> idx;
        size_t next = 0;
    };
    std::unordered_map<PosKey, Bucket, PosKeyHash> lookup;
    lookup.reserve(sourceMesh.vertices.size() * 2);
    for (size_t i = 0; i < sourceMesh.vertices.size(); ++i)
        lookup[posKey(sourceMesh.vertices[i])].idx.push_back(
            static_cast<int>(i));

    for (size_t i = 0; i < piece.vertices.size(); ++i) {
        auto it = lookup.find(posKey(piece.vertices[i]));
        if (it == lookup.end())
            continue;
        Bucket& b = it->second;
        // Consume duplicates in ascending order; extra claims (seam-cut
        // duplicated vertices) re-use the first source index.
        const int src = (b.next < b.idx.size()) ? b.idx[b.next++] : b.idx[0];
        out[i] = sourceProv.cell[static_cast<size_t>(src)];
    }
    return out;
}

std::unique_ptr<QuadSurface> writeBackToGrid(
    const TriMesh& piece, const std::vector<cv::Vec2i>& vertexCell,
    const GridMeshProvenance& sourceProv, const WriteBackOptions& opt,
    WriteBackStats* stats)
{
    WriteBackStats local_stats;
    WriteBackStats& st = stats ? *stats : local_stats;
    st = WriteBackStats();

    const size_t nv = piece.vertices.size();
    if (nv == 0 || vertexCell.size() != nv)
        return nullptr;

    cv::Vec2f scale = opt.scale;
    if (scale[0] <= 0.f || scale[1] <= 0.f)
        scale = sourceProv.scale;
    if (scale[0] <= 0.f || scale[1] <= 0.f)
        scale = cv::Vec2f(1.f, 1.f);
    const float nominal_step =
        0.5f * (1.f / scale[0] + 1.f / scale[1]);  // voxels per grid step

    // ── 1. assign grid coordinates to every vertex ─────────────────────────
    // placed: 0 = unknown, 1 = exact (provenance), 2 = relaxed (synthesized).
    std::vector<cv::Vec2d> gpos(nv, cv::Vec2d(0, 0));
    std::vector<uint8_t> placed(nv, 0);
    size_t n_exact = 0;
    for (size_t i = 0; i < nv; ++i) {
        if (vertexCell[i] != kNoCell) {
            gpos[i] = cv::Vec2d(vertexCell[i][1], vertexCell[i][0]);  // (x=col, y=row)
            placed[i] = 1;
            ++n_exact;
        }
    }
    if (n_exact == 0)
        return nullptr;  // nothing anchors this piece to the grid

    // Neighbor relaxation for synthesized vertices (hole-fill Steiner points):
    // Gauss-Seidel over mesh adjacency, inverse-3D-edge-length weights.
    if (n_exact < nv) {
        std::vector<std::vector<int>> adj(nv);
        for (const auto& f : piece.faces) {
            for (int k = 0; k < 3; ++k) {
                const int a = f[k], b = f[(k + 1) % 3];
                if (a >= 0 && b >= 0 && a < (int)nv && b < (int)nv) {
                    adj[a].push_back(b);
                    adj[b].push_back(a);
                }
            }
        }
        // Reachability from the anchored set.
        std::vector<uint8_t> reach(nv, 0);
        std::vector<int> queue;
        for (size_t i = 0; i < nv; ++i) {
            if (placed[i]) {
                reach[i] = 1;
                queue.push_back(static_cast<int>(i));
            }
        }
        for (size_t qi = 0; qi < queue.size(); ++qi) {
            for (int nb : adj[static_cast<size_t>(queue[qi])]) {
                if (!reach[nb]) {
                    reach[nb] = 1;
                    queue.push_back(nb);
                }
            }
        }
        // Initialize reachable unknowns at the mean anchored position.
        cv::Vec2d mean(0, 0);
        for (size_t i = 0; i < nv; ++i) {
            if (placed[i] == 1)
                mean += gpos[i];
        }
        mean *= 1.0 / static_cast<double>(n_exact);
        for (size_t i = 0; i < nv; ++i) {
            if (!placed[i] && reach[i]) {
                gpos[i] = mean;
                placed[i] = 2;
            } else if (!placed[i]) {
                ++st.unplacedVertices;
            }
        }
        // Relax.
        for (int iter = 0; iter < opt.placementIters; ++iter) {
            double max_delta = 0.0;
            for (size_t i = 0; i < nv; ++i) {
                if (placed[i] != 2)
                    continue;
                cv::Vec2d acc(0, 0);
                double wsum = 0.0;
                for (int nb : adj[i]) {
                    if (!placed[static_cast<size_t>(nb)])
                        continue;
                    const cv::Vec3f d =
                        piece.vertices[i] - piece.vertices[(size_t)nb];
                    const double len = std::sqrt((double)d.dot(d));
                    const double w = 1.0 / std::max(len, 1e-6);
                    acc += w * gpos[(size_t)nb];
                    wsum += w;
                }
                if (wsum > 0.0) {
                    const cv::Vec2d next = acc * (1.0 / wsum);
                    max_delta = std::max(
                        max_delta, cv::norm(next - gpos[i], cv::NORM_L2));
                    gpos[i] = next;
                }
            }
            if (max_delta < 1e-4)
                break;
        }
    }

    // ── 2. output rect = bbox of assigned coords (+pad) ────────────────────
    double min_x = std::numeric_limits<double>::max(), min_y = min_x;
    double max_x = std::numeric_limits<double>::lowest(), max_y = max_x;
    for (size_t i = 0; i < nv; ++i) {
        if (!placed[i])
            continue;
        min_x = std::min(min_x, gpos[i][0]);
        max_x = std::max(max_x, gpos[i][0]);
        min_y = std::min(min_y, gpos[i][1]);
        max_y = std::max(max_y, gpos[i][1]);
    }
    const int pad = std::max(0, opt.padCells);
    const int rx = static_cast<int>(std::floor(min_x)) - pad;
    const int ry = static_cast<int>(std::floor(min_y)) - pad;
    const int rw = static_cast<int>(std::ceil(max_x)) - rx + 1 + pad;
    const int rh = static_cast<int>(std::ceil(max_y)) - ry + 1 + pad;
    if (rw <= 0 || rh <= 0)
        return nullptr;
    checkRasterGridSize(cv::Size(rw, rh));
    const cv::Rect rect(rx, ry, rw, rh);
    st.rect = rect;

    cv::Mat_<cv::Vec3f> points(rect.size(), cv::Vec3f(-1, -1, -1));
    cv::Mat_<int> claim(rect.size(), -1);

    // Source positions for the conflict tie-break (optional).
    cv::Mat_<cv::Vec3f> src_points;
    if (opt.channelSource)
        src_points =
            const_cast<QuadSurface*>(opt.channelSource)->rawPoints();

    // ── 3. exact writes ─────────────────────────────────────────────────────
    for (size_t i = 0; i < nv; ++i) {
        if (placed[i] != 1)
            continue;
        const int lc = vertexCell[i][1] - rect.x;
        const int lr = vertexCell[i][0] - rect.y;
        if (lc < 0 || lr < 0 || lc >= rect.width || lr >= rect.height)
            continue;
        if (claim(lr, lc) < 0) {
            points(lr, lc) = piece.vertices[i];
            claim(lr, lc) = static_cast<int>(i);
        } else {
            const cv::Vec3f diff = points(lr, lc) - piece.vertices[i];
            if (std::sqrt(diff.dot(diff)) > opt.conflictEps) {
                ++st.conflictCells;
                // Prefer the claim closest to the source surface's value.
                if (!src_points.empty() && vertexCell[i][0] >= 0 &&
                    vertexCell[i][0] < src_points.rows &&
                    vertexCell[i][1] >= 0 &&
                    vertexCell[i][1] < src_points.cols) {
                    const cv::Vec3f ref =
                        src_points(vertexCell[i][0], vertexCell[i][1]);
                    if (cellValid(ref)) {
                        const cv::Vec3f d_old = points(lr, lc) - ref;
                        const cv::Vec3f d_new = piece.vertices[i] - ref;
                        if (d_new.dot(d_new) < d_old.dot(d_old)) {
                            points(lr, lc) = piece.vertices[i];
                            claim(lr, lc) = static_cast<int>(i);
                        }
                    }
                }
            }
        }
    }
    for (int r = 0; r < rect.height; ++r)
        for (int c = 0; c < rect.width; ++c)
            if (claim(r, c) >= 0)
                ++st.provenancedCells;

    // ── 4. raster fill of still-invalid cells ──────────────────────────────
    for (const auto& f : piece.faces) {
        bool ok = true;
        for (int k = 0; k < 3; ++k) {
            if (f[k] < 0 || f[k] >= (int)nv || !placed[(size_t)f[k]])
                ok = false;
        }
        if (!ok) {
            ++st.droppedTriangles;
            continue;
        }
        // Stretch gate: 3D size vs grid-space size must roughly agree.
        double max_3d = 0.0, max_grid = 0.0;
        for (int k = 0; k < 3; ++k) {
            const int a = f[k], b = f[(k + 1) % 3];
            const cv::Vec3f d3 =
                piece.vertices[(size_t)a] - piece.vertices[(size_t)b];
            max_3d = std::max(max_3d, std::sqrt((double)d3.dot(d3)));
            max_grid = std::max(
                max_grid, cv::norm(gpos[(size_t)a] - gpos[(size_t)b]));
        }
        if (max_grid <= 1e-12 ||
            max_3d > (double)opt.maxStretch * max_grid * nominal_step) {
            ++st.droppedTriangles;
            continue;
        }

        cv::Vec2d g[3];
        cv::Vec3f pos[3];
        for (int k = 0; k < 3; ++k) {
            g[k] = gpos[(size_t)f[k]] - cv::Vec2d(rect.x, rect.y);
            pos[k] = piece.vertices[(size_t)f[k]];
        }
        rasterizeTriangleExact(
            rect.size(), g, [&](int y, int x, float b0, float b1, float b2) {
                if (cellValid(points(y, x)))
                    return;  // exact writes and earlier triangles win
                points(y, x) = b0 * pos[0] + b1 * pos[1] + b2 * pos[2];
                ++st.rasterizedCells;
            });
    }

    // ── 5. stats ────────────────────────────────────────────────────────────
    double step_sum = 0.0;
    int step_n = 0;
    for (int r = 0; r < rect.height; ++r) {
        for (int c = 0; c < rect.width; ++c) {
            if (!cellValid(points(r, c)))
                continue;
            ++st.validCells;
            if (c + 1 < rect.width && cellValid(points(r, c + 1))) {
                const cv::Vec3f d = points(r, c + 1) - points(r, c);
                const float len = std::sqrt(d.dot(d));
                st.maxNeighborStep = std::max(st.maxNeighborStep, len);
                step_sum += len;
                ++step_n;
            }
            if (r + 1 < rect.height && cellValid(points(r + 1, c))) {
                const cv::Vec3f d = points(r + 1, c) - points(r, c);
                const float len = std::sqrt(d.dot(d));
                st.maxNeighborStep = std::max(st.maxNeighborStep, len);
                step_sum += len;
                ++step_n;
            }
        }
    }
    st.meanNeighborStep =
        step_n ? static_cast<float>(step_sum / step_n) : 0.f;
    if (st.validCells == 0)
        return nullptr;

    auto surf = std::make_unique<QuadSurface>(points, scale);

    // ── 6. channels (same absolute grid frame; nearest sample) ─────────────
    if (opt.channelSource && !src_points.empty()) {
        for (const auto& name : opt.channels) {
            cv::Mat src = const_cast<QuadSurface*>(opt.channelSource)
                              ->channel(name, SURF_CHANNEL_NORESIZE);
            if (src.empty())
                continue;
            const double ratio_r =
                static_cast<double>(src.rows) / src_points.rows;
            const double ratio_c =
                static_cast<double>(src.cols) / src_points.cols;
            cv::Mat out(rect.height, rect.width, src.type(), cv::Scalar::all(0));
            for (int r = 0; r < rect.height; ++r) {
                for (int c = 0; c < rect.width; ++c) {
                    if (claim(r, c) < 0)
                        continue;  // only provenanced cells carry channels
                    const int sr = std::clamp(
                        (int)std::lround((rect.y + r) * ratio_r), 0,
                        src.rows - 1);
                    const int sc = std::clamp(
                        (int)std::lround((rect.x + c) * ratio_c), 0,
                        src.cols - 1);
                    std::memcpy(
                        out.ptr(r, c), src.ptr(sr, sc), src.elemSize());
                }
            }
            surf->setChannel(name, out);
        }
    }
    return surf;
}

MeshGridAudit auditGridAgainstMesh(
    const QuadSurface& s, const TriMesh& m, int samples)
{
    MeshGridAudit audit;
    if (m.faces.empty())
        return audit;
    const cv::Mat_<cv::Vec3f> points =
        const_cast<QuadSurface&>(s).rawPoints();

    // Collect valid cells, then stride-sample.
    std::vector<cv::Vec3f> pts;
    for (int r = 0; r < points.rows; ++r)
        for (int c = 0; c < points.cols; ++c)
            if (cellValid(points(r, c)))
                pts.push_back(points(r, c));
    if (pts.empty())
        return audit;
    const size_t stride =
        std::max<size_t>(1, pts.size() / std::max(1, samples));

    double sum = 0.0;
    for (size_t i = 0; i < pts.size(); i += stride) {
        float best = std::numeric_limits<float>::max();
        for (const auto& f : m.faces) {
            best = std::min(
                best, pointTriangleDistSq(
                          pts[i], m.vertices[(size_t)f[0]],
                          m.vertices[(size_t)f[1]],
                          m.vertices[(size_t)f[2]]));
        }
        const float d = std::sqrt(best);
        audit.maxDist = std::max(audit.maxDist, d);
        sum += d;
        ++audit.samples;
    }
    audit.meanDist =
        audit.samples ? static_cast<float>(sum / audit.samples) : 0.f;
    return audit;
}

}  // namespace vc::core::util
