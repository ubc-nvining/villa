#include "vc/core/fiesta/FiestaOps.hpp"

#include "vc/core/util/QuadSurface.hpp"

#include <sstream>

namespace vc::fiesta
{

namespace util = vc::core::util;

namespace
{

// Grid ROI -> TriMesh + provenance, with a helpful error on empty input.
util::TriMesh meshOf(
    QuadSurface& surf, const cv::Rect& roi, util::GridMeshProvenance& prov)
{
    util::MeshFromGridOptions opt;
    opt.roi = roi;
    opt.withUVs = false;  // in-process path carries provenance instead
    util::TriMesh mesh = util::meshFromQuadSurface(surf, &prov, opt);
    if (mesh.faces.empty())
        throw FiestaError(
            SF_ERROR_BAD_ARG,
            "selected region contains no valid 2x2 quads to mesh");
    return mesh;
}

// Provenance for an op output: the op's vmap when present (composed with the
// input provenance), else exact-position recovery against the input mesh.
std::vector<cv::Vec2i> provenanceFor(
    const sf_mesh& out, const util::TriMesh& outMesh,
    const util::TriMesh& inMesh, const util::GridMeshProvenance& prov)
{
    const auto vmap = vmapOf(out);
    if (!vmap.empty())
        return util::composeProvenance(prov.cell, vmap);
    return util::recoverProvenanceByPosition(outMesh, inMesh, prov);
}

}  // namespace

std::string AuditReport::pretty() const
{
    std::ostringstream os;
    os << "vertices:            " << topo.n_verts << "\n"
       << "faces:               " << topo.n_faces << "\n"
       << "edges:               " << topo.n_edges << "\n"
       << "components:          " << topo.n_components << "\n"
       << "boundary loops:      " << topo.n_boundary_loops << "\n"
       << "boundary edges:      " << topo.n_boundary_edges << "\n"
       << "non-manifold edges:  " << topo.n_nonmanifold_edges << "\n"
       << "unreferenced verts:  " << topo.n_unref_verts << "\n"
       << "euler characteristic:" << topo.euler << "\n"
       << "genus:               " << topo.genus << "\n"
       << "topological disk:    " << (topo.is_disk ? "yes" : "no") << "\n";
    return os.str();
}

std::string AuditReport::json() const
{
    std::ostringstream os;
    os << "{\"vertices\":" << topo.n_verts << ",\"faces\":" << topo.n_faces
       << ",\"edges\":" << topo.n_edges
       << ",\"components\":" << topo.n_components
       << ",\"boundary_loops\":" << topo.n_boundary_loops
       << ",\"boundary_edges\":" << topo.n_boundary_edges
       << ",\"nonmanifold_edges\":" << topo.n_nonmanifold_edges
       << ",\"unref_verts\":" << topo.n_unref_verts
       << ",\"euler\":" << topo.euler << ",\"genus\":" << topo.genus
       << ",\"is_disk\":" << (topo.is_disk ? "true" : "false") << "}";
    return os.str();
}

AuditReport auditQuadSurface(
    QuadSurface& surf, const cv::Rect& roi, ProgressFn progress)
{
    const sf_api* api = FiestaRuntime::instance().require();

    util::GridMeshProvenance prov;
    util::TriMesh mesh = meshOf(surf, roi, prov);
    const sf_mesh in = viewOf(mesh);

    ProgressTrampoline tramp(std::move(progress));
    sf_common_opts opts = api->common_opts_default();
    tramp.attach(opts);

    AuditReport report;
    const sf_status rc = api->topology_audit(&in, &opts, &report.topo);
    tramp.rethrow();
    throwOnError(api, rc, "topology_audit");
    return report;
}

CleanupResult cleanupQuadSurfaceRoi(
    QuadSurface& surf, const cv::Rect& roi, const sf_cleanup_config* cfg,
    ProgressFn progress)
{
    const sf_api* api = FiestaRuntime::instance().require();

    util::GridMeshProvenance prov;
    util::TriMesh mesh = meshOf(surf, roi, prov);
    const sf_mesh in = viewOf(mesh);

    sf_cleanup_config config = cfg ? *cfg : api->cleanup_config_default();
    ProgressTrampoline tramp(std::move(progress));
    tramp.attach(config.common);

    CleanupResult result;
    {
        sf_common_opts audit_opts = api->common_opts_default();
        throwOnError(
            api, api->topology_audit(&in, &audit_opts, &result.before.topo),
            "topology_audit(before)");
    }

    SfMeshOwner cleaned(api);
    const sf_status rc = api->cleanup(&in, &config, cleaned.out(), &result.report);
    tramp.rethrow();
    throwOnError(api, rc, "cleanup");

    {
        sf_common_opts audit_opts = api->common_opts_default();
        throwOnError(
            api,
            api->topology_audit(
                &cleaned.get(), &audit_opts, &result.after.topo),
            "topology_audit(after)");
    }

    util::TriMesh outMesh = toTriMesh(cleaned.get());
    const auto cells = provenanceFor(cleaned.get(), outMesh, mesh, prov);

    util::WriteBackOptions wb;
    wb.channelSource = &surf;
    result.surface = util::writeBackToGrid(
        outMesh, cells, prov, wb, &result.writeback);
    if (!result.surface)
        throw FiestaError(
            SF_ERROR, "cleanup result could not be written back to the grid");
    return result;
}

DetangleResult detangleQuadSurface(
    QuadSurface& surf, const cv::Rect& roi, const sf_detangle_config* cfg,
    ProgressFn progress)
{
    const sf_api* api = FiestaRuntime::instance().require();

    util::GridMeshProvenance prov;
    util::TriMesh mesh = meshOf(surf, roi, prov);
    const sf_mesh in = viewOf(mesh);

    sf_detangle_config config = cfg ? *cfg : api->detangle_config_default();
    ProgressTrampoline tramp(std::move(progress));
    tramp.attach(config.common);

    DetangleResult result;
    SfMeshListOwner pieces(api);
    const sf_status rc = api->detangle(&in, &config, pieces.out(), &result.report);
    tramp.rethrow();
    throwOnError(api, rc, "detangle");

    util::WriteBackOptions wb;
    wb.channelSource = &surf;
    for (size_t i = 0; i < pieces.get().count; ++i) {
        const sf_mesh& pm = pieces.get().items[i];
        util::TriMesh pieceMesh = toTriMesh(pm);
        const auto cells = provenanceFor(pm, pieceMesh, mesh, prov);

        SplitPiece piece;
        piece.surface = util::writeBackToGrid(
            pieceMesh, cells, prov, wb, &piece.writeback);
        if (piece.surface)
            result.pieces.push_back(std::move(piece));
    }
    if (result.pieces.empty())
        throw FiestaError(
            SF_ERROR, "no detangle piece could be written back to the grid");
    return result;
}

}  // namespace vc::fiesta
