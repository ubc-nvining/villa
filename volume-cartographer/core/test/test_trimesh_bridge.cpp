// TriMeshBridge round-trip tests: QuadSurface -> TriMesh (with provenance)
// -> writeBackToGrid must be bit-exact on every cell that participates in at
// least one fully-valid 2x2 quad; splits, moved vertices, synthesized
// vertices, and the stretch gate each get a dedicated case.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "vc/core/util/Geometry.hpp"
#include "vc/core/util/QuadSurface.hpp"
#include "vc/core/util/TriMeshBridge.hpp"

#include <cstring>
#include <set>

using namespace vc::core::util;

namespace
{

// A gently curved synthetic grid with a few invalid cells.
cv::Mat_<cv::Vec3f> makeGrid(int rows, int cols)
{
    cv::Mat_<cv::Vec3f> P(rows, cols);
    for (int j = 0; j < rows; ++j) {
        for (int i = 0; i < cols; ++i) {
            P(j, i) = cv::Vec3f(
                100.f + i * 20.f + j * 0.25f, 200.f + j * 20.f,
                50.f + 0.5f * i + 0.25f * j + 0.01f * i * j);
        }
    }
    return P;
}

// Cells that participate in >=1 fully-valid 2x2 quad — the set the round
// trip guarantees to reproduce (isolated cells and 1-wide filaments never
// become mesh vertices).
std::set<std::pair<int, int>> quadSupportedCells(const cv::Mat_<cv::Vec3f>& P)
{
    std::set<std::pair<int, int>> cells;
    for (int j = 0; j < P.rows - 1; ++j) {
        for (int i = 0; i < P.cols - 1; ++i) {
            if (!loc_valid(P, cv::Vec2d(j, i)))
                continue;
            cells.insert({j, i});
            cells.insert({j, i + 1});
            cells.insert({j + 1, i});
            cells.insert({j + 1, i + 1});
        }
    }
    return cells;
}

bool bitEqual(const cv::Vec3f& a, const cv::Vec3f& b)
{
    return std::memcmp(a.val, b.val, sizeof(a.val)) == 0;
}

}  // namespace

TEST_CASE("identity round trip is bit-exact on quad-supported cells")
{
    cv::Mat_<cv::Vec3f> P = makeGrid(12, 10);
    P(4, 4) = cv::Vec3f(-1, -1, -1);  // interior hole
    P(0, 9) = cv::Vec3f(-1, -1, -1);  // corner nick
    QuadSurface surf(P, cv::Vec2f(0.05f, 0.05f));

    GridMeshProvenance prov;
    TriMesh mesh = meshFromQuadSurface(surf, &prov);
    REQUIRE(!mesh.vertices.empty());
    REQUIRE(prov.cell.size() == mesh.vertices.size());
    CHECK(prov.scale == cv::Vec2f(0.05f, 0.05f));

    // UV convention: u = col/scale.x, v = row/scale.y.
    REQUIRE(mesh.uvs.size() == mesh.vertices.size());
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        CHECK(mesh.uvs[i][0] == doctest::Approx(prov.cell[i][1] / 0.05f));
        CHECK(mesh.uvs[i][1] == doctest::Approx(prov.cell[i][0] / 0.05f));
    }

    WriteBackStats st;
    auto out = writeBackToGrid(mesh, prov.cell, prov, {}, &st);
    REQUIRE(out);
    CHECK(st.conflictCells == 0);
    CHECK(st.droppedTriangles == 0);
    CHECK(st.unplacedVertices == 0);
    CHECK(st.provenancedCells == (int)mesh.vertices.size());

    const cv::Mat_<cv::Vec3f> O = out->rawPoints();
    const auto supported = quadSupportedCells(P);
    CHECK(st.validCells >= (int)supported.size());
    for (const auto& [r, c] : supported) {
        const int lr = r - st.rect.y;
        const int lc = c - st.rect.x;
        REQUIRE(lr >= 0);
        REQUIRE(lc >= 0);
        REQUIRE(lr < O.rows);
        REQUIRE(lc < O.cols);
        CHECK(bitEqual(O(lr, lc), P(r, c)));
    }
}

TEST_CASE("move-only op writes moved positions to the same cells")
{
    cv::Mat_<cv::Vec3f> P = makeGrid(6, 6);
    QuadSurface surf(P, cv::Vec2f(0.05f, 0.05f));

    GridMeshProvenance prov;
    TriMesh mesh = meshFromQuadSurface(surf, &prov);
    for (auto& v : mesh.vertices)
        v[2] += 1.5f;  // fairing-style displacement

    WriteBackStats st;
    auto out = writeBackToGrid(mesh, prov.cell, prov, {}, &st);
    REQUIRE(out);
    const cv::Mat_<cv::Vec3f> O = out->rawPoints();
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        const int lr = prov.cell[i][0] - st.rect.y;
        const int lc = prov.cell[i][1] - st.rect.x;
        CHECK(bitEqual(O(lr, lc), mesh.vertices[i]));
    }
}

TEST_CASE("split pieces write back to their own rects via position recovery")
{
    cv::Mat_<cv::Vec3f> P = makeGrid(8, 12);
    QuadSurface surf(P, cv::Vec2f(0.05f, 0.05f));

    GridMeshProvenance prov;
    TriMesh mesh = meshFromQuadSurface(surf, &prov);

    // Simulate a split op: partition faces by source column, compact each
    // piece's vertices (positions copied bit-verbatim — the ScrollFiesta
    // contract), and drop the provenance (the op "lost" it).
    auto buildPiece = [&](bool left) {
        TriMesh piece;
        std::vector<int> remap(mesh.vertices.size(), -1);
        for (const auto& f : mesh.faces) {
            bool take = true;
            for (int k = 0; k < 3; ++k) {
                const bool is_left = prov.cell[(size_t)f[k]][1] < 6;
                if (is_left != left)
                    take = false;
            }
            if (!take)
                continue;
            cv::Vec3i nf;
            for (int k = 0; k < 3; ++k) {
                int& m = remap[(size_t)f[k]];
                if (m == -1) {
                    m = (int)piece.vertices.size();
                    piece.vertices.push_back(mesh.vertices[(size_t)f[k]]);
                }
                nf[k] = m;
            }
            piece.faces.push_back(nf);
        }
        return piece;
    };

    for (bool left : {true, false}) {
        TriMesh piece = buildPiece(left);
        REQUIRE(!piece.faces.empty());
        auto cells = recoverProvenanceByPosition(piece, mesh, prov);
        for (const auto& c : cells)
            CHECK(c != kNoCell);  // verbatim copies must all match

        WriteBackStats st;
        auto out = writeBackToGrid(piece, cells, prov, {}, &st);
        REQUIRE(out);
        CHECK(st.conflictCells == 0);
        // Every piece vertex landed exactly.
        const cv::Mat_<cv::Vec3f> O = out->rawPoints();
        for (size_t i = 0; i < piece.vertices.size(); ++i) {
            const int lr = cells[i][0] - st.rect.y;
            const int lc = cells[i][1] - st.rect.x;
            CHECK(bitEqual(O(lr, lc), piece.vertices[i]));
        }
    }
}

TEST_CASE("synthesized vertices are placed by neighbor relaxation")
{
    cv::Mat_<cv::Vec3f> P = makeGrid(6, 6);
    QuadSurface surf(P, cv::Vec2f(0.05f, 0.05f));

    GridMeshProvenance prov;
    TriMesh mesh = meshFromQuadSurface(surf, &prov);

    // Hole-fill simulation: fan face 0 around a new centroid vertex.
    const cv::Vec3i f0 = mesh.faces[0];
    const cv::Vec3f centroid = (mesh.vertices[(size_t)f0[0]] +
                                mesh.vertices[(size_t)f0[1]] +
                                mesh.vertices[(size_t)f0[2]]) /
                               3.f;
    const int vc_new = (int)mesh.vertices.size();
    mesh.vertices.push_back(centroid);
    std::vector<cv::Vec2i> cells = prov.cell;
    cells.push_back(kNoCell);

    mesh.faces[0] = cv::Vec3i(vc_new, f0[0], f0[1]);
    mesh.faces.push_back(cv::Vec3i(vc_new, f0[1], f0[2]));
    mesh.faces.push_back(cv::Vec3i(vc_new, f0[2], f0[0]));

    WriteBackStats st;
    auto out = writeBackToGrid(mesh, cells, prov, {}, &st);
    REQUIRE(out);
    CHECK(st.unplacedVertices == 0);
    CHECK(st.droppedTriangles == 0);
    CHECK(st.provenancedCells == (int)mesh.vertices.size() - 1);
}

TEST_CASE("stretch gate drops triangles whose 3D size disagrees with grid size")
{
    // One triangle claiming adjacent cells but spanning 1000 voxels.
    TriMesh piece;
    piece.vertices = {
        {0.f, 0.f, 0.f}, {1000.f, 0.f, 0.f}, {0.f, 1000.f, 0.f}};
    piece.faces = {{0, 1, 2}};
    std::vector<cv::Vec2i> cells = {{5, 5}, {5, 6}, {6, 5}};

    GridMeshProvenance prov;
    prov.roi = cv::Rect(0, 0, 32, 32);
    prov.scale = cv::Vec2f(0.05f, 0.05f);  // nominal step 20 voxels

    WriteBackStats st;
    auto out = writeBackToGrid(piece, cells, prov, {}, &st);
    REQUIRE(out);
    CHECK(st.droppedTriangles == 1);
    CHECK(st.rasterizedCells == 0);
    CHECK(st.provenancedCells == 3);  // exact writes still land
}

TEST_CASE("composeProvenance maps through an op's vertex map")
{
    std::vector<cv::Vec2i> cellIn = {{0, 0}, {0, 1}, {1, 0}};
    std::vector<int32_t> vmap = {2, -1, 0};
    auto out = composeProvenance(cellIn, vmap);
    REQUIRE(out.size() == 3);
    CHECK(out[0] == cv::Vec2i(1, 0));
    CHECK(out[1] == kNoCell);
    CHECK(out[2] == cv::Vec2i(0, 0));
}

TEST_CASE("position recovery consumes duplicates in ascending order")
{
    TriMesh src;
    src.vertices = {{1.f, 2.f, 3.f}, {1.f, 2.f, 3.f}, {4.f, 5.f, 6.f}};
    src.faces = {{0, 1, 2}};
    GridMeshProvenance prov;
    prov.cell = {{0, 0}, {0, 1}, {1, 1}};
    prov.scale = cv::Vec2f(0.05f, 0.05f);

    TriMesh piece;
    piece.vertices = {
        {1.f, 2.f, 3.f}, {1.f, 2.f, 3.f}, {1.f, 2.f, 3.f}, {9.f, 9.f, 9.f}};
    piece.faces = {{0, 1, 2}};

    auto cells = recoverProvenanceByPosition(piece, src, prov);
    CHECK(cells[0] == cv::Vec2i(0, 0));   // first duplicate
    CHECK(cells[1] == cv::Vec2i(0, 1));   // second duplicate
    CHECK(cells[2] == cv::Vec2i(0, 0));   // exhausted -> re-use first
    CHECK(cells[3] == kNoCell);           // no match
}
