# ScrollFiesta integration

[ScrollFiesta](https://github.com/Hob3rMallow/scrollfiesta_public) is an
external, independently-developed mesh toolkit for scroll segmentations:
sheet **detangling/splitting** (depth peel, developability cut, max-flow
bridge cut, lifted-multicut overlap separation), **cleanup** (manifold
repair, pinhole/hole filling, sliver cleanup, component culling), plus QEM
decimation, developability fairing, Seamster cut-to-disk, BPA reconstruction,
and a full binary-volume→mesh pipeline.

VC3D integrates it without merging it: ScrollFiesta stays its own repo and
VC3D **loads it at runtime** — `dlopen`/`LoadLibrary` of
`scrollfiesta.dll` / `libscrollfiesta.so`, one resolved symbol
(`sf_get_api`), and a function-pointer table. Nothing links the library, so:

- swapping in a newer ScrollFiesta is *replace the DLL next to the
  executable, restart* — `sf_get_api(SCROLLFIESTA_ABI_VERSION)` returning
  NULL is the whole compatibility gate;
- the library and VC3D can be built with different compilers/CRTs;
- when the library is absent, the features are simply unavailable (menu
  entries pop a dialog with the loader's reason), never fatal.

Library search order: the executable's directory → the `SCROLLFIESTA_DLL`
environment variable (full path) → the platform's default library search.

## GUI workflows

**Per-segment** — right-click a segment in the surface panel →
**ScrollFiesta** submenu:

- **Topology Audit** (read-only): components, boundary loops, non-manifold
  edges, genus, is-it-a-disk. Results in a message box (details expandable).
- **Clean**: manifold repair, pinhole fill, optional sliver cleanup and
  component culling. The dialog offers two result modes:
  - *Save as a new segment* (default) — the original is untouched;
  - *Overwrite the original* — a disk backup is written first
    (`backups/<id>/…`); undo via right-click → *Reload from backup*.
- **Detangle / Split**: the split cascade (depth peel → gated developability
  cut → bridge cut → overlap separation). Pieces are always saved as new
  segments; a clean single sheet self-gates to one piece.

**Subregions** — enable *Edit → Draw BBox*, drag one or more selections on
the segmentation viewer, then **Selection → ScrollFiesta: Clean Selection /
Detangle Selection**. The operation runs on exactly those grid regions; all
selections are processed in one job with per-selection progress.

Everything runs on a worker thread with a determinate progress dialog and a
working **Cancel** (the original is never touched by a cancelled run).
Segments written by ScrollFiesta carry `fiesta-clean` / `fiesta-split` tags
and a `meta["fiesta"]` provenance block (`op`, `parent`, `roi`, `piece`).

## CLI tools

```
vc_fiesta <audit|clean|detangle> <tifxyz_dir> [out] [--roi=X,Y,W,H] [--json=FILE] [--progress]
```
The batch counterpart of the GUI actions (same code path). `--progress`
emits `PROGRESS <pct> <stage>` lines. Exit codes: 0 ok, 1 usage, 2 failed,
3 cancelled.

```
vc_fiesta survey <tree> [out] [--detangle] [--csv=FILE] [--max-cells=N]
                 [--no-peel] [--no-dev] [--no-bridge] [--no-overlap]
```
Batch audit (and optional detangle) over every materialized tifxyz segment
under `<tree>`, writing a per-segment CSV (verts/faces/components/loops/
non-manifold/genus/timings; with `--detangle`, piece counts per stage and
write-back warnings). This is the harness for evaluating the ops on real
data; the stage toggles let you A/B a single splitter. Segments above
`--max-cells` (default 6M) are skipped.

```
vc_opendata list                    [--json] [--manifest=<url|file>]
vc_opendata segments <sampleId>     [--json] [--cache=<dir>]
vc_opendata pull     <sampleId> [segmentId...] [--smallest=N] [--cache=<dir>] [--force]
```
Lists the streamable open-data volpkgs from the hosted manifest and pulls
their tifxyz segments to a local cache using VC3D's own segment-cache engine
(so the pulled dir has a correctly synthesized `meta.json` and is usable by
every `vc_*` tool). `--smallest=N` grabs the N smallest segments — handy for
quick experiments.

## Calibration caveat for the region/detangle ops (important)

The split thresholds are tuned for ScrollFiesta's native ~0.77-voxel meshes.
A VC3D segment grid step is **~20 voxels**, so a segment converted to
triangles has ~20-voxel edges — and **depth peel's default `min_gap` of 1.5
voxels fires on ordinary sheet curvature at that scale.** Measured on two
clean, already-flattened PHerc0139 published segments (both audit as genus-0
single-boundary disks): depth peel split them into 3 and 2 pieces, while
developability-cut, bridge-cut, and overlap-separation all correctly reported
zero splits *when peel runs first* — but with peel disabled, overlap
separation found ~3M spurious overlap pairs on the full coarse mesh. The
write-back mechanics are correct (zero conflicts/unplaced/dropped); it is the
split *decision* that is mis-scaled.

**Consequence:** `Topology Audit` is safe and useful today. `Detangle` at the
default grid resolution will over-split clean geometry and should be treated
as experimental until the per-stage thresholds are exposed and rescaled to
the grid (or the region is resampled to near-voxel density before meshing).
`Clean` is a near-no-op on traced segments (they are manifold by
construction) — its value is component culling and CDT hole fill.

```
vc_mesh2tifxyz <in.obj> <out_tifxyz> [--source=<tifxyz>] [--scale=SX,SY] [--flatten] [--lscm-only] [--zyx]
```
Turns an arbitrary triangle mesh into a tifxyz segment, choosing the route
automatically (see below). `--zyx` accepts ScrollFiesta's *tool* OBJs, which
are written `v z y x` (the API and everything else here is true x,y,z).

## How meshes come back to the quad grid

Two routes, both in `vc_core`/`vc_flattening` (no ScrollFiesta dependency):

1. **Provenance write-back** (`vc/core/util/TriMeshBridge.hpp`) — the
   triangles VC3D hands out are generated *from* the grid, so every vertex
   carries its source `(row, col)`. After an operation, surviving vertices
   write their (possibly moved) positions back into their cells —
   bit-exactly when unmoved — newly created vertices are placed by neighbor
   relaxation, covered holes are filled by exact rasterization, and cells no
   piece claims become invalid: **absence is deletion**, which is how culled
   components and cut seams appear on a grid. Split pieces become separate
   cropped segments in the source row/col frame (`WriteBackStats::rect` says
   where).
2. **Flatten + rasterize** (`vc/flattening/MeshFlatten.hpp`) — for meshes
   with no grid ancestry (BPA/pipeline reconstruction, external welds):
   ABF++/LSCM to a single metric chart (UV units = voxels), then exact
   rasterization at the requested `scale`. Input must be a manifold disk —
   ScrollFiesta's cleanup / cut-to-disk are the repair tools; UV-flipped
   triangle counts are reported as the distortion red flag.

## Building

`VC_WITH_SCROLLFIESTA` (default ON) makes `cmake/FetchScrollFiesta.cmake`
fetch the pinned tag, build `scrollfiesta.dll`/`.so`, and stage it next to
the executables in `build/bin`; `cmake --install … --component vc_runtime`
ships it alongside VC3D. With the option OFF there is no network access, no
ScrollFiesta target, and the GUI entries are compiled out
(`VC_HAVE_SCROLLFIESTA` guards the call sites).

Developing against a local ScrollFiesta checkout (skips the fetch):

```
cmake -DFETCHCONTENT_SOURCE_DIR_SCROLLFIESTA=D:/work/scrollfiesta_public …
```

ScrollFiesta vendors all of its dependencies, so no packaging manifest
(vcpkg.json, install_build_deps.sh, Homebrew) changes are needed. Note the
vendored Shewchuk **Triangle** carries a no-commercial-distribution license —
see the ScrollFiesta repo's `THIRD_PARTY_LICENSES.md`.

## Troubleshooting

- *Menu says the library could not be loaded*: the dialog shows exactly
  which paths were tried. Build the `scrollfiesta` target (any
  `VC_WITH_SCROLLFIESTA=ON` build does this) or point `SCROLLFIESTA_DLL` at
  a library, and make sure its ABI matches this build's
  `SCROLLFIESTA_ABI_VERSION`.
- *"selected region contains no valid 2x2 quads"*: the selection covers only
  invalid cells — pick a region with surface in it.
- *Detangle returns one piece*: that is the self-gate — the splitters found
  no inter-wrap seam to cut. It is a result, not a failure.
- *Write-back warnings (conflicts / unplaced / dropped triangles)*: the
  operation produced geometry that does not fit the source grid cleanly;
  inspect before trusting, or re-run on a larger region.
