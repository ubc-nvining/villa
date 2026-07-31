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
the segmentation viewer, then **Selection → ScrollFiesta: Generate Spiral
Hints**. Supply the scroll axis and expected wrap spacing in the dialog. Each
selected grid region becomes a new `scroll-hint` / `unverified` tifxyz patch;
the source segment is never modified. When a Spiral session is active, the
dialog can upload the generated patches directly as low-trust inputs.

Everything runs on a worker thread with a determinate progress dialog and a
working **Cancel** (the original is never touched by a cancelled run).
Segments written by ScrollFiesta carry `fiesta-clean`, `fiesta-split`, or
`scroll-hint` tags and a `meta["fiesta"]` provenance block. Spiral hints also
record their ROI, source volume, scroll geometry, and explicit `unverified`
trust state.

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
vc_fiesta hints <tifxyz_dir> <out_root> --roi=X,Y,W,H [--roi=...]
                --json=<manifest.json> --axis-point-zyx=Z,Y,X
                --axis-direction-zyx=Z,Y,X --wrap-spacing=N
                [--volume-id=ID] [--volume-location=PATH]
                [--voxel-size=N] [--volume-tag=TAG] [--progress]
                [--cancel-file=FILE]
```
Generates one new unverified hint per ROI and writes a
`vc3d-scrollfiesta-hints-v1` import manifest. Output directories use
`QuadSurface`'s atomic save, existing paths are never overwritten, and any
outputs already created by a failed or cancelled multi-ROI run are rolled
back. Creating `--cancel-file` requests cooperative cancellation (exit 3).

### Feeding a file-based hint dataset to Spiral

`scripts/spiral/scrollfiesta_dataset.py` validates an exchange directory and
passes its verified and unverified patch roles to `fit_spiral.py` without a
Python/C++ ABI dependency. The directory must contain
`villa_dataset.json` with format `scrollfiesta-villa-dataset-v1`, paths named
by `verified_patches` and `unverified_patches`, and any point-collection files
listed by `point_collections`. A tifxyz patch may use either `-1` coordinate
sentinels or an explicit `mask.tif`; the mask is optional.

```
python scripts/spiral/scrollfiesta_dataset.py <exchange_dir>
python scripts/spiral/scrollfiesta_dataset.py <exchange_dir> \
  --base-dataset <villa_dataset> --load-only
python scripts/spiral/scrollfiesta_dataset.py <exchange_dir> \
  --connect-overlaps
```

The first command is validation-only. `--load-only` exercises patch/PCL
loading and linkage without starting optimization; omit it and use `--fit` to
run the fitter. Add `--dry-run` to inspect the exact command and
`FIT_SPIRAL_*` environment without launching it.

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
voxels marks essentially every curved region as an inter-wrap seam at that
scale.** Measured on two clean, already-flattened PHerc0139 published
segments (both audit as genus-0 single-boundary disks), `vc_fiesta detangle`
did not split them into comparable pieces — it *deleted almost the whole
sheet*:

| segment | input verts | pieces kept | verts kept | retained |
|---|---|---|---|---|
| 20260302000001 | 691,264 | 3 | 231 + 228 + 382 = 841 | **0.12 %** |
| 20260422000000 | 323,216 | 2 | 338 + 276 = 614       | **0.19 %** |

Depth peel classifies the curved sheet as seam, removes the seam band, and
returns only the few surviving islands large enough to clear
`min_comp_verts` (200). Developability-cut, bridge-cut and overlap-separation
correctly report zero splits *when peel runs first* — but with peel disabled,
overlap separation finds ~3M spurious overlap pairs on the full coarse mesh,
so the whole cascade is scale-sensitive, not just peel. Write-back is correct
throughout (zero conflicts/unplaced/dropped); the destruction is entirely in
the split *decision*, inside the library, before write-back sees anything.

**Consequence:** `Topology Audit` is safe and useful today. `Detangle` at the
default grid resolution is **destructive** — it discards ~99.9 % of a clean
segment. `Clean` is a near-no-op on traced segments (they are manifold by
construction) — its value is component culling and CDT hole fill.

### Guards in place

- **Mass-retention guard** (`vc::fiesta::detangleQuadSurface`): if a detangle
  keeps less than half the input mesh, the call throws `FiestaError` instead of
  writing near-empty crumbs. The GUI shows the message in a dialog; the
  `vc_fiesta detangle` CLI prints it and exits non-zero. `cleanupQuadSurfaceRoi`
  has the same defensive check (fires only when no component cull was
  requested). The research `vc_fiesta survey` disables the guard
  (`minRetainedFraction = 0`) so it can *measure* the loss and report it
  ("kept 0.1%").
- **Detangle GUI hidden by default**: the per-segment "Detangle / Split" and
  Selection "Detangle Selection" actions are gated behind the CMake option
  `VC_FIESTA_DETANGLE_UI` (**OFF** by default). Audit and Clean are always
  available. `vc_fiesta detangle` / `survey` remain for research regardless.

The correct capability for "fix a tangled region" is to **re-mesh the region
from the surface-prediction volume** (`sf_pipeline_run` — ScrollFiesta's native
per-cube regime), then convert back through the flatten route. The catalog
exposes streamable `surface-prediction-zarr` volumes for exactly this; it is
planned, not yet implemented.

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
fetch the pinned public revision, build `scrollfiesta.dll`/`.so`, and stage it next to
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
