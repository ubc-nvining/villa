"""Validate and feed a ScrollFiesta patch dataset into Villa's spiral fitter.

This is deliberately a process/file boundary: ScrollFiesta writes ordinary
tifxyz patches plus VC point-collection JSON; Villa validates them with its own
loader, can synthesize overlap constraints, then launches fit_spiral.py with
environment overrides. No C/Python ABI or repository-relative data path is
required.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image

from tifxyz import load_tifxyz


PCL_FILES = ("abs_winding.json", "patch-overlap-pcls.json",
             "relative_windings.json", "same_windings.json")


def _read_config(root: Path) -> dict:
    path = root / "villa_dataset.json"
    if not path.is_file():
        raise RuntimeError(f"missing {path}")
    cfg = json.loads(path.read_text())
    if cfg.get("format") != "scrollfiesta-villa-dataset-v1":
        raise RuntimeError(f"unsupported dataset format {cfg.get('format')!r}")
    return cfg


def _validate_patch(path: Path) -> dict:
    # mask.tif is optional in the native tifxyz contract. QuadSurface::save()
    # writes x/y/z with -1 sentinels, while some imported datasets retain real
    # coordinates under an explicit zero mask. The Spiral loader supports both
    # forms, so the adapter must not reject VC3D/ScrollFiesta's own outputs.
    required = ("x.tif", "y.tif", "z.tif", "meta.json")
    missing = [name for name in required if not (path / name).is_file()]
    if missing:
        raise RuntimeError(f"{path}: missing {missing}")
    patch = load_tifxyz(str(path))
    xyz = []
    for axis in "xyz":
        with Image.open(path / f"{axis}.tif") as image:
            xyz.append(np.asarray(image, dtype=np.float32).copy())
    mask_path = path / "mask.tif"
    if mask_path.is_file():
        with Image.open(mask_path) as image:
            mask = np.asarray(image).copy()
        if mask.ndim == 3:
            mask = mask[..., 0]
        if any(array.shape != mask.shape for array in xyz):
            raise RuntimeError(f"{path}: mask and xyz dimensions differ")
    valid = patch.valid_vertex_mask.detach().cpu().numpy()
    return {"id": path.name, "valid_cells": int(valid.sum()),
            "shape": list(valid.shape)}


def validate_dataset(root: Path) -> dict:
    root = root.resolve()
    cfg = _read_config(root)
    results = {"verified": [], "unverified": [], "point_collections": {}}
    for kind, key in (("verified", "verified_patches"),
                      ("unverified", "unverified_patches")):
        patch_root = root / cfg[key]
        if not patch_root.is_dir():
            raise RuntimeError(f"missing patch directory {patch_root}")
        for path in sorted(p for p in patch_root.iterdir() if p.is_dir()):
            results[kind].append(_validate_patch(path))
    if not results["verified"]:
        raise RuntimeError("dataset contains no verified patches")
    for name in cfg.get("point_collections", PCL_FILES):
        path = root / name
        data = json.loads(path.read_text())
        if data.get("vc_pointcollections_json_version") != "1":
            raise RuntimeError(f"{path}: not VC point-collection JSON v1")
        results["point_collections"][name] = len(data.get("collections", {}))
    return results


def fit_environment(root: Path, base_dataset: Path | None,
                    z_begin: int | None, z_end: int | None) -> dict:
    cfg = _read_config(root)
    env = dict(os.environ)
    if base_dataset:
        env["FIT_SPIRAL_DATASET_PATH"] = str(base_dataset.resolve())
    env["FIT_SPIRAL_VERIFIED_PATCHES_PATH"] = str(
        (root / cfg["verified_patches"]).resolve())
    env["FIT_SPIRAL_UNVERIFIED_PATCHES_PATH"] = str(
        (root / cfg["unverified_patches"]).resolve())
    env["FIT_SPIRAL_PCL_JSON_PATHS"] = json.dumps([
        str((root / name).resolve())
        for name in cfg.get("point_collections", PCL_FILES)
    ])
    if z_begin is not None:
        env["FIT_SPIRAL_Z_BEGIN"] = str(z_begin)
    if z_end is not None:
        env["FIT_SPIRAL_Z_END"] = str(z_end)
    return env


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--base-dataset", type=Path,
                    help="Villa dataset supplying normals/tracks/shell/umbilicus")
    ap.add_argument("--connect-overlaps", action="store_true")
    ap.add_argument("--fit", action="store_true")
    ap.add_argument("--load-only", action="store_true",
                    help="ask fit_spiral to load/link patches and PCLs, then stop")
    ap.add_argument("--z-begin", type=int)
    ap.add_argument("--z-end", type=int)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)
    root = args.dataset.resolve()

    report = validate_dataset(root)
    print(json.dumps(report, indent=2))

    here = Path(__file__).resolve().parent
    if args.connect_overlaps:
        cfg = _read_config(root)
        verified = (root / cfg["verified_patches"]).resolve()
        cmd = [sys.executable, str(here / "connect_overlapping_patches.py"),
               str(verified),
               "--pairs-output", str(root / "overlap_pairs.jsonl"),
               "--collection-output", str(root / "patch-overlap-pcls.json")]
        if args.dry_run:
            print("DRY RUN:", subprocess.list2cmdline(cmd))
        else:
            subprocess.run(cmd, check=True)
            report = validate_dataset(root)
            print("overlap collections:",
                  report["point_collections"].get("patch-overlap-pcls.json", 0))

    if args.fit or args.load_only:
        env = fit_environment(root, args.base_dataset, args.z_begin, args.z_end)
        if args.load_only:
            env["FIT_SPIRAL_LOAD_ONLY"] = "1"
            # Patch/PCL linkage validation should not require a shell asset.
            overrides = json.loads(env.get("FIT_SPIRAL_CONFIG_OVERRIDES", "{}"))
            overrides.update({"loss_weight_shell_outer": 0.0,
                              "loss_weight_shell_patch_radius": 0.0})
            env["FIT_SPIRAL_CONFIG_OVERRIDES"] = json.dumps(overrides)
            env["FIT_SPIRAL_SHELL_PATH"] = ""
        cmd = [sys.executable, str(here / "fit_spiral.py")]
        if args.dry_run:
            keys = sorted(k for k in env if k.startswith("FIT_SPIRAL_"))
            print("DRY RUN environment:")
            print(json.dumps({k: env[k] for k in keys}, indent=2))
            print("DRY RUN:", subprocess.list2cmdline(cmd))
        else:
            subprocess.run(cmd, env=env, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
