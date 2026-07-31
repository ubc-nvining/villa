"""Contract tests for the ScrollFiesta-to-Spiral dataset adapter."""

from __future__ import annotations

import ast
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scrollfiesta_dataset import fit_environment, validate_dataset
from config import Config


def _write_patch(path: Path, *, mask: bool = False,
                 hidden_positive: bool = False) -> None:
    path.mkdir(parents=True)
    yy, xx = np.mgrid[:4, :4].astype(np.float32)
    arrays = {
        "x": 100.0 + xx,
        "y": 200.0 + yy,
        "z": 300.0 + xx * 0.1 + yy * 0.2,
    }
    if mask and not hidden_positive:
        for array in arrays.values():
            array[0, 0] = -1.0
    for axis, array in arrays.items():
        Image.fromarray(array).save(path / f"{axis}.tif")
    if mask:
        valid = np.full((4, 4), 255, dtype=np.uint8)
        valid[0, 0] = 0
        Image.fromarray(valid).save(path / "mask.tif")
    (path / "meta.json").write_text(json.dumps({
        "format": "tifxyz",
        "type": "seg",
        "uuid": path.name,
        "scale": [0.05, 0.05],
    }), encoding="utf-8")


class ScrollFiestaDatasetTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        (self.root / "verified").mkdir()
        (self.root / "unverified").mkdir()
        (self.root / "villa_dataset.json").write_text(json.dumps({
            "format": "scrollfiesta-villa-dataset-v1",
            "verified_patches": "verified",
            "unverified_patches": "unverified",
            "point_collections": [],
        }), encoding="utf-8")

    def tearDown(self):
        self.temporary.cleanup()

    def test_accepts_native_maskless_tifxyz_and_optional_mask(self):
        _write_patch(self.root / "verified" / "base")
        _write_patch(self.root / "unverified" / "hint", mask=True)

        report = validate_dataset(self.root)

        self.assertEqual(report["verified"][0]["valid_cells"], 16)
        self.assertEqual(report["unverified"][0]["valid_cells"], 15)

    def test_accepts_positive_coordinates_hidden_by_mask(self):
        _write_patch(self.root / "verified" / "base")
        _write_patch(
            self.root / "unverified" / "masked",
            mask=True,
            hidden_positive=True,
        )

        report = validate_dataset(self.root)

        self.assertEqual(report["unverified"][0]["valid_cells"], 15)

    def test_fit_environment_points_at_both_patch_roles(self):
        _write_patch(self.root / "verified" / "base")
        previous = os.environ.get("FIT_SPIRAL_Z_BEGIN")
        try:
            os.environ.pop("FIT_SPIRAL_Z_BEGIN", None)
            env = fit_environment(
                self.root,
                Path("D:/villa-base"),
                z_begin=400,
                z_end=900,
            )
        finally:
            if previous is not None:
                os.environ["FIT_SPIRAL_Z_BEGIN"] = previous

        self.assertEqual(
            Path(env["FIT_SPIRAL_VERIFIED_PATCHES_PATH"]),
            (self.root / "verified").resolve(),
        )
        self.assertEqual(
            Path(env["FIT_SPIRAL_UNVERIFIED_PATCHES_PATH"]),
            (self.root / "unverified").resolve(),
        )
        self.assertEqual(env["FIT_SPIRAL_Z_BEGIN"], "400")
        self.assertEqual(env["FIT_SPIRAL_Z_END"], "900")

    def test_fit_spiral_uses_only_declared_config_keys(self):
        source = (Path(__file__).resolve().parents[1] /
                  "fit_spiral.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        referenced = {
            node.slice.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Subscript)
            and isinstance(node.value, ast.Name)
            and node.value.id == "cfg"
            and isinstance(node.slice, ast.Constant)
            and isinstance(node.slice.value, str)
        }
        self.assertEqual(
            sorted(referenced - set(Config().as_dict())),
            [],
            "fit_spiral references configuration keys absent from Config",
        )


if __name__ == "__main__":
    unittest.main()
