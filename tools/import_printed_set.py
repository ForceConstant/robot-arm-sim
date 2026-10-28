"""Import six user-supplied, z-up STL pieces without redistributing their files.

Run with an offline tooling environment containing trimesh. Runtime
needs only the generated pack, not the original STLs. Outputs remain git-ignored.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "2")
import numpy as np
import trimesh
from PIL import Image

PIECES = ("pawn", "rook", "knight", "bishop", "queen", "king")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--scale", type=float, default=0.5, help="uniform source scale, not height fitting")
    ap.add_argument("--density", type=float, default=600, help="estimated effective print density kg/m3")
    args = ap.parse_args()
    if not 0 < args.scale <= 2 or args.density <= 0:
        ap.error("scale and density must be positive (scale <= 2)")
    files = {p.stem.lower(): p for p in args.source.glob("*.stl")}
    if not all(p in files for p in PIECES):
        ap.error("source must contain Pawn/Rook/Knight/Bishop/Queen/King.stl")
    if args.output.exists():
        ap.error("output already exists; use a fresh directory to preserve prior assets")
    slab_height = 0.00075  # 0.75 mm vertical convex approximation
    manifest = dict(name=args.name, scale="absolute", uniform_source_scale=args.scale,
                    source_units="mm", density_kg_m3=args.density,
                    collision_method="0.75 mm horizontal convex slabs per disconnected solid; underside weight pockets filled",
                    collision_slab_height_m=slab_height, pieces={})
    for name in PIECES:
        source = trimesh.load(files[name], force="mesh")
        if not source.is_watertight or source.volume <= 0:
            raise ValueError(f"{name}: requires a closed positive-volume source mesh")
        source_bounds = source.bounds.copy()
        # Centre on the circular foot, not the asymmetric knight head.
        base = source.vertices[source.vertices[:, 2] < source.bounds[0, 2] + 10]
        centre = (base[:, :2].min(axis=0) + base[:, :2].max(axis=0)) / 2
        source.apply_translation([-centre[0], -centre[1], -source.bounds[0, 2]])
        source.apply_scale(args.scale * 0.001)
        folder = args.output / name
        folder.mkdir(parents=True)
        source.export(folder / f"{name}.obj")
        Image.new("RGB", (8, 8), (225, 214, 190)).save(folder / "material_0.png")
        collision = folder / "collision"
        collision.mkdir()
        parts = []
        for component in source.split():
            lo, hi = component.bounds[:, 2]
            edges = np.linspace(lo, hi, max(2, int(np.ceil((hi - lo) / slab_height)) + 1))
            for bottom, top in zip(edges[:-1], edges[1:]):
                # Clip triangles at both planes before taking a local hull.
                # A whole-piece hull would erase the neck. Horizontal cavities
                # inside a slab (notably the underside weight pocket) are filled.
                band = component.slice_plane([0, 0, bottom], [0, 0, 1])
                band = band.slice_plane([0, 0, top], [0, 0, -1])
                if len(band.vertices) < 4 or band.extents[2] < 1e-8:
                    continue
                hull = band.convex_hull
                rel = Path(name) / "collision" / f"{len(parts):03d}.obj"
                hull.export(args.output / rel)
                parts.append(str(rel))
        # Conservative, finely sampled envelope for controller clearance ONLY.
        # Real contact uses the convex parts above, never these cylinders.
        height = float(source.extents[2])
        profile = []
        for z0, z1 in zip(np.linspace(0, height, 49)[:-1], np.linspace(0, height, 49)[1:]):
            points = []
            band = source.vertices[(source.vertices[:, 2] >= z0) & (source.vertices[:, 2] <= z1)]
            if len(band):
                points.append(band[:, :2])
            for z in (z0 + 1e-8, (z0 + z1) / 2, z1 - 1e-8):
                section = source.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
                if section is not None:
                    points.append(section.vertices[:, :2])
            if not points:
                raise ValueError(f"{name}: empty silhouette slab")
            radius = float(np.linalg.norm(np.concatenate(points), axis=1).max()) + 0.0001
            profile.append([float(z0), float(z1), radius])
        source.density = args.density
        manifest["pieces"][name] = dict(source_sha256=hashlib.sha256(files[name].read_bytes()).hexdigest(),
            source_bounds_mm=source_bounds.tolist(), extent_m=source.extents.tolist(),
            collision_meshes=parts, profile=profile,
            mass_kg=float(source.mass), center_mass_m=source.center_mass.tolist(),
            inertia_kg_m2=source.moment_inertia.tolist())
        print(f"{name}: {len(parts)} convex parts, {source.mass*1000:.2f} g estimated", flush=True)
    (args.output / "set.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(args.output / "set.json", flush=True)


if __name__ == "__main__":
    main()
