#!/usr/bin/env python3
"""Run with Blender Python: regression checks for the 3-D mesh reference."""

from pathlib import Path
import sys

from mathutils import Vector
from mathutils.bvhtree import BVHTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_baixiangshan_reference_3d import (
    between_floor_and_roof, edge_safe, surface_clearance)


def plane_at_x_zero():
    vertices = [Vector((0, -10, -10)), Vector((0, 10, -10)),
                Vector((0, 10, 10)), Vector((0, -10, 10))]
    return BVHTree.FromPolygons(vertices, [(0, 1, 2), (0, 2, 3)],
                                all_triangles=True)


def main():
    bvhs = {"wall": plane_at_x_zero()}
    # A nearest query with too small a radius provides only a lower bound; it
    # must never return infinity, which would falsely certify an edge.
    assert surface_clearance(bvhs, Vector((3, 0, 0)), 1.5) == 1.5
    assert not edge_safe(bvhs, Vector((-2, 0, 0)), Vector((2, 0, 0)),
                         2.0, 2.0, 1.0)
    assert edge_safe(bvhs, Vector((2, 0, 0)), Vector((3, 0, 0)),
                     2.0, 3.0, 1.0)
    assert between_floor_and_roof(2.0, [0.0], [4.0], 1.0)
    assert not between_floor_and_roof(2.0, [3.0, 0.0], [4.0], 1.0)
    print("3-D reference geometry regression checks passed")


if __name__ == "__main__":
    main()
