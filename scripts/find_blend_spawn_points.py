#!/usr/bin/env python3

"""Rank flat, obstacle-free launch points in the surveyed Blender tunnel."""

import math

import bpy
from mathutils import Vector


def cast(scene, depsgraph, origin, direction, distance):
    hit, location, normal, _face, obj, _matrix = scene.ray_cast(
        depsgraph, origin, direction, distance=distance)
    if not hit:
        return None
    return location, normal, obj


def main():
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    candidates = []
    # Search the main passage around the existing scene camera.  All rays
    # begin inside the tunnel, avoiding the exterior side of the roof mesh.
    for ix in range(25):
        x = -13.0 + 0.25 * ix
        for iy in range(81):
            y = 7.5 + 0.25 * iy
            floor_hit = cast(
                scene, depsgraph, Vector((x, y, 2.2)),
                Vector((0.0, 0.0, -1.0)), 8.0)
            if floor_hit is None:
                continue
            floor, normal, floor_obj = floor_hit
            if "地面" not in floor_obj.name and "ground" not in floor_obj.name.casefold():
                continue
            normal_z = abs(normal.normalized().z)
            if normal_z < math.cos(math.radians(2.0)):
                continue
            hover = Vector((x, y, floor.z + 0.8))
            ceiling_hit = cast(
                scene, depsgraph, hover, Vector((0.0, 0.0, 1.0)), 10.0)
            if ceiling_hit is None:
                continue
            ceiling_clearance = (ceiling_hit[0] - hover).length
            horizontal = []
            for direction in (
                    Vector((1.0, 0.0, 0.0)), Vector((-1.0, 0.0, 0.0)),
                    Vector((0.0, 1.0, 0.0)), Vector((0.0, -1.0, 0.0))):
                wall_hit = cast(scene, depsgraph, hover, direction, 20.0)
                horizontal.append(
                    20.0 if wall_hit is None else (wall_hit[0] - hover).length)
            minimum_horizontal = min(horizontal)
            if ceiling_clearance < 1.2 or minimum_horizontal < 1.2:
                continue
            slope_deg = math.degrees(math.acos(min(1.0, normal_z)))
            candidates.append((
                slope_deg, -minimum_horizontal, -ceiling_clearance,
                x, y, floor.z, normal, horizontal, ceiling_clearance,
                floor_obj.name))

    for item in sorted(candidates)[:30]:
        (slope_deg, neg_horizontal, _neg_ceiling, x, y, floor_z, normal,
         horizontal, ceiling, floor_name) = item
        print(
            "SPAWN_CANDIDATE "
            f"source=({x:.3f},{y:.3f},{floor_z:.4f}) "
            f"slope_deg={slope_deg:.3f} normal={tuple(round(v, 5) for v in normal)} "
            f"min_horizontal={-neg_horizontal:.3f} "
            f"horizontal={tuple(round(v, 3) for v in horizontal)} "
            f"ceiling_above_hover={ceiling:.3f} floor={floor_name}")


if __name__ == "__main__":
    main()
