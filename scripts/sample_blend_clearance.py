#!/usr/bin/env python3

"""Report first-hit clearances around camera locations in a Blender scene."""

import bpy
from mathutils import Vector


def main():
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    directions = {
        "east_x+": Vector((1.0, 0.0, 0.0)),
        "west_x-": Vector((-1.0, 0.0, 0.0)),
        "north_y+": Vector((0.0, 1.0, 0.0)),
        "south_y-": Vector((0.0, -1.0, 0.0)),
        "ceiling_z+": Vector((0.0, 0.0, 1.0)),
        "floor_z-": Vector((0.0, 0.0, -1.0)),
    }
    for camera in sorted(
            (obj for obj in scene.objects if obj.type == "CAMERA"),
            key=lambda obj: obj.name.casefold()):
        origin = camera.matrix_world.translation
        print(f"CLEARANCE_CAMERA {camera.name} origin={tuple(round(v, 4) for v in origin)}")
        for label, direction in directions.items():
            hit, location, _normal, _face, obj, _matrix = scene.ray_cast(
                depsgraph, origin, direction, distance=100.0)
            if hit:
                distance = (location - origin).length
                print(
                    f"  {label}: {distance:.4f} m hit={obj.name} "
                    f"at={tuple(round(v, 4) for v in location)}")
            else:
                print(f"  {label}: no hit within 100 m")


if __name__ == "__main__":
    main()
