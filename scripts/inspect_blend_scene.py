#!/usr/bin/env python3

"""Write a deterministic JSON inventory of the currently opened Blender scene."""

import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector


def finite_vec(values):
    return [float(value) if math.isfinite(value) else None for value in values]


def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if len(args) != 1:
        raise SystemExit("usage: blender --background file.blend --python inspect_blend_scene.py -- OUTPUT.json")

    output_path = Path(args[0])
    scene = bpy.context.scene
    objects = []
    aggregate_min = Vector((math.inf, math.inf, math.inf))
    aggregate_max = Vector((-math.inf, -math.inf, -math.inf))

    for obj in sorted(bpy.data.objects, key=lambda item: item.name.casefold()):
        record = {
            "name": obj.name,
            "type": obj.type,
            "collection_names": sorted(collection.name for collection in obj.users_collection),
            "hide_viewport": bool(obj.hide_viewport),
            "hide_render": bool(obj.hide_render),
            "location": finite_vec(obj.location),
            "rotation_euler": finite_vec(obj.rotation_euler),
            "scale": finite_vec(obj.scale),
            "dimensions": finite_vec(obj.dimensions),
            "modifiers": [modifier.type for modifier in obj.modifiers],
        }
        if obj.type == "MESH":
            mesh = obj.data
            record.update({
                "vertices": len(mesh.vertices),
                "edges": len(mesh.edges),
                "polygons": len(mesh.polygons),
                "loops": len(mesh.loops),
                "materials": [slot.material.name if slot.material else None for slot in obj.material_slots],
            })
            if len(mesh.vertices):
                world_corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
                bbox_min = Vector((
                    min(point.x for point in world_corners),
                    min(point.y for point in world_corners),
                    min(point.z for point in world_corners),
                ))
                bbox_max = Vector((
                    max(point.x for point in world_corners),
                    max(point.y for point in world_corners),
                    max(point.z for point in world_corners),
                ))
                record["world_bbox_min"] = finite_vec(bbox_min)
                record["world_bbox_max"] = finite_vec(bbox_max)
                if not obj.hide_render:
                    aggregate_min.x = min(aggregate_min.x, bbox_min.x)
                    aggregate_min.y = min(aggregate_min.y, bbox_min.y)
                    aggregate_min.z = min(aggregate_min.z, bbox_min.z)
                    aggregate_max.x = max(aggregate_max.x, bbox_max.x)
                    aggregate_max.y = max(aggregate_max.y, bbox_max.y)
                    aggregate_max.z = max(aggregate_max.z, bbox_max.z)
        objects.append(record)

    images = []
    for item in sorted(bpy.data.images, key=lambda image: image.name.casefold()):
        images.append({
            "name": item.name,
            "source": item.source,
            "filepath": item.filepath,
            "absolute_filepath": bpy.path.abspath(item.filepath) if item.filepath else "",
            "packed": item.packed_file is not None,
            "size": list(item.size),
        })

    mesh_objects = [item for item in objects if item["type"] == "MESH"]
    report = {
        "blend_file": bpy.data.filepath,
        "blender_version": bpy.app.version_string,
        "scene": scene.name,
        "unit_settings": {
            "system": scene.unit_settings.system,
            "scale_length": scene.unit_settings.scale_length,
            "length_unit": scene.unit_settings.length_unit,
        },
        "object_count": len(objects),
        "mesh_object_count": len(mesh_objects),
        "mesh_vertices": sum(item.get("vertices", 0) for item in mesh_objects),
        "mesh_polygons": sum(item.get("polygons", 0) for item in mesh_objects),
        "visible_mesh_bbox_min": finite_vec(aggregate_min),
        "visible_mesh_bbox_max": finite_vec(aggregate_max),
        "visible_mesh_dimensions": finite_vec(aggregate_max - aggregate_min),
        "collections": sorted(collection.name for collection in bpy.data.collections),
        "materials": sorted(material.name for material in bpy.data.materials),
        "images": images,
        "objects": objects,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"BLEND_SCENE_REPORT={output_path}")


if __name__ == "__main__":
    main()
