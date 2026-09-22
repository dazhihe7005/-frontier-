#!/usr/bin/env python3

"""Export the opened Bai Xiang Shan Blender scene for Gazebo Classic.

The source scene is never saved.  Terrain meshes are deterministically
decimated, transformed into a spawn-relative ENU frame, and exported once per
category.  Gazebo then references the same OBJ for visual, ray and collision
geometry.
"""

import json
import math
import re
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector


TERRAIN_COLLECTIONS = {
    "rock": "01_岩壁_干燥岩石",
    "ground": "02_地面_砂砾",
    "roof": "03_顶板_可隐藏",
}

# Camera 01 sits on the main passage centerline.  The downward ray audit found
# the floor at z=0.7689 m.  Shift this point to the simulation origin and rotate
# Blender +Y to Gazebo +X so the existing task-one forward explorer is valid.
SOURCE_ORIGIN = Vector((-10.0, 17.5, 0.7689))
YAW_RADIANS = math.radians(-90.0)
TERRAIN_DECIMATE_RATIO = 0.08
INFRASTRUCTURE_DECIMATE_RATIO = 0.25


def safe_filename(name):
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("._")
    return stem or "texture"


def extract_packed_images(texture_dir):
    texture_dir.mkdir(parents=True, exist_ok=True)
    exported = []
    for image in sorted(bpy.data.images, key=lambda item: item.name.casefold()):
        if image.source != "FILE" or not image.packed_file:
            continue
        filename = safe_filename(Path(image.name).name)
        target = texture_dir / filename
        target.write_bytes(bytes(image.packed_file.data))
        image.filepath = f"//textures/{filename}"
        image.filepath_raw = image.filepath
        exported.append({"name": image.name, "file": str(target.name)})
    return exported


def convert_curves_and_text():
    converted = 0
    for obj in list(bpy.context.scene.objects):
        if obj.type not in {"CURVE", "FONT"} or obj.hide_render:
            continue
        bpy.ops.object.select_all(action="DESELECT")
        obj.hide_set(False)
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.convert(target="MESH")
        converted += 1
    return converted


def category_for(obj):
    collection_names = {collection.name for collection in obj.users_collection}
    for category, collection_name in TERRAIN_COLLECTIONS.items():
        if collection_name in collection_names:
            return category
    return "infrastructure"


def apply_spawn_frame_transform():
    rotation = Matrix.Rotation(YAW_RADIANS, 4, "Z")
    transform = rotation @ Matrix.Translation(-SOURCE_ORIGIN)
    for obj in bpy.context.scene.objects:
        if obj.type == "MESH":
            obj.matrix_world = transform @ obj.matrix_world


def add_decimation(objects, ratio, modifier_name):
    for obj in objects:
        if obj.type != "MESH" or len(obj.data.polygons) < 100:
            continue
        modifier = obj.modifiers.new(name=modifier_name, type="DECIMATE")
        modifier.decimate_type = "COLLAPSE"
        modifier.ratio = ratio
        modifier.use_collapse_triangulate = True


def export_category(output_dir, category, objects):
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.hide_set(False)
        obj.select_set(True)
    target = output_dir / "meshes" / f"baixianshan_{category}.obj"
    bpy.ops.wm.obj_export(
        filepath=str(target),
        export_selected_objects=True,
        forward_axis="Y",
        up_axis="Z",
        global_scale=1.0,
        apply_modifiers=True,
        apply_transform=True,
        export_eval_mode="DAG_EVAL_RENDER",
        export_uv=True,
        export_normals=True,
        export_materials=True,
        path_mode="RELATIVE",
        export_triangulated_mesh=True,
        export_curves_as_nurbs=False,
        export_object_groups=True,
        export_material_groups=True,
    )
    add_rock_texture_references(target.with_suffix(".mtl"))
    return target


def add_rock_texture_references(mtl_path):
    """Expose the packed Blender rock texture to OBJ/Assimp consumers.

    Blender's OBJ exporter does not follow the source material's procedural
    Mix/Hue chain, even though the diffuse image is embedded.  Add conventional
    MTL maps without changing the original .blend.
    """
    text = mtl_path.read_text(encoding="utf-8")
    marker = "newmtl V2_Rock_Photoscanned_Matte"
    if marker not in text or "map_Kd ../textures/rock_boulder_dry_diff_4k.jpg" in text:
        return
    lines = text.splitlines()
    output = []
    in_rock = False
    inserted = False
    for line in lines:
        if line.startswith("newmtl "):
            in_rock = line == marker
        output.append(line)
        if in_rock and line.startswith("Kd ") and not inserted:
            output.extend([
                "map_Kd ../textures/rock_boulder_dry_diff_4k.jpg",
                "map_Pr ../textures/rock_boulder_dry_rough_4k.jpg",
                "map_Bump -bm 0.08 ../textures/rock_boulder_dry_disp_4k.png",
            ])
            inserted = True
    mtl_path.write_text("\n".join(output) + "\n", encoding="utf-8")


def obj_stats(path):
    vertices = faces = 0
    bbox_min = Vector((math.inf, math.inf, math.inf))
    bbox_max = Vector((-math.inf, -math.inf, -math.inf))
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if line.startswith("v "):
                fields = line.split()
                point = Vector(tuple(float(value) for value in fields[1:4]))
                bbox_min.x = min(bbox_min.x, point.x)
                bbox_min.y = min(bbox_min.y, point.y)
                bbox_min.z = min(bbox_min.z, point.z)
                bbox_max.x = max(bbox_max.x, point.x)
                bbox_max.y = max(bbox_max.y, point.y)
                bbox_max.z = max(bbox_max.z, point.z)
                vertices += 1
            elif line.startswith("f "):
                faces += 1
    return {
        "file": path.name,
        "bytes": path.stat().st_size,
        "vertices": vertices,
        "triangles": faces,
        "bbox_min": list(bbox_min),
        "bbox_max": list(bbox_max),
    }


def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if len(args) != 1:
        raise SystemExit("usage: blender --background FILE.blend --python export_blend_gazebo_map.py -- OUTPUT_DIR")

    output_dir = Path(args[0]).resolve()
    (output_dir / "meshes").mkdir(parents=True, exist_ok=True)
    textures = extract_packed_images(output_dir / "textures")
    converted = convert_curves_and_text()
    apply_spawn_frame_transform()

    categories = {name: [] for name in (*TERRAIN_COLLECTIONS, "infrastructure")}
    for obj in bpy.context.scene.objects:
        if obj.type != "MESH" or obj.hide_render or not len(obj.data.vertices):
            continue
        categories[category_for(obj)].append(obj)

    for name in TERRAIN_COLLECTIONS:
        add_decimation(
            categories[name], TERRAIN_DECIMATE_RATIO, "GazeboTerrainDecimate")
    add_decimation(
        categories["infrastructure"],
        INFRASTRUCTURE_DECIMATE_RATIO,
        "GazeboInfrastructureDecimate")

    exported = []
    for category, objects in categories.items():
        if not objects:
            continue
        path = export_category(output_dir, category, objects)
        exported.append({
            "category": category,
            "source_objects": len(objects),
            **obj_stats(path),
        })

    metadata = {
        "source_blend": bpy.data.filepath,
        "blender_version": bpy.app.version_string,
        "source_units": "metres",
        "source_origin": list(SOURCE_ORIGIN),
        "source_to_gazebo_yaw_degrees": math.degrees(YAW_RADIANS),
        "gazebo_spawn_xyz": [0.0, 0.0, 0.1404],
        "gazebo_spawn_yaw_degrees": 0.0,
        "terrain_decimate_ratio": TERRAIN_DECIMATE_RATIO,
        "infrastructure_decimate_ratio": INFRASTRUCTURE_DECIMATE_RATIO,
        "converted_curve_and_text_objects": converted,
        "textures": textures,
        "exports": exported,
    }
    (output_dir / "export_metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
