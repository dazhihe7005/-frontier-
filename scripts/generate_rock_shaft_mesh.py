#!/usr/bin/env python3
"""Generate the deterministic 5 m x 500 m irregular rock-shaft mesh."""

import argparse
import math
from pathlib import Path


def wall_radius(theta, depth):
    # Multi-scale deterministic relief; nominal radius stays 2.50 m.
    return (
        2.50
        + 0.075 * math.sin(3.0 * theta + 0.071 * depth)
        + 0.045 * math.sin(7.0 * theta - 0.137 * depth)
        + 0.025 * math.sin(13.0 * theta + 0.311 * depth)
    )


def floor_height(radius, theta):
    taper = min(1.0, radius / 0.5)
    return -499.75 + taper * (
        0.055 * math.sin(4.0 * theta + 1.7 * radius)
        + 0.025 * math.sin(9.0 * theta - 2.3 * radius)
    )


def generate(path, angular_segments=32, vertical_step=1.0):
    top_z = 0.25
    bottom_z = -499.75
    vertical_segments = int(round((top_z - bottom_z) / vertical_step))
    vertices = []
    for row in range(vertical_segments + 1):
        z = top_z - row * vertical_step
        depth = top_z - z
        for column in range(angular_segments):
            theta = 2.0 * math.pi * column / angular_segments
            radius = wall_radius(theta, depth)
            vertices.append((radius * math.cos(theta), radius * math.sin(theta), z))

    faces = []
    for row in range(vertical_segments):
        for column in range(angular_segments):
            nxt = (column + 1) % angular_segments
            top = row * angular_segments + column + 1
            top_next = row * angular_segments + nxt + 1
            low = (row + 1) * angular_segments + column + 1
            low_next = (row + 1) * angular_segments + nxt + 1
            # Inward-facing winding for a vehicle inside the shaft.
            faces.append((top, low_next, low))
            faces.append((top, top_next, low_next))

    # Irregular floor, connected to the last wall ring. Centre remains at the
    # exact 500 m reference depth; four radial rings add rock-like relief.
    floor_rings = 4
    floor_indices = []
    centre_index = len(vertices) + 1
    vertices.append((0.0, 0.0, bottom_z))
    for ring in range(1, floor_rings + 1):
        ring_indices = []
        fraction = ring / floor_rings
        for column in range(angular_segments):
            theta = 2.0 * math.pi * column / angular_segments
            boundary = wall_radius(theta, 500.0)
            radius = fraction * boundary
            z = bottom_z if ring == floor_rings else floor_height(radius, theta)
            ring_indices.append(len(vertices) + 1)
            vertices.append((radius * math.cos(theta), radius * math.sin(theta), z))
        floor_indices.append(ring_indices)

    first_ring = floor_indices[0]
    for column in range(angular_segments):
        nxt = (column + 1) % angular_segments
        faces.append((centre_index, first_ring[column], first_ring[nxt]))
    for ring in range(floor_rings - 1):
        inner = floor_indices[ring]
        outer = floor_indices[ring + 1]
        for column in range(angular_segments):
            nxt = (column + 1) % angular_segments
            faces.append((inner[column], outer[column], outer[nxt]))
            faces.append((inner[column], outer[nxt], inner[nxt]))

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as output:
        output.write("# 5 m nominal diameter, 500 m deep irregular mine shaft\n")
        output.write("o shaft_rock_inner\n")
        for x, y, z in vertices:
            output.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        output.write("s 1\n")
        for a, b, c in faces:
            output.write(f"f {a} {b} {c}\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    generate(arguments.output)


if __name__ == "__main__":
    main()
