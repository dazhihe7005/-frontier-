#!/usr/bin/env python3
"""Generate Task 1's deterministic, irregular half-elliptical rock cavern."""

import argparse
import math
from pathlib import Path


X_MIN = -2.0
X_MAX = 22.0
HALF_WIDTH = 4.0
HEIGHT = 4.0


def section_scale(x):
    """Slow geological variation while preserving the requested mean size."""
    distance = x - X_MIN
    width = 1.0 + 0.045 * math.sin(0.31 * distance + 0.4)
    width += 0.025 * math.sin(0.79 * distance - 0.8)
    height = 1.0 + 0.055 * math.sin(0.27 * distance - 0.3)
    height += 0.020 * math.sin(0.91 * distance + 1.1)
    return width, height


def wall_relief(x, theta):
    """Deterministic multi-scale rock relief in metres."""
    return (
        0.110 * math.sin(1.37 * x + 3.0 * theta)
        + 0.060 * math.sin(3.11 * x - 7.0 * theta + 0.7)
        + 0.028 * math.sin(7.43 * x + 13.0 * theta - 1.2)
    )


def floor_height(x, lateral_fraction):
    # Keep a traversable but visibly non-planar floor. Relief fades to zero at
    # the arch/floor seam so the shared rock shell remains watertight.
    seam_taper = max(0.0, 1.0 - abs(lateral_fraction) ** 6)
    relief = (
        0.055 * math.sin(1.17 * x + 2.7 * lateral_fraction)
        + 0.030 * math.sin(3.73 * x - 7.1 * lateral_fraction)
        + 0.015 * math.sin(8.19 * x + 11.0 * lateral_fraction)
    )
    return seam_taper * relief


def add_face(faces, a, b, c):
    faces.append((a + 1, b + 1, c + 1))


def generate(path, longitudinal_segments=96, arch_segments=48,
             floor_segments=24):
    if longitudinal_segments < 2 or arch_segments < 4 or floor_segments < 2:
        raise ValueError("mesh resolution is too low")

    vertices = []
    wall_rows = []
    floor_rows = []

    for row in range(longitudinal_segments + 1):
        x = X_MIN + (X_MAX - X_MIN) * row / longitudinal_segments
        width_scale, height_scale = section_scale(x)

        wall_row = []
        for column in range(arch_segments + 1):
            theta = -0.5 * math.pi + math.pi * column / arch_segments
            # Relief acts normal to the nominal half ellipse. It is tapered at
            # both floor seams so wall and floor share exactly the same edge.
            seam_taper = math.cos(theta) ** 0.35
            relief = seam_taper * wall_relief(x, theta)
            y = (HALF_WIDTH * width_scale + relief) * math.sin(theta)
            z = (HEIGHT * height_scale + relief) * math.cos(theta)
            wall_row.append(len(vertices))
            vertices.append((x, y, z))
        wall_rows.append(wall_row)

        floor_row = []
        edge_width = HALF_WIDTH * width_scale
        for column in range(floor_segments + 1):
            fraction = -1.0 + 2.0 * column / floor_segments
            floor_row.append(len(vertices))
            vertices.append((x, edge_width * fraction,
                             floor_height(x, fraction)))
        floor_rows.append(floor_row)

    faces = []
    for row in range(longitudinal_segments):
        wall_a, wall_b = wall_rows[row], wall_rows[row + 1]
        for column in range(arch_segments):
            # Winding points toward the cavern interior.
            add_face(faces, wall_a[column], wall_b[column + 1], wall_b[column])
            add_face(faces, wall_a[column], wall_a[column + 1], wall_b[column + 1])

        floor_a, floor_b = floor_rows[row], floor_rows[row + 1]
        for column in range(floor_segments):
            add_face(faces, floor_a[column], floor_b[column], floor_b[column + 1])
            add_face(faces, floor_a[column], floor_b[column + 1], floor_a[column + 1])

    # Stitch floor edges to the two arch edges. Separate duplicate edge
    # vertices make the floor shading crisp without leaving physical gaps.
    for row in range(longitudinal_segments):
        for wall_column, floor_column in ((0, 0), (-1, -1)):
            a = wall_rows[row][wall_column]
            b = wall_rows[row + 1][wall_column]
            c = floor_rows[row + 1][floor_column]
            d = floor_rows[row][floor_column]
            add_face(faces, a, b, c)
            add_face(faces, a, c, d)

    # Close the far end. Boundary order follows the arch from left to right,
    # then the floor back from right to left. A slightly recessed centre makes
    # the end wall visibly uneven while keeping all triangles well-conditioned.
    boundary = wall_rows[-1] + list(reversed(floor_rows[-1]))
    centre = len(vertices)
    vertices.append((X_MAX + 0.09, 0.0, 1.55))
    for column in range(len(boundary)):
        add_face(faces, centre, boundary[(column + 1) % len(boundary)],
                 boundary[column])

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as output:
        output.write("# 24 x 8 x 4 m nominal irregular half-elliptical goaf\n")
        output.write("# Open entrance x=-2 m; closed irregular end x=22 m\n")
        output.write("o goaf_rock_inner\n")
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
