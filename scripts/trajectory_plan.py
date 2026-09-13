#!/usr/bin/env python3
"""Data-driven local trajectory segments used by the ground station."""

import math
from dataclasses import dataclass


@dataclass
class TrajectoryPoint:
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    yaw: float


class TrajectoryPlan:
    """Compile and sample a sequence of local ENU trajectory segments."""

    SUPPORTED = ("takeoff", "hover", "line", "arc")

    def __init__(self, origin, segments):
        self.origin = dict(origin)
        self.segments = [dict(segment) for segment in segments]
        self.compiled = []
        self.duration = 0.0
        self._compile()

    @staticmethod
    def _number(segment, key, default=0.0):
        try:
            return float(segment.get(key, default))
        except (TypeError, ValueError):
            raise ValueError("segment %s must be numeric" % key)

    def _compile(self):
        current = dict(self.origin)
        elapsed = 0.0
        for segment in self.segments:
            kind = str(segment.get("kind", "")).lower()
            if kind not in self.SUPPORTED:
                raise ValueError("unsupported segment type: %s" % kind)
            start = dict(current)
            compiled = {"kind": kind, "start": start, "begin": elapsed}

            if kind == "takeoff":
                height = self._number(segment, "height")
                climb_speed = self._number(segment, "climb_speed", 0.5)
                if height < 0.0 or climb_speed <= 0.0:
                    raise ValueError("takeoff height must be non-negative and speed positive")
                compiled.update({"height": height, "speed": climb_speed})
                compiled["duration"] = height / climb_speed
                current["z"] += height
            elif kind == "hover":
                duration = self._number(segment, "duration")
                if duration < 0.0:
                    raise ValueError("hover duration must be non-negative")
                compiled["duration"] = duration
            elif kind == "line":
                distance = self._number(segment, "distance")
                speed = self._number(segment, "speed")
                if distance < 0.0 or speed <= 0.0:
                    raise ValueError("line distance must be non-negative and speed positive")
                compiled.update({"distance": distance, "speed": speed})
                compiled["duration"] = distance / speed
                current["x"] += distance * math.cos(current["yaw"])
                current["y"] += distance * math.sin(current["yaw"])
            else:
                radius = self._number(segment, "radius")
                speed = self._number(segment, "speed")
                angle_deg = self._number(segment, "angle_deg")
                direction = int(segment.get("direction", 1))
                if radius <= 0.0 or speed <= 0.0 or angle_deg < 0.0:
                    raise ValueError("arc radius/speed must be positive and angle non-negative")
                if direction not in (-1, 1):
                    raise ValueError("arc direction must be +1 or -1")
                angle_rad = math.radians(angle_deg)
                side = float(direction)
                center_x = current["x"] - side * radius * math.sin(current["yaw"])
                center_y = current["y"] + side * radius * math.cos(current["yaw"])
                initial_angle = math.atan2(
                    current["y"] - center_y, current["x"] - center_x
                )
                omega = speed / radius
                compiled.update(
                    {
                        "radius": radius,
                        "speed": speed,
                        "angle_rad": angle_rad,
                        "direction": direction,
                        "center_x": center_x,
                        "center_y": center_y,
                        "initial_angle": initial_angle,
                        "omega": omega,
                    }
                )
                compiled["duration"] = angle_rad / omega
                current["x"] = center_x + radius * math.cos(initial_angle + side * angle_rad)
                current["y"] = center_y + radius * math.sin(initial_angle + side * angle_rad)
                current["yaw"] = wrap_yaw(current["yaw"] + side * angle_rad)

            compiled["end"] = dict(current)
            self.compiled.append(compiled)
            elapsed += compiled["duration"]
        self.duration = elapsed
        self.end_point = TrajectoryPoint(
            current["x"], current["y"], current["z"], 0.0, 0.0, 0.0, current["yaw"]
        )

    def sample(self, elapsed):
        if not self.compiled:
            return self.end_point, True, -1
        elapsed = max(0.0, float(elapsed))
        if elapsed >= self.duration:
            return self.end_point, True, len(self.compiled) - 1

        selected = self.compiled[-1]
        for candidate in self.compiled:
            if elapsed < candidate["begin"] + candidate["duration"]:
                selected = candidate
                break
        local = elapsed - selected["begin"]
        start = selected["start"]
        kind = selected["kind"]

        if kind == "takeoff":
            progress = min(selected["height"], local * selected["speed"])
            vz = selected["speed"] if progress < selected["height"] else 0.0
            point = TrajectoryPoint(
                start["x"], start["y"], start["z"] + progress, 0.0, 0.0, vz, start["yaw"]
            )
        elif kind == "hover":
            point = TrajectoryPoint(
                start["x"], start["y"], start["z"], 0.0, 0.0, 0.0, start["yaw"]
            )
        elif kind == "line":
            distance = min(selected["distance"], local * selected["speed"])
            point = TrajectoryPoint(
                start["x"] + distance * math.cos(start["yaw"]),
                start["y"] + distance * math.sin(start["yaw"]),
                start["z"],
                selected["speed"] * math.cos(start["yaw"]),
                selected["speed"] * math.sin(start["yaw"]),
                0.0,
                start["yaw"],
            )
        else:
            side = float(selected["direction"])
            angle = selected["initial_angle"] + side * selected["omega"] * local
            point = TrajectoryPoint(
                selected["center_x"] + selected["radius"] * math.cos(angle),
                selected["center_y"] + selected["radius"] * math.sin(angle),
                start["z"],
                -side * selected["speed"] * math.sin(angle),
                side * selected["speed"] * math.cos(angle),
                0.0,
                wrap_yaw(math.atan2(side * math.cos(angle), -side * math.sin(angle))),
            )
        return point, False, self.compiled.index(selected)


def wrap_yaw(value):
    wrapped = math.fmod(float(value), 2.0 * math.pi)
    if wrapped > math.pi:
        wrapped -= 2.0 * math.pi
    elif wrapped < -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped
