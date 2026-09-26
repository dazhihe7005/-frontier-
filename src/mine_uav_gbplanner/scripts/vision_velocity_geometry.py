#!/usr/bin/env python3
"""Pure, fail-closed visual-pose velocity estimation for the SITL A/B."""

from collections import deque
import math


def world_to_body(velocity, quaternion):
    """Rotate an ENU world velocity into the FLU child frame."""
    x, y, z, w = quaternion
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1.0e-8:
        raise ValueError("invalid pose quaternion")
    x, y, z, w = (value/norm for value in (x, y, z, w))
    # Conjugate quaternion rotation, equivalent to R(world<-body)^T * v.
    r00 = 1.0-2.0*(y*y+z*z)
    r01 = 2.0*(x*y-z*w)
    r02 = 2.0*(x*z+y*w)
    r10 = 2.0*(x*y+z*w)
    r11 = 1.0-2.0*(x*x+z*z)
    r12 = 2.0*(y*z-x*w)
    r20 = 2.0*(x*z-y*w)
    r21 = 2.0*(y*z+x*w)
    r22 = 1.0-2.0*(x*x+y*y)
    vx, vy, vz = velocity
    return (r00*vx+r10*vy+r20*vz,
            r01*vx+r11*vy+r21*vz,
            r02*vx+r12*vy+r22*vz)


class VelocityWindow:
    def __init__(self, min_span=0.18, max_span=0.40,
                 max_speed=3.5, max_acceleration=4.0):
        self.min_span = min_span
        self.max_span = max_span
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self._samples = deque(maxlen=8)
        self._last_velocity = None
        self._last_velocity_stamp = None

    def clear(self):
        self._samples.clear()
        self._last_velocity = None
        self._last_velocity_stamp = None

    def add(self, stamp, xyz):
        xyz = tuple(xyz)
        if not math.isfinite(stamp) or not all(math.isfinite(v) for v in xyz):
            self.clear()
            return None
        if self._samples and (stamp <= self._samples[-1][0] or
                              stamp-self._samples[-1][0] > self.max_span):
            self.clear()
        self._samples.append((stamp, xyz))
        while len(self._samples) > 1 and \
                stamp-self._samples[0][0] > self.max_span:
            self._samples.popleft()
        if len(self._samples) < 2:
            return None
        oldest_stamp, oldest_xyz = self._samples[0]
        dt = stamp-oldest_stamp
        if dt < self.min_span:
            return None
        velocity = tuple((xyz[i]-oldest_xyz[i])/dt for i in range(3))
        speed = math.sqrt(sum(v*v for v in velocity))
        if speed > self.max_speed:
            self.clear()
            return None
        if self._last_velocity is not None:
            interval = stamp-self._last_velocity_stamp
            acceleration = math.sqrt(sum(
                (velocity[i]-self._last_velocity[i])**2 for i in range(3))) / interval
            if acceleration > self.max_acceleration:
                self.clear()
                return None
        self._last_velocity = velocity
        self._last_velocity_stamp = stamp
        return velocity
