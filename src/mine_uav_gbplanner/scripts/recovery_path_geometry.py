#!/usr/bin/env python3
"""Pure geometry for reversing an actually observed vehicle path."""

import math


def distance(left, right):
    return math.dist(left, right)


def blocked_path_needs_backtrack(previous_retry, current, reset_distance):
    """Try a fresh plan once; retrace only on a repeated nearby guard stop."""
    if reset_distance <= 0:
        raise ValueError("invalid retry reset distance")
    return (previous_retry is not None and
            distance(previous_retry, current) < reset_distance)


def horizontal_progress(anchor, current):
    """Ignore vertical escape oscillation when testing exploration progress."""
    return math.hypot(current[0]-anchor[0], current[1]-anchor[1])


def reverse_path_has_progress(points, travelled, min_horizontal=1.0,
                              min_efficiency=0.30):
    """Reject a 6 m rewind that merely replays a local orbit or U-turn.

    Reversing observed poses is safe only if it actually exits the blocked
    neighborhood. Net XY displacement is intentionally direction-agnostic;
    legitimate curved corridors pass, while a loop returning near its start
    cannot be called a useful backtrack.
    """
    if len(points) < 2 or travelled <= 0.0 or not math.isfinite(travelled):
        return False
    displacement = horizontal_progress(points[0], points[-1])
    return (displacement >= min_horizontal and
            displacement >= min_efficiency*travelled)


def reverse_observed_path(current, history, max_distance=6.0,
                          max_gap=0.30, min_step=0.05):
    """Return [current, ...] along dense observed poses, never commands.

    History is chronological XYZ tuples. A localization gap, discontinuity,
    or connector longer than max_gap terminates the reverse route. The caller
    must still use a live obstacle guard: a once-traversed path can change.
    """
    if max_distance <= 0 or max_gap <= 0 or min_step <= 0:
        raise ValueError("invalid reverse-path limits")
    points = [tuple(current)]
    travelled = 0.0
    for sample in reversed(history):
        sample = tuple(sample)
        if not all(math.isfinite(value) for value in sample):
            break
        gap = distance(points[-1], sample)
        if gap < min_step:
            continue
        if gap > max_gap:
            break
        if travelled + gap > max_distance:
            ratio = (max_distance-travelled)/gap
            sample = tuple(points[-1][axis] + ratio*(sample[axis]-points[-1][axis])
                           for axis in range(3))
            gap = distance(points[-1], sample)
        if gap < min_step:
            break
        points.append(sample)
        travelled += gap
        if travelled >= max_distance-1e-6:
            break
    return points, travelled
