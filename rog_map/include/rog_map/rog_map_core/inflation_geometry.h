#pragma once

#include <cstdlib>

namespace rog_map {

// A negative vertical step preserves the upstream spherical inflation.
// An explicit vertical step uses a horizontal disk extruded only through
// the vehicle's vertical body height; it does not turn floor stand-off into
// a horizontal wall stand-off.
inline bool includeInflationOffset(int dx, int dy, int dz,
                                   int horizontal_step, int vertical_step) {
    if (vertical_step < 0) {
        return horizontal_step == 1 ||
               dx * dx + dy * dy + dz * dz <= horizontal_step * horizontal_step;
    }
    return dx * dx + dy * dy <= horizontal_step * horizontal_step &&
           std::abs(dz) <= vertical_step;
}

}  // namespace rog_map
