#pragma once

#include <cstddef>
#include <optional>

namespace super_planner {

template <typename Path, typename IsOccupied>
std::optional<std::size_t> firstUnoccupiedPathIndex(
    const Path& path, IsOccupied is_occupied) {
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (!is_occupied(path[i])) return i;
    }
    return std::nullopt;
}

}  // namespace super_planner
