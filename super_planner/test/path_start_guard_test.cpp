#include <gtest/gtest.h>
#include <super_core/path_start_guard.h>
#include <vector>

TEST(PathStartGuard, RejectsEmptyPath) {
    const std::vector<int> path;
    EXPECT_FALSE(super_planner::firstUnoccupiedPathIndex(path,
        [](int) { return true; }).has_value());
}

TEST(PathStartGuard, RejectsFullyOccupiedPathWithoutOutOfBoundsRead) {
    const std::vector<int> path{1, 2, 3};
    std::size_t calls = 0;
    const auto index = super_planner::firstUnoccupiedPathIndex(path,
        [&calls](int) { ++calls; return true; });
    EXPECT_FALSE(index.has_value());
    EXPECT_EQ(calls, path.size());
}

TEST(PathStartGuard, FindsFirstFreePoint) {
    const std::vector<int> path{1, 2, 3};
    const auto index = super_planner::firstUnoccupiedPathIndex(path,
        [](int point) { return point < 3; });
    ASSERT_TRUE(index.has_value());
    EXPECT_EQ(*index, 2U);
}
