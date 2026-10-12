#include <gtest/gtest.h>
#include <rog_map/rog_map_core/inflation_geometry.h>

TEST(InflationGeometry, DefaultPreservesSphere) {
    EXPECT_TRUE(rog_map::includeInflationOffset(2, 2, 1, 3, -1));
    EXPECT_FALSE(rog_map::includeInflationOffset(3, 3, 0, 3, -1));
}

TEST(InflationGeometry, ExplicitVerticalExtentKeepsWallAndExcludesFarFloor) {
    EXPECT_TRUE(rog_map::includeInflationOffset(7, 0, 2, 7, 2));
    EXPECT_FALSE(rog_map::includeInflationOffset(7, 0, 3, 7, 2));
    EXPECT_FALSE(rog_map::includeInflationOffset(6, 6, 0, 7, 2));
}
