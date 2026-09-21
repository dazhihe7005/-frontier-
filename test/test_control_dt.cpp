#include "mine_uav_control/control_dt.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace mine_uav_control {

TEST(Task2ControlDt, UsesActualPositiveInterval) {
  EXPECT_DOUBLE_EQ(task2ControlDt(0.052, 0.05), 0.052);
  EXPECT_DOUBLE_EQ(task2ControlDt(1.2, 0.05), 1.2);
}

TEST(Task2ControlDt, ZeroActualIntervalUsesExpectedPeriod) {
  EXPECT_DOUBLE_EQ(task2ControlDt(0.0, 0.05), 0.05);
}

TEST(Task2ControlDt, InvalidIntervalsRemainFailClosed) {
  EXPECT_LT(task2ControlDt(-0.01, 0.05), 0.0);
  EXPECT_TRUE(std::isnan(task2ControlDt(
      std::numeric_limits<double>::quiet_NaN(), 0.05)));
}

}  // namespace mine_uav_control
