#include <gtest/gtest.h>

#include <rog_map/rog_map.h>

namespace {

class TimedROGMap final : public rog_map::ROGMap {
public:
    explicit TimedROGMap(const std::string& config_path) {
        cfg_ = rog_map::Config(config_path);
        init();
    }

    void observe(double stamp, const rog_map::Vec3f& position) {
        now_ = stamp;
        updateRobotState(std::make_pair(position, super_utils::Quatf::Identity()));
    }

private:
    const double getSystemWalltimeNow() override { return now_; }
    double now_{0.0};
};

TEST(RobotStateVelocity, UsesPoseMotionAndRejectsJumps) {
    TimedROGMap map(ROG_MAP_TEST_CONFIG_PATH);
    map.observe(1.0, rog_map::Vec3f(0.0, 0.0, 0.0));
    EXPECT_NEAR(map.getRobotState().v.norm(), 0.0, 1e-6);
    map.observe(1.1, rog_map::Vec3f(0.1, 0.0, 0.0));
    EXPECT_NEAR(map.getRobotState().v.x(), 0.35, 1e-5);
    map.observe(1.2, rog_map::Vec3f(0.2, 0.0, 0.0));
    EXPECT_NEAR(map.getRobotState().v.x(), 0.5775, 1e-5);
    map.observe(1.3, rog_map::Vec3f(5.2, 0.0, 0.0));
    EXPECT_NEAR(map.getRobotState().v.norm(), 0.0, 1e-6);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
