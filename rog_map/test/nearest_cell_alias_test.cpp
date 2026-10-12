#include <gtest/gtest.h>

#include <rog_map/rog_map.h>

namespace {

class TestROGMap final : public rog_map::ROGMap {
public:
    explicit TestROGMap(const std::string& config_path) {
        cfg_ = rog_map::Config(config_path);
        init();
    }

private:
    const double getSystemWalltimeNow() override {
        return 0.0;
    }
};

TEST(NearestCellSearch, AliasedInputOutputHonorsMaximumDistance) {
    TestROGMap map(ROG_MAP_TEST_CONFIG_PATH);
    const rog_map::Vec3f query(0.03, 0.04, 0.02);

    rog_map::Vec3f separate_output;
    const bool separate_inf_result = map.getNearestInfCellNot(
        super_utils::OCCUPIED, query, separate_output, 0.0);

    rog_map::Vec3f aliased_inf_query = query;
    const bool aliased_inf_result = map.getNearestInfCellNot(
        super_utils::OCCUPIED, aliased_inf_query, aliased_inf_query, 0.0);

    EXPECT_FALSE(separate_inf_result);
    EXPECT_EQ(separate_inf_result, aliased_inf_result)
        << "Using the same vector for input and output must not bypass max_dis";

    rog_map::Vec3f separate_prob_output;
    const bool separate_prob_result = map.getNearestCellNot(
        super_utils::OCCUPIED, query, separate_prob_output, 0.0);

    rog_map::Vec3f aliased_prob_query = query;
    const bool aliased_prob_result = map.getNearestCellNot(
        super_utils::OCCUPIED, aliased_prob_query, aliased_prob_query, 0.0);

    EXPECT_FALSE(separate_prob_result);
    EXPECT_EQ(separate_prob_result, aliased_prob_result)
        << "Probabilistic-map search has the same aliasing contract";
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
