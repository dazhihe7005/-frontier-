#include <gtest/gtest.h>

#include <super_core/flight_z_contract.h>

namespace {

Eigen::MatrixXd makePolynomial(double constant, double linear,
                               double quadratic) {
    Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
    coefficients(2, 5) = quadratic;
    coefficients(2, 6) = linear;
    coefficients(2, 7) = constant;
    return coefficients;
}

TEST(FlightZContract, AcceptsInteriorSafeTrajectory) {
    const auto polynomial = makePolynomial(1.5, 0.1, -0.1);
    EXPECT_TRUE(super_planner::polynomialInsideFlightZ(polynomial, 1.0, -2.0, 1.8));
}

TEST(FlightZContract, RejectsHiddenInteriorPeak) {
    // Endpoints are 1.6 m, but the middle reaches 1.85 m.
    const auto polynomial = makePolynomial(1.6, 1.0, -1.0);
    EXPECT_FALSE(super_planner::polynomialInsideFlightZ(polynomial, 1.0, -2.0, 1.8));
}

TEST(FlightZContract, RejectsSmallOptimizerOvershoot) {
    const auto polynomial = makePolynomial(1.7522, 0.2, -0.2);
    EXPECT_FALSE(super_planner::polynomialInsideFlightZ(polynomial, 1.0, -2.0, 1.8));
}

TEST(FlightZContract, RejectsInvalidDuration) {
    const auto polynomial = makePolynomial(1.5, 0.0, 0.0);
    EXPECT_FALSE(super_planner::polynomialInsideFlightZ(polynomial, 0.0, -2.0, 1.8));
}

}  // namespace
