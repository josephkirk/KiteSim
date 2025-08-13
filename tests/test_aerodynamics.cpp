#include "kite_sim/aerodynamics.h"
#include "kite_sim/kite_geometry.h"
#include "kite_sim/kite_state.h"
#include "kite_sim/wind_model.h"
#include <gtest/gtest.h>
#include <cmath>

namespace kite_sim {

class AerodynamicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple test kite geometry
        geometry_ = KiteGeometry::createTestKite();
        
        // Set up simple aerodynamic coefficients for testing
        AeroCoefficients coeffs;
        std::vector<double> alpha_deg = {-10, 0, 10, 20};
        std::vector<double> cl_vals = {-0.5, 0.0, 1.0, 1.5};
        std::vector<double> cd_vals = {0.05, 0.02, 0.05, 0.15};
        std::vector<double> cm_vals = {0.1, 0.0, -0.1, -0.2};
        
        coeffs.setCL1D(alpha_deg, cl_vals);
        coeffs.setCD1D(alpha_deg, cd_vals);
        coeffs.setCM1D(alpha_deg, cm_vals);
        
        geometry_.setAeroCoefficients(coeffs);
        
        // Create aerodynamics calculator
        aero_calc_ = std::make_unique<AerodynamicsCalculator>(geometry_);
        
        // Set up test state
        test_state_ = KiteState(
            Vector3(0, 0, 100),           // 100m altitude
            Vector3(10, 0, 0),            // 10 m/s forward velocity
            Quaternion::identity(),        // Level attitude
            Vector3(0, 0, 0),             // No angular velocity
            0.0                           // Time = 0
        );
        
        // Set up test wind
        test_wind_ = WindVector(Vector3(15, 0, 0)); // 15 m/s headwind
    }
    
    KiteGeometry geometry_;
    std::unique_ptr<AerodynamicsCalculator> aero_calc_;
    KiteState test_state_;
    WindVector test_wind_;
    
    // Helper function to check if values are approximately equal
    bool isApproxEqual(double a, double b, double tolerance = 1e-6) {
        return std::abs(a - b) < tolerance;
    }
};

// Test basic construction and configuration
TEST_F(AerodynamicsTest, Construction) {
    // Test default constructor
    AerodynamicsCalculator calc1;
    EXPECT_FALSE(calc1.isConfigured());
    
    // Test constructor with geometry
    AerodynamicsCalculator calc2(geometry_);
    EXPECT_TRUE(calc2.isConfigured());
    
    // Test setGeometry
    calc1.setGeometry(geometry_);
    EXPECT_TRUE(calc1.isConfigured());
}

// Test relative wind calculation
TEST_F(AerodynamicsTest, RelativeWindCalculation) {
    // Test case: kite moving at 10 m/s, wind at 15 m/s, both in +x direction
    // Relative wind should be 5 m/s in +x direction
    Vector3 relative_wind = aero_calc_->calculateRelativeWind(test_state_, test_wind_);
    
    EXPECT_NEAR(relative_wind.x(), 5.0, 1e-6);
    EXPECT_NEAR(relative_wind.y(), 0.0, 1e-6);
    EXPECT_NEAR(relative_wind.z(), 0.0, 1e-6);
    
    // Test with crosswind
    WindVector crosswind(Vector3(10, 5, 0)); // Wind with crosswind component
    relative_wind = aero_calc_->calculateRelativeWind(test_state_, crosswind);
    
    EXPECT_NEAR(relative_wind.x(), 0.0, 1e-6);  // 10 - 10 = 0
    EXPECT_NEAR(relative_wind.y(), 5.0, 1e-6);  // 5 - 0 = 5
    EXPECT_NEAR(relative_wind.z(), 0.0, 1e-6);
}

// Test flow angle calculations
TEST_F(AerodynamicsTest, FlowAngleCalculation) {
    // Test zero angle of attack and sideslip
    Vector3 relative_wind_body(5, 0, 0); // Pure forward flow
    auto [alpha, beta] = aero_calc_->calculateFlowAngles(relative_wind_body);
    
    EXPECT_NEAR(alpha, 0.0, 1e-6); // Zero angle of attack
    EXPECT_NEAR(beta, 0.0, 1e-6);  // Zero sideslip
    
    // Test positive angle of attack
    relative_wind_body = Vector3(5, 0, 2); // Downward component
    std::tie(alpha, beta) = aero_calc_->calculateFlowAngles(relative_wind_body);
    
    double expected_alpha = std::atan2(2, 5);
    EXPECT_NEAR(alpha, expected_alpha, 1e-6);
    EXPECT_NEAR(beta, 0.0, 1e-6);
    
    // Test sideslip angle
    relative_wind_body = Vector3(5, 3, 0); // Side component
    std::tie(alpha, beta) = aero_calc_->calculateFlowAngles(relative_wind_body);
    
    double expected_beta = std::atan2(3, 5);
    EXPECT_NEAR(alpha, 0.0, 1e-6);
    EXPECT_NEAR(beta, expected_beta, 1e-6);
}

// Test dynamic pressure calculation
TEST_F(AerodynamicsTest, DynamicPressureCalculation) {
    Vector3 relative_wind(5, 0, 0); // 5 m/s airspeed
    double q = aero_calc_->calculateDynamicPressure(relative_wind);
    
    double expected_q = 0.5 * AerodynamicsCalculator::AIR_DENSITY * 25; // 0.5 * rho * V^2
    EXPECT_NEAR(q, expected_q, 1e-6);
    
    // Test with different airspeed
    relative_wind = Vector3(3, 4, 0); // 5 m/s magnitude (3-4-5 triangle)
    q = aero_calc_->calculateDynamicPressure(relative_wind);
    EXPECT_NEAR(q, expected_q, 1e-6);
}

// Test coordinate transformations
TEST_F(AerodynamicsTest, CoordinateTransformations) {
    Vector3 test_vector(1, 2, 3);
    Quaternion identity = Quaternion::identity();
    
    // Test identity transformation
    Vector3 transformed = aero_calc_->transformWindToBody(test_vector, identity);
    EXPECT_NEAR(transformed.x(), test_vector.x(), 1e-6);
    EXPECT_NEAR(transformed.y(), test_vector.y(), 1e-6);
    EXPECT_NEAR(transformed.z(), test_vector.z(), 1e-6);
    
    // Test round-trip transformation
    Vector3 back_transformed = aero_calc_->transformBodyToWorld(transformed, identity);
    EXPECT_NEAR(back_transformed.x(), test_vector.x(), 1e-6);
    EXPECT_NEAR(back_transformed.y(), test_vector.y(), 1e-6);
    EXPECT_NEAR(back_transformed.z(), test_vector.z(), 1e-6);
}

// Test full force calculation
TEST_F(AerodynamicsTest, ForceCalculation) {
    // Test with known conditions
    AeroForces forces = aero_calc_->calculateForces(test_state_, test_wind_);
    
    // Verify basic properties
    EXPECT_TRUE(forces.isValid());
    EXPECT_GT(forces.dynamic_pressure, 0.0);
    EXPECT_GT(forces.lift, 0.0); // Should have positive lift for positive alpha
    EXPECT_GT(forces.drag, 0.0); // Should always have positive drag
    
    // Check that angle of attack is reasonable (small for this test case)
    EXPECT_LT(std::abs(forces.angle_of_attack), M_PI/4); // Less than 45 degrees
    EXPECT_LT(std::abs(forces.sideslip_angle), M_PI/4);  // Less than 45 degrees
}

// Test with different flight conditions
TEST_F(AerodynamicsTest, DifferentFlightConditions) {
    // Test with higher angle of attack
    KiteState high_alpha_state(
        Vector3(0, 0, 100),
        Vector3(5, 0, 2),             // Velocity with upward component
        Quaternion::identity(),
        Vector3(0, 0, 0),
        0.0
    );
    
    AeroForces forces = aero_calc_->calculateForces(high_alpha_state, test_wind_);
    EXPECT_TRUE(forces.isValid());
    EXPECT_GT(forces.angle_of_attack, 0.0); // Should have positive angle of attack
    
    // Test with sideslip
    KiteState sideslip_state(
        Vector3(0, 0, 100),
        Vector3(10, 3, 0),            // Velocity with side component
        Quaternion::identity(),
        Vector3(0, 0, 0),
        0.0
    );
    
    forces = aero_calc_->calculateForces(sideslip_state, test_wind_);
    EXPECT_TRUE(forces.isValid());
    EXPECT_NE(forces.sideslip_angle, 0.0); // Should have non-zero sideslip
}

// Test control input effects
TEST_F(AerodynamicsTest, ControlInputs) {
    ControlInputs controls;
    controls.elevator = 0.1; // 0.1 rad elevator deflection
    
    AeroForces forces_with_control = aero_calc_->calculateForces(test_state_, test_wind_, controls);
    AeroForces forces_no_control = aero_calc_->calculateForces(test_state_, test_wind_);
    
    EXPECT_TRUE(forces_with_control.isValid());
    EXPECT_TRUE(forces_no_control.isValid());
    
    // Forces should be different with control input
    // (exact comparison depends on coefficient tables)
    EXPECT_NE(forces_with_control.pitch_moment, forces_no_control.pitch_moment);
}

// Test input validation
TEST_F(AerodynamicsTest, InputValidation) {
    // Test with invalid state
    KiteState invalid_state;
    invalid_state.setPosition(Vector3(NAN, 0, 0)); // Invalid position
    
    EXPECT_THROW(aero_calc_->calculateForces(invalid_state, test_wind_), std::invalid_argument);
    
    // Test with invalid wind
    WindVector invalid_wind(Vector3(NAN, 0, 0));
    EXPECT_THROW(aero_calc_->calculateForces(test_state_, invalid_wind), std::invalid_argument);
    
    // Test with invalid control
    ControlInputs invalid_control;
    invalid_control.elevator = 2.0; // Too large deflection
    EXPECT_THROW(aero_calc_->calculateForces(test_state_, test_wind_, invalid_control), 
                 std::invalid_argument);
}

// Test unconfigured calculator
TEST_F(AerodynamicsTest, UnconfiguredCalculator) {
    AerodynamicsCalculator unconfigured_calc;
    EXPECT_THROW(unconfigured_calc.calculateForces(test_state_, test_wind_), std::runtime_error);
}

// Test analytical comparison for simple case
TEST_F(AerodynamicsTest, AnalyticalComparison) {
    // Create a simple case where we can predict the result
    // Zero angle of attack, known coefficients
    
    // Set up state with zero relative angle of attack
    KiteState level_state(
        Vector3(0, 0, 100),
        Vector3(10, 0, 0),            // Same speed as wind
        Quaternion::identity(),
        Vector3(0, 0, 0),
        0.0
    );
    
    WindVector same_wind(Vector3(10, 0, 0)); // Same as kite velocity
    
    AeroForces forces = aero_calc_->calculateForces(level_state, same_wind);
    
    // With zero relative wind, forces should be zero
    EXPECT_NEAR(forces.dynamic_pressure, 0.0, 1e-6);
    EXPECT_NEAR(forces.lift, 0.0, 1e-6);
    EXPECT_NEAR(forces.drag, 0.0, 1e-6);
}

// Test AeroForces utility functions
TEST_F(AerodynamicsTest, AeroForcesUtilities) {
    AeroForces forces;
    
    // Test initial state
    EXPECT_TRUE(forces.isValid());
    
    // Test reset
    forces.lift = 100.0;
    forces.drag = 50.0;
    forces.reset();
    EXPECT_EQ(forces.lift, 0.0);
    EXPECT_EQ(forces.drag, 0.0);
    
    // Test invalid state
    forces.dynamic_pressure = -1.0; // Invalid negative pressure
    EXPECT_FALSE(forces.isValid());
    
    forces.lift = NAN; // Invalid NaN value
    EXPECT_FALSE(forces.isValid());
}

// Test ControlInputs validation
TEST_F(AerodynamicsTest, ControlInputsValidation) {
    ControlInputs valid_controls(0.1, -0.05, 0.2);
    EXPECT_TRUE(valid_controls.isValid());
    
    ControlInputs invalid_controls(2.0, 0.0, 0.0); // Too large deflection
    EXPECT_FALSE(invalid_controls.isValid());
    
    ControlInputs default_controls;
    EXPECT_TRUE(default_controls.isValid());
}

// Test ControlInputs utility functions
TEST_F(AerodynamicsTest, ControlInputsUtilities) {
    ControlInputs controls(0.1, -0.2, 0.15);
    
    // Test getMaxDeflection
    EXPECT_NEAR(controls.getMaxDeflection(), 0.2, 1e-6);
    
    // Test hasDeflection
    EXPECT_TRUE(controls.hasDeflection());
    
    ControlInputs zero_controls;
    EXPECT_FALSE(zero_controls.hasDeflection());
    
    // Test reset
    controls.reset();
    EXPECT_FALSE(controls.hasDeflection());
    EXPECT_EQ(controls.elevator, 0.0);
    EXPECT_EQ(controls.rudder, 0.0);
    EXPECT_EQ(controls.aileron, 0.0);
}

// Test ControlEffectiveness validation and utilities
TEST_F(AerodynamicsTest, ControlEffectivenessValidation) {
    ControlEffectiveness valid_effectiveness;
    EXPECT_TRUE(valid_effectiveness.isValid());
    
    // Test with invalid values
    ControlEffectiveness invalid_effectiveness;
    invalid_effectiveness.elevator_cl_effectiveness = NAN;
    EXPECT_FALSE(invalid_effectiveness.isValid());
    
    // Test reset
    valid_effectiveness.elevator_cl_effectiveness = 999.0;
    valid_effectiveness.reset();
    EXPECT_NEAR(valid_effectiveness.elevator_cl_effectiveness, 0.5, 1e-6);
    
    // Test JSON serialization
    std::string json = valid_effectiveness.toJson();
    EXPECT_FALSE(json.empty());
    EXPECT_TRUE(valid_effectiveness.fromJson(json));
}

// Test control effectiveness calculations
TEST_F(AerodynamicsTest, ControlEffectivenessCalculations) {
    // Set up control effectiveness
    ControlEffectiveness effectiveness;
    effectiveness.elevator_cl_effectiveness = 0.5;
    effectiveness.elevator_cd_effectiveness = 0.1;
    effectiveness.elevator_cm_effectiveness = -1.0;
    effectiveness.aileron_cl_effectiveness = 0.2;
    effectiveness.aileron_cd_effectiveness = 0.05;
    
    aero_calc_->setControlEffectiveness(effectiveness);
    
    // Test control effects on CL
    ControlInputs controls(0.1, 0.0, 0.05); // elevator = 0.1, aileron = 0.05
    double delta_cl = aero_calc_->calculateControlEffectOnCL(controls);
    double expected_cl = 0.5 * 0.1 + 0.2 * 0.05; // elevator + aileron effects
    EXPECT_NEAR(delta_cl, expected_cl, 1e-6);
    
    // Test control effects on CD
    double delta_cd = aero_calc_->calculateControlEffectOnCD(controls);
    double expected_cd = 0.1 * 0.1 + 0.05 * 0.05; // elevator + aileron effects (absolute values)
    EXPECT_NEAR(delta_cd, expected_cd, 1e-6);
    
    // Test control effects on CM
    double delta_cm = aero_calc_->calculateControlEffectOnCM(controls);
    double expected_cm = -1.0 * 0.1; // only elevator affects pitching moment
    EXPECT_NEAR(delta_cm, expected_cm, 1e-6);
}

// Test control system integration in force calculation
TEST_F(AerodynamicsTest, ControlSystemIntegration) {
    // Set up control effectiveness
    ControlEffectiveness effectiveness;
    effectiveness.elevator_cl_effectiveness = 0.5;
    effectiveness.elevator_cd_effectiveness = 0.1;
    effectiveness.elevator_cm_effectiveness = -1.0;
    aero_calc_->setControlEffectiveness(effectiveness);
    
    // Calculate forces without control
    AeroForces forces_no_control = aero_calc_->calculateForces(test_state_, test_wind_);
    
    // Calculate forces with elevator deflection
    ControlInputs controls;
    controls.elevator = 0.1; // 0.1 rad elevator deflection
    AeroForces forces_with_control = aero_calc_->calculateForces(test_state_, test_wind_, controls);
    
    // Verify both calculations are valid
    EXPECT_TRUE(forces_no_control.isValid());
    EXPECT_TRUE(forces_with_control.isValid());
    
    // With positive elevator deflection, lift should increase and pitch moment should decrease
    EXPECT_GT(forces_with_control.lift, forces_no_control.lift);
    EXPECT_LT(forces_with_control.pitch_moment, forces_no_control.pitch_moment);
    
    // Drag should increase due to control deflection
    EXPECT_GT(forces_with_control.drag, forces_no_control.drag);
}

// Test control authority limits
TEST_F(AerodynamicsTest, ControlAuthorityLimits) {
    // Test maximum deflection
    ControlInputs max_controls;
    max_controls.elevator = ControlInputs::MAX_DEFLECTION;
    max_controls.rudder = ControlInputs::MAX_DEFLECTION;
    max_controls.aileron = ControlInputs::MAX_DEFLECTION;
    
    EXPECT_TRUE(max_controls.isValid());
    
    // Test beyond maximum deflection
    ControlInputs over_limit_controls;
    over_limit_controls.elevator = ControlInputs::MAX_DEFLECTION + 0.1;
    EXPECT_FALSE(over_limit_controls.isValid());
    
    // Test that forces can still be calculated with maximum deflections
    EXPECT_NO_THROW(aero_calc_->calculateForces(test_state_, test_wind_, max_controls));
}

// Test control response linearity
TEST_F(AerodynamicsTest, ControlResponseLinearity) {
    // Set up control effectiveness
    ControlEffectiveness effectiveness;
    effectiveness.elevator_cl_effectiveness = 0.5;
    aero_calc_->setControlEffectiveness(effectiveness);
    
    // Test different elevator deflections
    ControlInputs small_control;
    small_control.elevator = 0.05;
    
    ControlInputs large_control;
    large_control.elevator = 0.10; // Double the deflection
    
    double small_effect = aero_calc_->calculateControlEffectOnCL(small_control);
    double large_effect = aero_calc_->calculateControlEffectOnCL(large_control);
    
    // Effect should be linear (double deflection = double effect)
    EXPECT_NEAR(large_effect, 2.0 * small_effect, 1e-6);
}

// Test multiple control surface interactions
TEST_F(AerodynamicsTest, MultipleControlSurfaces) {
    // Set up control effectiveness
    ControlEffectiveness effectiveness;
    effectiveness.elevator_cl_effectiveness = 0.5;
    effectiveness.aileron_cl_effectiveness = 0.2;
    effectiveness.elevator_cd_effectiveness = 0.1;
    effectiveness.aileron_cd_effectiveness = 0.05;
    aero_calc_->setControlEffectiveness(effectiveness);
    
    // Test combined control inputs
    ControlInputs combined_controls;
    combined_controls.elevator = 0.1;
    combined_controls.aileron = 0.05;
    
    // Calculate individual effects
    ControlInputs elevator_only;
    elevator_only.elevator = 0.1;
    
    ControlInputs aileron_only;
    aileron_only.aileron = 0.05;
    
    double combined_cl = aero_calc_->calculateControlEffectOnCL(combined_controls);
    double elevator_cl = aero_calc_->calculateControlEffectOnCL(elevator_only);
    double aileron_cl = aero_calc_->calculateControlEffectOnCL(aileron_only);
    
    // Combined effect should equal sum of individual effects
    EXPECT_NEAR(combined_cl, elevator_cl + aileron_cl, 1e-6);
}

// Test control effectiveness configuration
TEST_F(AerodynamicsTest, ControlEffectivenessConfiguration) {
    // Test setting valid control effectiveness
    ControlEffectiveness valid_effectiveness;
    EXPECT_NO_THROW(aero_calc_->setControlEffectiveness(valid_effectiveness));
    
    // Test getting control effectiveness
    const ControlEffectiveness& retrieved = aero_calc_->getControlEffectiveness();
    EXPECT_NEAR(retrieved.elevator_cl_effectiveness, valid_effectiveness.elevator_cl_effectiveness, 1e-6);
    
    // Test setting invalid control effectiveness
    ControlEffectiveness invalid_effectiveness;
    invalid_effectiveness.elevator_cl_effectiveness = NAN;
    EXPECT_THROW(aero_calc_->setControlEffectiveness(invalid_effectiveness), std::invalid_argument);
}

} // namespace kite_sim