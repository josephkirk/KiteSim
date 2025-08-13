#include <gtest/gtest.h>
#include "kite_sim/integration_engine.h"
#include "kite_sim/kite_state.h"
#include <cmath>
#include <vector>

namespace kite_sim {

class IntegrationEngineTest : public ::testing::Test {
protected:
    void SetUp() override {
        engine_ = std::make_unique<IntegrationEngine>();
        
        // Set up default parameters
        params_.initial_step_size = 0.01;
        params_.min_step_size = 1e-6;
        params_.max_step_size = 0.1;
        params_.absolute_tolerance = 1e-8;
        params_.relative_tolerance = 1e-8;
        
        engine_->setParameters(params_);
    }
    
    void TearDown() override {
        engine_.reset();
    }
    
    // Simple harmonic oscillator: y'' + ω²y = 0
    // State vector: [y, y']
    // Derivative: [y', -ω²y]
    static void harmonicOscillatorDerivative(double t, const double* state, double* derivative) {
        const double omega = 2.0 * M_PI; // 1 Hz oscillation
        derivative[0] = state[1];         // y' = velocity
        derivative[1] = -omega * omega * state[0]; // y'' = -ω²y
    }
    
    // Exponential decay: y' = -λy
    static void exponentialDecayDerivative(double t, const double* state, double* derivative) {
        const double lambda = 1.0;
        derivative[0] = -lambda * state[0];
    }
    
    // Linear system: y' = Ay where A = [[0, 1], [-1, 0]] (rotation)
    static void rotationDerivative(double t, const double* state, double* derivative) {
        derivative[0] = state[1];   // x' = y
        derivative[1] = -state[0];  // y' = -x
    }
    
    std::unique_ptr<IntegrationEngine> engine_;
    IntegrationParameters params_;
};

// Test parameter validation
TEST_F(IntegrationEngineTest, ParameterValidation) {
    IntegrationParameters valid_params;
    EXPECT_TRUE(valid_params.isValid());
    
    // Test invalid parameters
    IntegrationParameters invalid_params;
    
    invalid_params.initial_step_size = -0.01;
    EXPECT_FALSE(invalid_params.isValid());
    
    invalid_params = IntegrationParameters();
    invalid_params.min_step_size = 0.0;
    EXPECT_FALSE(invalid_params.isValid());
    
    invalid_params = IntegrationParameters();
    invalid_params.max_step_size = invalid_params.min_step_size - 1e-6;
    EXPECT_FALSE(invalid_params.isValid());
    
    invalid_params = IntegrationParameters();
    invalid_params.absolute_tolerance = 0.0;
    EXPECT_FALSE(invalid_params.isValid());
    
    invalid_params = IntegrationParameters();
    invalid_params.safety_factor = 1.5; // Should be < 1.0
    EXPECT_FALSE(invalid_params.isValid());
}

// Test configuration
TEST_F(IntegrationEngineTest, Configuration) {
    EXPECT_FALSE(engine_->isConfigured()); // No derivative function set
    
    engine_->setDerivativeFunction(harmonicOscillatorDerivative);
    EXPECT_TRUE(engine_->isConfigured());
    
    // Test method setting
    engine_->setMethod(IntegrationMethod::EULER);
    EXPECT_EQ(engine_->getMethod(), IntegrationMethod::EULER);
    
    engine_->setMethod(IntegrationMethod::RUNGE_KUTTA_4);
    EXPECT_EQ(engine_->getMethod(), IntegrationMethod::RUNGE_KUTTA_4);
}

// Test Euler integration with harmonic oscillator
TEST_F(IntegrationEngineTest, EulerHarmonicOscillator) {
    engine_->setMethod(IntegrationMethod::EULER);
    engine_->setDerivativeFunction(harmonicOscillatorDerivative);
    
    // Initial conditions: y(0) = 1, y'(0) = 0
    std::vector<double> state = {1.0, 0.0};
    const int state_size = 2;
    
    // Integrate for one period (should return to initial conditions)
    double period = 1.0; // 1 Hz oscillation
    bool success = engine_->integrate(state.data(), state_size, 0.0, period);
    
    EXPECT_TRUE(success);
    
    // Check that we're close to initial conditions (Euler has some error)
    EXPECT_NEAR(state[0], 1.0, 0.1); // Position
    EXPECT_NEAR(state[1], 0.0, 0.1); // Velocity
}

// Test RK4 integration with harmonic oscillator (higher accuracy)
TEST_F(IntegrationEngineTest, RK4HarmonicOscillator) {
    engine_->setMethod(IntegrationMethod::RUNGE_KUTTA_4);
    engine_->setDerivativeFunction(harmonicOscillatorDerivative);
    
    // Reduce step size for better accuracy
    params_.initial_step_size = 0.001;
    engine_->setParameters(params_);
    
    // Initial conditions: y(0) = 1, y'(0) = 0
    std::vector<double> state = {1.0, 0.0};
    const int state_size = 2;
    
    // Integrate for one period
    double period = 1.0;
    bool success = engine_->integrate(state.data(), state_size, 0.0, period);
    
    EXPECT_TRUE(success);
    
    // RK4 should be much more accurate
    EXPECT_NEAR(state[0], 1.0, 1e-3); // Position
    EXPECT_NEAR(state[1], 0.0, 1e-3); // Velocity
}

// Test adaptive RK45 integration
TEST_F(IntegrationEngineTest, AdaptiveRK45HarmonicOscillator) {
    engine_->setMethod(IntegrationMethod::RUNGE_KUTTA_45);
    engine_->setDerivativeFunction(harmonicOscillatorDerivative);
    
    // Initial conditions: y(0) = 1, y'(0) = 0
    std::vector<double> state = {1.0, 0.0};
    const int state_size = 2;
    
    // Integrate for one period
    double period = 1.0;
    bool success = engine_->integrate(state.data(), state_size, 0.0, period);
    
    EXPECT_TRUE(success);
    
    // Adaptive method should be very accurate
    EXPECT_NEAR(state[0], 1.0, 1e-6); // Position
    EXPECT_NEAR(state[1], 0.0, 1e-6); // Velocity
    
    // Check statistics
    const auto& stats = engine_->getStats();
    EXPECT_GT(stats.accepted_steps, 0);
    EXPECT_GE(stats.getStepAcceptanceRate(), 0.5); // Should have reasonable acceptance rate
}

// Test Dormand-Prince integration
TEST_F(IntegrationEngineTest, DormandPrince45HarmonicOscillator) {
    engine_->setMethod(IntegrationMethod::DORMAND_PRINCE_45);
    engine_->setDerivativeFunction(harmonicOscillatorDerivative);
    
    // Initial conditions: y(0) = 1, y'(0) = 0
    std::vector<double> state = {1.0, 0.0};
    const int state_size = 2;
    
    // Integrate for one period
    double period = 1.0;
    bool success = engine_->integrate(state.data(), state_size, 0.0, period);
    
    EXPECT_TRUE(success);
    
    // Dormand-Prince should be very accurate
    EXPECT_NEAR(state[0], 1.0, 1e-6); // Position
    EXPECT_NEAR(state[1], 0.0, 1e-6); // Velocity
}

// Test exponential decay (analytical solution available)
TEST_F(IntegrationEngineTest, ExponentialDecay) {
    engine_->setMethod(IntegrationMethod::RUNGE_KUTTA_45);
    engine_->setDerivativeFunction(exponentialDecayDerivative);
    
    // Initial condition: y(0) = 1
    std::vector<double> state = {1.0};
    const int state_size = 1;
    
    // Integrate to t = 1
    double final_time = 1.0;
    bool success = engine_->integrate(state.data(), state_size, 0.0, final_time);
    
    EXPECT_TRUE(success);
    
    // Analytical solution: y(t) = e^(-t)
    double expected = std::exp(-final_time);
    EXPECT_NEAR(state[0], expected, 1e-8);
}

// Test rotation (conservation of energy)
TEST_F(IntegrationEngineTest, RotationEnergyConservation) {
    engine_->setMethod(IntegrationMethod::RUNGE_KUTTA_45);
    engine_->setDerivativeFunction(rotationDerivative);
    
    // Initial conditions: x(0) = 1, y(0) = 0
    std::vector<double> state = {1.0, 0.0};
    const int state_size = 2;
    
    // Calculate initial energy
    double initial_energy = 0.5 * (state[0] * state[0] + state[1] * state[1]);
    
    // Integrate for one full rotation (2π time units)
    double final_time = 2.0 * M_PI;
    bool success = engine_->integrate(state.data(), state_size, 0.0, final_time);
    
    EXPECT_TRUE(success);
    
    // Should return to initial position
    EXPECT_NEAR(state[0], 1.0, 1e-6);
    EXPECT_NEAR(state[1], 0.0, 1e-6);
    
    // Energy should be conserved
    double final_energy = 0.5 * (state[0] * state[0] + state[1] * state[1]);
    EXPECT_NEAR(final_energy, initial_energy, 1e-8);
}

// Test step size control
TEST_F(IntegrationEngineTest, StepSizeControl) {
    engine_->setMethod(IntegrationMethod::RUNGE_KUTTA_45);
    engine_->setDerivativeFunction(harmonicOscillatorDerivative);
    
    // Set tight tolerances to force small steps
    params_.absolute_tolerance = 1e-10;
    params_.relative_tolerance = 1e-10;
    params_.max_step_size = 0.01;
    engine_->setParameters(params_);
    
    std::vector<double> state = {1.0, 0.0};
    const int state_size = 2;
    
    bool success = engine_->integrate(state.data(), state_size, 0.0, 0.1);
    EXPECT_TRUE(success);
    
    const auto& stats = engine_->getStats();
    EXPECT_GT(stats.total_steps, 10); // Should take many small steps
    EXPECT_LE(stats.max_step_size, params_.max_step_size);
    EXPECT_GE(stats.min_step_size, params_.min_step_size);
}

// Test error estimation
TEST_F(IntegrationEngineTest, ErrorEstimation) {
    engine_->setMethod(IntegrationMethod::RUNGE_KUTTA_45);
    engine_->setDerivativeFunction(harmonicOscillatorDerivative);
    
    std::vector<double> state1 = {1.0, 0.0};
    std::vector<double> state2 = {1.001, 0.001};
    
    double error = engine_->estimateError(state1.data(), state2.data(), 2);
    EXPECT_GT(error, 0.0);
    EXPECT_LT(error, 1.0); // Should be reasonable magnitude
}

// Test KiteState integration wrapper
TEST_F(IntegrationEngineTest, KiteStateIntegration) {
    // Create a simple derivative function for KiteState
    engine_->setDerivativeFunction([](double t, const double* state, double* derivative) {
        // Simple constant velocity motion
        // Position derivative = velocity
        derivative[0] = state[3]; // x' = vx
        derivative[1] = state[4]; // y' = vy
        derivative[2] = state[5]; // z' = vz
        
        // Velocity derivative = 0 (constant velocity)
        derivative[3] = 0.0; // vx' = 0
        derivative[4] = 0.0; // vy' = 0
        derivative[5] = 0.0; // vz' = 0
        
        // Quaternion derivative (no rotation)
        derivative[6] = 0.0; // qw' = 0
        derivative[7] = 0.0; // qx' = 0
        derivative[8] = 0.0; // qy' = 0
        derivative[9] = 0.0; // qz' = 0
        
        // Angular velocity derivative = 0
        derivative[10] = 0.0; // wx' = 0
        derivative[11] = 0.0; // wy' = 0
        derivative[12] = 0.0; // wz' = 0
    });
    
    // Initial state: position at origin, velocity = (1, 0, 0)
    KiteState state(Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0),
                   Quaternion::identity(), Vector3(0.0, 0.0, 0.0), 0.0);
    
    // Integrate for 1 second
    bool success = engine_->integrate(state, 1.0);
    EXPECT_TRUE(success);
    
    // Should have moved 1 unit in x direction
    EXPECT_NEAR(state.position().x(), 1.0, 1e-6);
    EXPECT_NEAR(state.position().y(), 0.0, 1e-6);
    EXPECT_NEAR(state.position().z(), 0.0, 1e-6);
    EXPECT_NEAR(state.time(), 1.0, 1e-6);
}

// Test integration statistics
TEST_F(IntegrationEngineTest, IntegrationStatistics) {
    engine_->setMethod(IntegrationMethod::RUNGE_KUTTA_45);
    engine_->setDerivativeFunction(harmonicOscillatorDerivative);
    
    // Reset statistics
    engine_->resetStats();
    
    std::vector<double> state = {1.0, 0.0};
    const int state_size = 2;
    
    bool success = engine_->integrate(state.data(), state_size, 0.0, 0.1);
    EXPECT_TRUE(success);
    
    const auto& stats = engine_->getStats();
    EXPECT_GT(stats.total_steps, 0);
    EXPECT_GT(stats.accepted_steps, 0);
    EXPECT_GE(stats.rejected_steps, 0);
    EXPECT_GT(stats.avg_step_size, 0.0);
    EXPECT_GE(stats.getStepAcceptanceRate(), 0.0);
    EXPECT_LE(stats.getStepAcceptanceRate(), 1.0);
}

// Test stability with stiff system
TEST_F(IntegrationEngineTest, StiffSystemStability) {
    // Stiff system: y' = -1000*y + 1000*cos(t)
    // Analytical solution: y(t) = cos(t) + sin(t) + C*exp(-1000*t)
    engine_->setDerivativeFunction([](double t, const double* state, double* derivative) {
        derivative[0] = -1000.0 * state[0] + 1000.0 * std::cos(t);
    });
    
    engine_->setMethod(IntegrationMethod::DORMAND_PRINCE_45);
    
    // Set small tolerances for stiff system
    params_.absolute_tolerance = 1e-10;
    params_.relative_tolerance = 1e-10;
    params_.min_step_size = 1e-8;
    engine_->setParameters(params_);
    
    std::vector<double> state = {1.0}; // Initial condition
    const int state_size = 1;
    
    // Integrate for short time (stiff systems are challenging)
    bool success = engine_->integrate(state.data(), state_size, 0.0, 0.01);
    EXPECT_TRUE(success);
    
    // Should remain finite and reasonable
    EXPECT_TRUE(std::isfinite(state[0]));
    EXPECT_LT(std::abs(state[0]), 10.0); // Should not blow up
}

// Test invalid inputs
TEST_F(IntegrationEngineTest, InvalidInputs) {
    engine_->setDerivativeFunction(harmonicOscillatorDerivative);
    
    std::vector<double> state = {1.0, 0.0};
    
    // Test null state vector
    EXPECT_THROW(engine_->integrate(nullptr, 2, 0.0, 1.0), std::invalid_argument);
    
    // Test invalid state size
    EXPECT_THROW(engine_->integrate(state.data(), 0, 0.0, 1.0), std::invalid_argument);
    EXPECT_THROW(engine_->integrate(state.data(), -1, 0.0, 1.0), std::invalid_argument);
    
    // Test invalid time range
    bool success = engine_->integrate(state.data(), 2, 1.0, 0.0); // target < current
    EXPECT_TRUE(success); // Should return immediately without error
    
    // Test with NaN in state
    state[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(engine_->integrate(state.data(), 2, 0.0, 1.0), std::invalid_argument);
}

// Test reset functionality
TEST_F(IntegrationEngineTest, Reset) {
    engine_->setDerivativeFunction(harmonicOscillatorDerivative);
    
    std::vector<double> state = {1.0, 0.0};
    engine_->integrate(state.data(), 2, 0.0, 0.1);
    
    // Should have some statistics
    EXPECT_GT(engine_->getStats().total_steps, 0);
    
    // Reset should clear statistics
    engine_->reset();
    EXPECT_EQ(engine_->getStats().total_steps, 0);
}

} // namespace kite_sim