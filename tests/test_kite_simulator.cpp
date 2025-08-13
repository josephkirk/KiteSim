#include <gtest/gtest.h>
#include "kite_sim/kite_simulator.h"
#include "kite_sim/kite_geometry.h"
#include "kite_sim/tether.h"
#include "kite_sim/wind_model.h"
#include <cmath>

using namespace kite_sim;

class KiteSimulatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create basic test configuration
        geometry_ = KiteGeometry::createTestKite();
        tether_ = TetherProperties::createDefaultTether();
        
        // Set up simulation config
        config_.time_step = 0.01;
        config_.simulation_duration = 1.0;  // Short test duration
        config_.enable_data_logging = true;
        config_.logging_interval = 0.1;
        
        // Create initial state
        initial_state_.setPosition(Vector3(0.0, 0.0, 50.0));  // 50m altitude
        initial_state_.setVelocity(Vector3(5.0, 0.0, 0.0));   // 5 m/s forward
        initial_state_.setAttitude(Quaternion(1.0, 0.0, 0.0, 0.0));  // Level
        initial_state_.setAngularVelocity(Vector3(0.0, 0.0, 0.0));   // No rotation
        initial_state_.setTime(0.0);
    }
    
    KiteGeometry geometry_;
    TetherProperties tether_;
    SimulationConfig config_;
    KiteState initial_state_;
};

TEST_F(KiteSimulatorTest, ConstructorAndBasicSetup) {
    KiteSimulator simulator;
    
    // Should be able to construct without errors
    EXPECT_FALSE(simulator.isConfigured());  // Not configured yet
    EXPECT_FALSE(simulator.isRunning());
    
    // Configure the simulator
    simulator.setConfig(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    // Should be configured now
    EXPECT_TRUE(simulator.isConfigured());
    
    // Check configuration access
    EXPECT_EQ(simulator.getConfig().time_step, config_.time_step);
    EXPECT_EQ(simulator.getInitialState().position().z(), 50.0);
}

TEST_F(KiteSimulatorTest, ConfigurationValidation) {
    KiteSimulator simulator;
    
    // Should have configuration errors initially
    auto errors = simulator.getConfigurationErrors();
    EXPECT_FALSE(errors.empty());
    
    // Configure step by step and check errors reduce
    simulator.setConfig(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    errors = simulator.getConfigurationErrors();
    EXPECT_TRUE(errors.empty()) << "Configuration errors: " << errors.size();
}

TEST_F(KiteSimulatorTest, BasicSimulationRun) {
    KiteSimulator simulator(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    // Run simulation
    auto results = simulator.run();
    
    // Check results
    EXPECT_TRUE(results.success) << "Simulation failed: " << results.error_message;
    EXPECT_GT(results.actual_duration, 0.0);
    EXPECT_NEAR(results.final_time, config_.simulation_duration, 0.01);
    EXPECT_GT(results.total_steps, 0);
    
    // Check final state is valid
    const auto& final_state = simulator.getCurrentState();
    EXPECT_TRUE(final_state.isValid());
    EXPECT_NEAR(final_state.time(), config_.simulation_duration, 0.01);
}

TEST_F(KiteSimulatorTest, SimulationWithDifferentWindConditions) {
    KiteSimulator simulator(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    // Test with different wind speeds
    std::vector<double> wind_speeds = {5.0, 10.0, 15.0};
    
    for (double wind_speed : wind_speeds) {
        auto wind_model = std::make_unique<UniformWindModel>(wind_speed, 0.0);
        simulator.setWindModel(std::move(wind_model));
        
        simulator.reset();
        auto results = simulator.run();
        
        EXPECT_TRUE(results.success) << "Simulation failed with wind speed " << wind_speed 
                                    << ": " << results.error_message;
        
        // Check that kite responds to wind (should have some velocity)
        const auto& final_state = simulator.getCurrentState();
        double final_speed = final_state.velocity().magnitude();
        EXPECT_GT(final_speed, 0.1) << "Kite should have some velocity with wind speed " << wind_speed;
    }
}

TEST_F(KiteSimulatorTest, StepByStepSimulation) {
    KiteSimulator simulator(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    double target_time = 0.0;
    double step_size = 0.01;
    int num_steps = 10;
    
    for (int i = 0; i < num_steps; ++i) {
        target_time += step_size;
        bool success = simulator.step(target_time);
        
        EXPECT_TRUE(success) << "Step " << i << " failed";
        
        const auto& state = simulator.getCurrentState();
        EXPECT_TRUE(state.isValid()) << "Invalid state at step " << i;
        EXPECT_NEAR(state.time(), target_time, 1e-6) << "Time mismatch at step " << i;
    }
}

TEST_F(KiteSimulatorTest, CallbackFunctionality) {
    KiteSimulator simulator(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    // Set up callback to collect data
    std::vector<double> callback_times;
    std::vector<KiteState> callback_states;
    
    auto callback = [&](double time, const KiteState& state, const Forces& total_forces,
                       const AeroForces& aero_forces, const TetherForces& tether_forces) -> bool {
        callback_times.push_back(time);
        callback_states.push_back(state);
        return true;  // Continue simulation
    };
    
    // Run with callback
    auto results = simulator.runWithCallback(callback);
    
    EXPECT_TRUE(results.success);
    EXPECT_FALSE(callback_times.empty());
    EXPECT_EQ(callback_times.size(), callback_states.size());
    
    // Check that callback was called with increasing times
    for (size_t i = 1; i < callback_times.size(); ++i) {
        EXPECT_GT(callback_times[i], callback_times[i-1]);
    }
}

TEST_F(KiteSimulatorTest, EarlyTerminationViaCallback) {
    KiteSimulator simulator(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    double termination_time = 0.5;  // Terminate at 0.5 seconds
    
    auto callback = [termination_time](double time, const KiteState& state, const Forces& total_forces,
                                      const AeroForces& aero_forces, const TetherForces& tether_forces) -> bool {
        return time < termination_time;  // Stop when time reaches termination_time
    };
    
    auto results = simulator.runWithCallback(callback);
    
    EXPECT_TRUE(results.success);
    EXPECT_LT(results.final_time, config_.simulation_duration);
    EXPECT_NEAR(results.final_time, termination_time, 0.1);  // Should stop around termination time
}

TEST_F(KiteSimulatorTest, ForceCalculations) {
    KiteSimulator simulator(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    // Run a single step to calculate forces
    simulator.step(0.01);
    
    // Check that forces are calculated
    const auto& total_forces = simulator.getTotalForces();
    const auto& aero_forces = simulator.getAeroForces();
    const auto& tether_forces = simulator.getTetherForces();
    
    EXPECT_TRUE(total_forces.isValid());
    EXPECT_TRUE(aero_forces.isValid());
    EXPECT_TRUE(tether_forces.isValid());
    
    // Forces should be non-zero (gravity, aerodynamics, tether tension)
    EXPECT_GT(total_forces.force.magnitude(), 0.1);
    
    // Tether should provide upward force (positive Z component)
    EXPECT_GT(tether_forces.force.z(), 0.0);
}

TEST_F(KiteSimulatorTest, EnergyConservationCheck) {
    // Use longer simulation for energy analysis
    config_.simulation_duration = 2.0;
    config_.time_step = 0.001;  // Smaller time step for accuracy
    
    KiteSimulator simulator(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    // Set up callback to track energy
    std::vector<double> kinetic_energies;
    std::vector<double> potential_energies;
    
    double mass = geometry_.massProperties().mass;
    
    auto callback = [&](double time, const KiteState& state, const Forces& total_forces,
                       const AeroForces& aero_forces, const TetherForces& tether_forces) -> bool {
        // Calculate kinetic energy
        double v_squared = state.velocity().dot(state.velocity());
        double omega_squared = state.angularVelocity().dot(state.angularVelocity());
        double ke = 0.5 * mass * v_squared + 0.5 * omega_squared;  // Simplified rotational KE
        
        // Calculate potential energy (gravitational)
        double pe = mass * 9.81 * state.position().z();
        
        kinetic_energies.push_back(ke);
        potential_energies.push_back(pe);
        
        return true;
    };
    
    auto results = simulator.runWithCallback(callback);
    
    EXPECT_TRUE(results.success);
    EXPECT_FALSE(kinetic_energies.empty());
    
    // Energy should remain bounded (not grow indefinitely)
    double initial_total_energy = kinetic_energies[0] + potential_energies[0];
    double final_total_energy = kinetic_energies.back() + potential_energies.back();
    
    // Allow for some energy change due to aerodynamic forces and numerical errors
    double energy_change_ratio = std::abs(final_total_energy - initial_total_energy) / initial_total_energy;
    EXPECT_LT(energy_change_ratio, 0.5) << "Energy change too large: " << energy_change_ratio;
}

TEST_F(KiteSimulatorTest, FactoryMethods) {
    // Test basic simulator factory
    auto simulator = KiteSimulator::createBasicSimulator(geometry_, tether_, 10.0, 0.0);
    
    EXPECT_TRUE(simulator.isConfigured());
    
    auto results = simulator.run();
    EXPECT_TRUE(results.success) << "Factory-created simulator failed: " << results.error_message;
}

TEST_F(KiteSimulatorTest, InvalidConfigurationHandling) {
    KiteSimulator simulator;
    
    // Try to run without proper configuration
    auto results = simulator.run();
    
    EXPECT_FALSE(results.success);
    EXPECT_FALSE(results.error_message.empty());
}

TEST_F(KiteSimulatorTest, ResetFunctionality) {
    KiteSimulator simulator(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    // Run simulation partway
    simulator.step(0.5);
    
    auto state_before_reset = simulator.getCurrentState();
    EXPECT_GT(state_before_reset.time(), 0.0);
    
    // Reset and check
    simulator.reset();
    
    auto state_after_reset = simulator.getCurrentState();
    EXPECT_EQ(state_after_reset.time(), initial_state_.time());
    EXPECT_FALSE(simulator.isRunning());
}

TEST_F(KiteSimulatorTest, DataLoggingAndOutput) {
    config_.enable_data_logging = true;
    config_.logging_interval = 0.05;
    config_.output_file = "test_output.csv";
    
    KiteSimulator simulator(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    auto results = simulator.run();
    
    EXPECT_TRUE(results.success);
    
    // Check if output file was created (basic check)
    std::ifstream file(config_.output_file);
    EXPECT_TRUE(file.good()) << "Output file should be created";
    file.close();
    
    // Clean up test file
    std::remove(config_.output_file.c_str());
}

// Performance test - should complete within reasonable time
TEST_F(KiteSimulatorTest, PerformanceTest) {
    config_.simulation_duration = 10.0;  // Longer simulation
    config_.time_step = 0.01;
    
    KiteSimulator simulator(config_);
    simulator.setKiteGeometry(geometry_);
    simulator.setTetherProperties(tether_);
    simulator.setInitialState(initial_state_);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto results = simulator.run();
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    EXPECT_TRUE(results.success);
    EXPECT_LT(duration.count(), 5000) << "Simulation took too long: " << duration.count() << " ms";
    
    // Check that simulation ran faster than real-time
    double real_time_ratio = static_cast<double>(duration.count()) / 1000.0 / config_.simulation_duration;
    EXPECT_LT(real_time_ratio, 1.0) << "Simulation should run faster than real-time";
}