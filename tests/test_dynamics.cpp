#include <gtest/gtest.h>
#include "kite_sim/dynamics.h"
#include "kite_sim/kite_geometry.h"
#include <cmath>

namespace kite_sim {

class DynamicsEngineTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up a simple test mass properties
        mass_props_.mass = 10.0; // 10 kg
        mass_props_.center_of_gravity = Vector3(0.0, 0.0, 0.0);
        
        // Simple diagonal inertia tensor
        double data[9] = {
            2.0, 0.0, 0.0,  // Ixx = 2.0 kg⋅m²
            0.0, 3.0, 0.0,  // Iyy = 3.0 kg⋅m²
            0.0, 0.0, 4.0   // Izz = 4.0 kg⋅m²
        };
        mass_props_.inertia_tensor = Matrix3x3(data);
        
        // Configure dynamics engine
        dynamics_engine_.setMassProperties(mass_props_);
        
        // Set up initial state
        initial_state_ = KiteState(
            Vector3(0.0, 0.0, 100.0),    // Position: 100m altitude
            Vector3(10.0, 0.0, 0.0),     // Velocity: 10 m/s in x direction
            Quaternion::identity(),       // No rotation
            Vector3(0.0, 0.0, 0.0),      // No angular velocity
            0.0                          // Time = 0
        );
    }
    
    MassProperties mass_props_;
    DynamicsEngine dynamics_engine_;
    KiteState initial_state_;
    
    // Helper function to create test forces
    Forces createTestForces(const Vector3& force, const Vector3& moment = Vector3(0.0, 0.0, 0.0)) {
        return Forces(force, moment);
    }
    
    // Helper function to check if two doubles are approximately equal
    bool isApproxEqual(double a, double b, double tolerance = 1e-9) {
        return std::abs(a - b) < tolerance;
    }
    
    // Helper function to check if two Vector3s are approximately equal
    bool isApproxEqual(const Vector3& a, const Vector3& b, double tolerance = 1e-9) {
        return isApproxEqual(a.x(), b.x(), tolerance) &&
               isApproxEqual(a.y(), b.y(), tolerance) &&
               isApproxEqual(a.z(), b.z(), tolerance);
    }
};

// Test basic construction and configuration
TEST_F(DynamicsEngineTest, Construction) {
    DynamicsEngine engine;
    EXPECT_FALSE(engine.isConfigured());
    
    engine.setMassProperties(mass_props_);
    EXPECT_TRUE(engine.isConfigured());
}

TEST_F(DynamicsEngineTest, IntegrationMethodConfiguration) {
    DynamicsEngine engine(IntegrationMethod::EULER);
    EXPECT_EQ(engine.getIntegrationMethod(), IntegrationMethod::EULER);
    
    engine.setIntegrationMethod(IntegrationMethod::RUNGE_KUTTA_4);
    EXPECT_EQ(engine.getIntegrationMethod(), IntegrationMethod::RUNGE_KUTTA_4);
}

// Test translational acceleration calculation
TEST_F(DynamicsEngineTest, TranslationalAcceleration) {
    Vector3 force(100.0, 0.0, 0.0); // 100 N in x direction
    Vector3 expected_acceleration = force / mass_props_.mass; // a = F/m
    
    Vector3 actual_acceleration = dynamics_engine_.calculateTranslationalAcceleration(force);
    
    EXPECT_TRUE(isApproxEqual(actual_acceleration, expected_acceleration));
}

// Test rotational acceleration calculation
TEST_F(DynamicsEngineTest, RotationalAccelerationNoAngularVelocity) {
    Vector3 moment(10.0, 0.0, 0.0); // 10 N⋅m about x-axis
    Vector3 angular_velocity(0.0, 0.0, 0.0); // No initial angular velocity
    
    // With no angular velocity, ω × (I*ω) = 0, so α = I^(-1) * M
    Vector3 expected_acceleration(10.0 / 2.0, 0.0, 0.0); // α_x = M_x / I_xx
    
    Vector3 actual_acceleration = dynamics_engine_.calculateRotationalAcceleration(moment, angular_velocity);
    
    EXPECT_TRUE(isApproxEqual(actual_acceleration, expected_acceleration));
}

TEST_F(DynamicsEngineTest, RotationalAccelerationWithAngularVelocity) {
    Vector3 moment(0.0, 0.0, 0.0); // No external moment
    Vector3 angular_velocity(1.0, 0.0, 0.0); // 1 rad/s about x-axis
    
    // With angular velocity but no external moment, we should get gyroscopic effects
    // M = I*α + ω × (I*ω), so α = I^(-1) * (M - ω × (I*ω))
    Vector3 I_omega(2.0 * 1.0, 0.0, 0.0); // I * ω
    Vector3 omega_cross_I_omega = angular_velocity.cross(I_omega); // Should be (0, 0, 0) for this case
    Vector3 expected_acceleration(0.0, 0.0, 0.0);
    
    Vector3 actual_acceleration = dynamics_engine_.calculateRotationalAcceleration(moment, angular_velocity);
    
    EXPECT_TRUE(isApproxEqual(actual_acceleration, expected_acceleration));
}

// Test attitude derivative calculation
TEST_F(DynamicsEngineTest, AttitudeDerivative) {
    Quaternion attitude = Quaternion::identity();
    Vector3 angular_velocity(0.1, 0.2, 0.3); // rad/s
    
    Quaternion attitude_dot = dynamics_engine_.calculateAttitudeDerivative(attitude, angular_velocity);
    
    // For identity quaternion and given angular velocity, the derivative should be:
    // q̇ = 0.5 * q * [0, ωx, ωy, ωz] = 0.5 * [1, 0, 0, 0] * [0, 0.1, 0.2, 0.3]
    // = 0.5 * [0, 0.1, 0.2, 0.3] = [0, 0.05, 0.1, 0.15]
    EXPECT_TRUE(isApproxEqual(attitude_dot.w(), 0.0));
    EXPECT_TRUE(isApproxEqual(attitude_dot.x(), 0.05));
    EXPECT_TRUE(isApproxEqual(attitude_dot.y(), 0.1));
    EXPECT_TRUE(isApproxEqual(attitude_dot.z(), 0.15));
}

// Test coordinate transformations
TEST_F(DynamicsEngineTest, CoordinateTransformations) {
    Vector3 world_vector(1.0, 0.0, 0.0);
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3(0.0, 0.0, 1.0), M_PI / 2.0); // 90° rotation about z
    
    Vector3 body_vector = dynamics_engine_.worldToBody(world_vector, attitude);
    Vector3 back_to_world = dynamics_engine_.bodyToWorld(body_vector, attitude);
    
    // Should get back the original vector
    EXPECT_TRUE(isApproxEqual(world_vector, back_to_world, 1e-6));
}

// Test simple Euler integration
TEST_F(DynamicsEngineTest, EulerIntegrationConstantForce) {
    dynamics_engine_.setIntegrationMethod(IntegrationMethod::EULER);
    
    // Apply constant force in x direction
    Forces forces = createTestForces(Vector3(100.0, 0.0, 0.0)); // 100 N
    double dt = 0.01; // 10 ms time step
    
    KiteState new_state = dynamics_engine_.integrate(initial_state_, forces, dt);
    
    // Expected acceleration: a = F/m = 100/10 = 10 m/s²
    // Expected velocity: v = v0 + a*dt = 10 + 10*0.01 = 10.1 m/s
    // Expected position: x = x0 + v0*dt = 0 + 10*0.01 = 0.1 m
    
    EXPECT_TRUE(isApproxEqual(new_state.velocity().x(), 10.1));
    EXPECT_TRUE(isApproxEqual(new_state.position().x(), 0.1));
    EXPECT_TRUE(isApproxEqual(new_state.time(), dt));
}

// Test Runge-Kutta 4 integration
TEST_F(DynamicsEngineTest, RungeKutta4IntegrationConstantForce) {
    dynamics_engine_.setIntegrationMethod(IntegrationMethod::RUNGE_KUTTA_4);
    
    // Apply constant force in x direction
    Forces forces = createTestForces(Vector3(100.0, 0.0, 0.0)); // 100 N
    double dt = 0.01; // 10 ms time step
    
    KiteState new_state = dynamics_engine_.integrate(initial_state_, forces, dt);
    
    // For constant acceleration, RK4 should give the same result as Euler for this simple case
    // Expected acceleration: a = F/m = 100/10 = 10 m/s²
    // Expected velocity: v = v0 + a*dt = 10 + 10*0.01 = 10.1 m/s
    // Expected position: x = x0 + v0*dt + 0.5*a*dt² = 0 + 10*0.01 + 0.5*10*0.01² = 0.1005 m
    
    EXPECT_TRUE(isApproxEqual(new_state.velocity().x(), 10.1));
    EXPECT_TRUE(isApproxEqual(new_state.position().x(), 0.1005, 1e-6));
    EXPECT_TRUE(isApproxEqual(new_state.time(), dt));
}

// Test free fall under gravity (analytical solution)
TEST_F(DynamicsEngineTest, FreeFallAnalyticalComparison) {
    dynamics_engine_.setIntegrationMethod(IntegrationMethod::RUNGE_KUTTA_4);
    
    // Start from rest at height 100m
    KiteState state(
        Vector3(0.0, 0.0, 100.0),    // Position
        Vector3(0.0, 0.0, 0.0),      // Velocity (at rest)
        Quaternion::identity(),       // No rotation
        Vector3(0.0, 0.0, 0.0),      // No angular velocity
        0.0                          // Time = 0
    );
    
    // Apply gravitational force (downward)
    double g = 9.81; // m/s²
    Forces gravity = createTestForces(Vector3(0.0, 0.0, -mass_props_.mass * g));
    
    double dt = 0.001; // 1 ms time step
    double total_time = 1.0; // 1 second
    int num_steps = static_cast<int>(total_time / dt);
    
    // Integrate for 1 second
    for (int i = 0; i < num_steps; ++i) {
        state = dynamics_engine_.integrate(state, gravity, dt);
    }
    
    // Analytical solution for free fall:
    // z(t) = z0 + v0*t - 0.5*g*t²
    // v_z(t) = v0 - g*t
    double expected_z = 100.0 - 0.5 * g * total_time * total_time;
    double expected_vz = -g * total_time;
    
    EXPECT_TRUE(isApproxEqual(state.position().z(), expected_z, 1e-3));
    EXPECT_TRUE(isApproxEqual(state.velocity().z(), expected_vz, 1e-3));
}

// Test rotational motion under constant torque
TEST_F(DynamicsEngineTest, ConstantTorqueRotation) {
    dynamics_engine_.setIntegrationMethod(IntegrationMethod::RUNGE_KUTTA_4);
    
    // Start from rest
    KiteState state(
        Vector3(0.0, 0.0, 100.0),    // Position
        Vector3(0.0, 0.0, 0.0),      // Velocity
        Quaternion::identity(),       // No rotation
        Vector3(0.0, 0.0, 0.0),      // No angular velocity
        0.0                          // Time = 0
    );
    
    // Apply constant torque about x-axis
    double torque = 20.0; // N⋅m
    Forces forces = createTestForces(Vector3(0.0, 0.0, 0.0), Vector3(torque, 0.0, 0.0));
    
    double dt = 0.001; // 1 ms time step
    double total_time = 0.1; // 0.1 second
    int num_steps = static_cast<int>(total_time / dt);
    
    // Integrate
    for (int i = 0; i < num_steps; ++i) {
        state = dynamics_engine_.integrate(state, forces, dt);
    }
    
    // Analytical solution for constant torque:
    // α = M/I = 20/2 = 10 rad/s²
    // ω(t) = α*t = 10*0.1 = 1.0 rad/s
    double expected_angular_velocity = torque / mass_props_.inertia_tensor(0, 0) * total_time;
    
    EXPECT_TRUE(isApproxEqual(state.angularVelocity().x(), expected_angular_velocity, 1e-3));
}

// Test energy conservation for conservative system
TEST_F(DynamicsEngineTest, EnergyConservationOscillator) {
    dynamics_engine_.setIntegrationMethod(IntegrationMethod::RUNGE_KUTTA_4);
    
    // Simple harmonic oscillator: F = -k*x
    double k = 100.0; // Spring constant
    double dt = 0.001; // Small time step for accuracy
    
    // Start with some initial displacement and velocity
    KiteState state(
        Vector3(1.0, 0.0, 100.0),    // 1m displacement in x
        Vector3(0.0, 0.0, 0.0),      // No initial velocity
        Quaternion::identity(),       // No rotation
        Vector3(0.0, 0.0, 0.0),      // No angular velocity
        0.0                          // Time = 0
    );
    
    // Calculate initial energy
    double initial_kinetic = 0.5 * mass_props_.mass * state.velocity().magnitudeSquared();
    double initial_potential = 0.5 * k * state.position().x() * state.position().x();
    double initial_energy = initial_kinetic + initial_potential;
    
    // Integrate for one period
    double omega = sqrt(k / mass_props_.mass);
    double period = 2.0 * M_PI / omega;
    int num_steps = static_cast<int>(period / dt);
    
    for (int i = 0; i < num_steps; ++i) {
        // Apply spring force: F = -k*x
        Forces spring_force = createTestForces(Vector3(-k * state.position().x(), 0.0, 0.0));
        state = dynamics_engine_.integrate(state, spring_force, dt);
    }
    
    // Calculate final energy
    double final_kinetic = 0.5 * mass_props_.mass * state.velocity().magnitudeSquared();
    double final_potential = 0.5 * k * state.position().x() * state.position().x();
    double final_energy = final_kinetic + final_potential;
    
    // Energy should be conserved (within numerical precision)
    EXPECT_TRUE(isApproxEqual(initial_energy, final_energy, 1e-2));
}

// Test quaternion normalization
TEST_F(DynamicsEngineTest, QuaternionNormalization) {
    dynamics_engine_.setIntegrationMethod(IntegrationMethod::RUNGE_KUTTA_4);
    
    // Start with a rotation and angular velocity
    KiteState state(
        Vector3(0.0, 0.0, 100.0),
        Vector3(0.0, 0.0, 0.0),
        Quaternion::fromAxisAngle(Vector3(0.0, 0.0, 1.0), 0.1), // Small rotation
        Vector3(0.0, 0.0, 1.0), // 1 rad/s about z-axis
        0.0
    );
    
    double dt = 0.01;
    int num_steps = 1000; // Long integration to test normalization
    
    for (int i = 0; i < num_steps; ++i) {
        Forces no_forces = createTestForces(Vector3(0.0, 0.0, 0.0));
        state = dynamics_engine_.integrate(state, no_forces, dt);
        
        // Quaternion should remain normalized
        EXPECT_TRUE(state.attitude().isUnit(1e-6));
    }
}

// Test validation functions
TEST_F(DynamicsEngineTest, StateValidation) {
    // Valid state
    EXPECT_TRUE(dynamics_engine_.validateState(initial_state_));
    
    // Invalid state (NaN position)
    KiteState invalid_state = initial_state_;
    invalid_state.setPosition(Vector3(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    EXPECT_FALSE(dynamics_engine_.validateState(invalid_state));
}

TEST_F(DynamicsEngineTest, ForcesValidation) {
    // Valid forces
    Forces valid_forces = createTestForces(Vector3(100.0, 0.0, 0.0));
    EXPECT_TRUE(dynamics_engine_.validateForces(valid_forces));
    
    // Invalid forces (infinite force)
    Forces invalid_forces = createTestForces(Vector3(std::numeric_limits<double>::infinity(), 0.0, 0.0));
    EXPECT_FALSE(dynamics_engine_.validateForces(invalid_forces));
}

// Test error handling
TEST_F(DynamicsEngineTest, ErrorHandling) {
    DynamicsEngine unconfigured_engine;
    
    // Should throw when not configured
    EXPECT_THROW(unconfigured_engine.integrate(initial_state_, createTestForces(Vector3(0.0, 0.0, 0.0)), 0.01),
                 std::runtime_error);
    
    // Should throw with invalid time step
    EXPECT_THROW(dynamics_engine_.integrate(initial_state_, createTestForces(Vector3(0.0, 0.0, 0.0)), -0.01),
                 std::invalid_argument);
    
    // Should throw with invalid mass properties
    MassProperties invalid_mass;
    invalid_mass.mass = -1.0; // Negative mass
    EXPECT_THROW(dynamics_engine_.setMassProperties(invalid_mass), std::invalid_argument);
}

} // namespace kite_sim