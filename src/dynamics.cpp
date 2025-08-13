#include "kite_sim/dynamics.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace kite_sim {

// Forces implementation
bool Forces::isValid() const {
    return std::isfinite(force.x()) && std::isfinite(force.y()) && std::isfinite(force.z()) &&
           std::isfinite(moment.x()) && std::isfinite(moment.y()) && std::isfinite(moment.z());
}

// StateDerivative implementation
void StateDerivative::toArray(double* array) const {
    if (!array) return;
    
    // Position derivative (velocity)
    array[0] = position_dot.x();
    array[1] = position_dot.y();
    array[2] = position_dot.z();
    
    // Velocity derivative (acceleration)
    array[3] = velocity_dot.x();
    array[4] = velocity_dot.y();
    array[5] = velocity_dot.z();
    
    // Attitude derivative (quaternion derivative)
    array[6] = attitude_dot.w();
    array[7] = attitude_dot.x();
    array[8] = attitude_dot.y();
    array[9] = attitude_dot.z();
    
    // Angular velocity derivative (angular acceleration)
    array[10] = angular_velocity_dot.x();
    array[11] = angular_velocity_dot.y();
    array[12] = angular_velocity_dot.z();
}

void StateDerivative::fromArray(const double* array) {
    if (!array) return;
    
    position_dot = Vector3(array[0], array[1], array[2]);
    velocity_dot = Vector3(array[3], array[4], array[5]);
    attitude_dot = Quaternion(array[6], array[7], array[8], array[9]);
    angular_velocity_dot = Vector3(array[10], array[11], array[12]);
}

bool StateDerivative::isValid() const {
    return std::isfinite(position_dot.x()) && std::isfinite(position_dot.y()) && std::isfinite(position_dot.z()) &&
           std::isfinite(velocity_dot.x()) && std::isfinite(velocity_dot.y()) && std::isfinite(velocity_dot.z()) &&
           std::isfinite(attitude_dot.w()) && std::isfinite(attitude_dot.x()) && 
           std::isfinite(attitude_dot.y()) && std::isfinite(attitude_dot.z()) &&
           std::isfinite(angular_velocity_dot.x()) && std::isfinite(angular_velocity_dot.y()) && 
           std::isfinite(angular_velocity_dot.z());
}

// DynamicsEngine implementation
DynamicsEngine::DynamicsEngine(IntegrationMethod method)
    : mass_props_()
    , mass_props_set_(false)
    , integration_method_(method) {
}

void DynamicsEngine::setMassProperties(const MassProperties& mass_props) {
    if (!mass_props.isValid()) {
        throw std::invalid_argument("Invalid mass properties provided to DynamicsEngine");
    }
    
    mass_props_ = mass_props;
    mass_props_set_ = true;
}

KiteState DynamicsEngine::integrate(const KiteState& current_state,
                                   const Forces& total_forces,
                                   double delta_time) const {
    if (!isConfigured()) {
        throw std::runtime_error("DynamicsEngine not properly configured");
    }
    
    if (!validateState(current_state)) {
        throw std::invalid_argument("Invalid current state provided to integrate");
    }
    
    if (!validateForces(total_forces)) {
        throw std::invalid_argument("Invalid forces provided to integrate");
    }
    
    if (delta_time <= 0.0 || !std::isfinite(delta_time)) {
        throw std::invalid_argument("Invalid time step provided to integrate");
    }
    
    switch (integration_method_) {
        case IntegrationMethod::EULER:
            return integrateEuler(current_state, total_forces, delta_time);
        case IntegrationMethod::RUNGE_KUTTA_4:
            return integrateRungeKutta4(current_state, total_forces, delta_time);
        default:
            throw std::runtime_error("Unknown integration method");
    }
}

StateDerivative DynamicsEngine::calculateDerivative(const KiteState& state,
                                                    const Forces& forces) const {
    StateDerivative derivative;
    
    // Position derivative is velocity
    derivative.position_dot = state.velocity();
    
    // Velocity derivative is acceleration
    derivative.velocity_dot = calculateTranslationalAcceleration(forces.force);
    
    // Attitude derivative from angular velocity
    derivative.attitude_dot = calculateAttitudeDerivative(state.attitude(), state.angularVelocity());
    
    // Angular velocity derivative is angular acceleration
    // Convert moment to body frame for rotational dynamics
    Vector3 body_moment = worldToBody(forces.moment, state.attitude());
    derivative.angular_velocity_dot = calculateRotationalAcceleration(body_moment, state.angularVelocity());
    
    return derivative;
}

Vector3 DynamicsEngine::calculateTranslationalAcceleration(const Vector3& total_force) const {
    // F = ma, so a = F/m
    return total_force / mass_props_.mass;
}

Vector3 DynamicsEngine::calculateRotationalAcceleration(const Vector3& total_moment,
                                                       const Vector3& angular_velocity) const {
    // Euler's equation: M = I*α + ω × (I*ω)
    // Solve for α: α = I^(-1) * (M - ω × (I*ω))
    
    // Calculate I*ω
    Vector3 I_omega = mass_props_.inertia_tensor * angular_velocity;
    
    // Calculate ω × (I*ω)
    Vector3 omega_cross_I_omega = angular_velocity.cross(I_omega);
    
    // Calculate M - ω × (I*ω)
    Vector3 net_moment = total_moment - omega_cross_I_omega;
    
    // Calculate α = I^(-1) * net_moment
    Matrix3x3 inertia_inverse = mass_props_.inertia_tensor.inverse();
    Vector3 angular_acceleration = inertia_inverse * net_moment;
    
    return angular_acceleration;
}

Quaternion DynamicsEngine::calculateAttitudeDerivative(const Quaternion& attitude,
                                                      const Vector3& angular_velocity) const {
    // Quaternion derivative: q̇ = 0.5 * q * ω_quaternion
    // where ω_quaternion = [0, ωx, ωy, ωz]
    
    Quaternion omega_quat(0.0, angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
    Quaternion attitude_dot = attitude * omega_quat;
    
    // Scale by 0.5
    return Quaternion(attitude_dot.w() * 0.5, attitude_dot.x() * 0.5, 
                     attitude_dot.y() * 0.5, attitude_dot.z() * 0.5);
}

Vector3 DynamicsEngine::worldToBody(const Vector3& world_vector, const Quaternion& attitude) const {
    // Use quaternion conjugate to transform from world to body frame
    return attitude.conjugate().rotate(world_vector);
}

Vector3 DynamicsEngine::bodyToWorld(const Vector3& body_vector, const Quaternion& attitude) const {
    // Use quaternion to transform from body to world frame
    return attitude.rotate(body_vector);
}

KiteState DynamicsEngine::integrateEuler(const KiteState& state,
                                        const Forces& forces,
                                        double dt) const {
    // Simple Euler integration: x(t+dt) = x(t) + dx/dt * dt
    
    StateDerivative derivative = calculateDerivative(state, forces);
    
    KiteState new_state = state;
    
    // Integrate position
    new_state.setPosition(state.position() + derivative.position_dot * dt);
    
    // Integrate velocity
    new_state.setVelocity(state.velocity() + derivative.velocity_dot * dt);
    
    // Integrate attitude
    Quaternion new_attitude = state.attitude();
    new_attitude = Quaternion(
        new_attitude.w() + derivative.attitude_dot.w() * dt,
        new_attitude.x() + derivative.attitude_dot.x() * dt,
        new_attitude.y() + derivative.attitude_dot.y() * dt,
        new_attitude.z() + derivative.attitude_dot.z() * dt
    );
    normalizeQuaternion(new_attitude);
    new_state.setAttitude(new_attitude);
    
    // Integrate angular velocity
    new_state.setAngularVelocity(state.angularVelocity() + derivative.angular_velocity_dot * dt);
    
    // Update time
    new_state.setTime(state.time() + dt);
    
    return new_state;
}

KiteState DynamicsEngine::integrateRungeKutta4(const KiteState& state,
                                              const Forces& forces,
                                              double dt) const {
    // 4th order Runge-Kutta integration
    // k1 = f(t, y)
    // k2 = f(t + dt/2, y + k1*dt/2)
    // k3 = f(t + dt/2, y + k2*dt/2)
    // k4 = f(t + dt, y + k3*dt)
    // y_new = y + (k1 + 2*k2 + 2*k3 + k4) * dt/6
    
    StateDerivative k1 = calculateDerivative(state, forces);
    
    KiteState state_k2 = addScaledDerivative(state, k1, dt * 0.5);
    StateDerivative k2 = calculateDerivative(state_k2, forces);
    
    KiteState state_k3 = addScaledDerivative(state, k2, dt * 0.5);
    StateDerivative k3 = calculateDerivative(state_k3, forces);
    
    KiteState state_k4 = addScaledDerivative(state, k3, dt);
    StateDerivative k4 = calculateDerivative(state_k4, forces);
    
    // Combine derivatives with RK4 weights
    StateDerivative combined;
    combined.position_dot = (k1.position_dot + k2.position_dot * 2.0 + k3.position_dot * 2.0 + k4.position_dot) / 6.0;
    combined.velocity_dot = (k1.velocity_dot + k2.velocity_dot * 2.0 + k3.velocity_dot * 2.0 + k4.velocity_dot) / 6.0;
    combined.attitude_dot = Quaternion(
        (k1.attitude_dot.w() + k2.attitude_dot.w() * 2.0 + k3.attitude_dot.w() * 2.0 + k4.attitude_dot.w()) / 6.0,
        (k1.attitude_dot.x() + k2.attitude_dot.x() * 2.0 + k3.attitude_dot.x() * 2.0 + k4.attitude_dot.x()) / 6.0,
        (k1.attitude_dot.y() + k2.attitude_dot.y() * 2.0 + k3.attitude_dot.y() * 2.0 + k4.attitude_dot.y()) / 6.0,
        (k1.attitude_dot.z() + k2.attitude_dot.z() * 2.0 + k3.attitude_dot.z() * 2.0 + k4.attitude_dot.z()) / 6.0
    );
    combined.angular_velocity_dot = (k1.angular_velocity_dot + k2.angular_velocity_dot * 2.0 + 
                                    k3.angular_velocity_dot * 2.0 + k4.angular_velocity_dot) / 6.0;
    
    // Apply combined derivative
    KiteState new_state = addScaledDerivative(state, combined, dt);
    new_state.setTime(state.time() + dt);
    
    return new_state;
}

StateDerivative DynamicsEngine::evaluateDerivative(const KiteState& state,
                                                   const Forces& forces,
                                                   double dt,
                                                   const StateDerivative& derivative) const {
    KiteState temp_state = addScaledDerivative(state, derivative, dt);
    return calculateDerivative(temp_state, forces);
}

KiteState DynamicsEngine::addScaledDerivative(const KiteState& state,
                                             const StateDerivative& derivative,
                                             double scale) const {
    KiteState new_state = state;
    
    // Add scaled derivatives
    new_state.setPosition(state.position() + derivative.position_dot * scale);
    new_state.setVelocity(state.velocity() + derivative.velocity_dot * scale);
    
    // Handle quaternion addition carefully
    Quaternion new_attitude = state.attitude();
    new_attitude = Quaternion(
        new_attitude.w() + derivative.attitude_dot.w() * scale,
        new_attitude.x() + derivative.attitude_dot.x() * scale,
        new_attitude.y() + derivative.attitude_dot.y() * scale,
        new_attitude.z() + derivative.attitude_dot.z() * scale
    );
    normalizeQuaternion(new_attitude);
    new_state.setAttitude(new_attitude);
    
    new_state.setAngularVelocity(state.angularVelocity() + derivative.angular_velocity_dot * scale);
    
    return new_state;
}

void DynamicsEngine::normalizeQuaternion(Quaternion& q) const {
    q.normalize();
}

bool DynamicsEngine::isQuaternionValid(const Quaternion& q) const {
    return q.isUnit();
}

bool DynamicsEngine::isConfigured() const {
    return mass_props_set_ && isMassValid() && isInertiaValid() && isCenterOfGravityValid();
}

bool DynamicsEngine::validateState(const KiteState& state) const {
    return state.isValid();
}

bool DynamicsEngine::validateForces(const Forces& forces) const {
    return forces.isValid();
}

void DynamicsEngine::reset() {
    mass_props_ = MassProperties();
    mass_props_set_ = false;
    integration_method_ = IntegrationMethod::RUNGE_KUTTA_4;
}

bool DynamicsEngine::isMassValid() const {
    return mass_props_.mass > 0.0 && std::isfinite(mass_props_.mass);
}

bool DynamicsEngine::isInertiaValid() const {
    // Check that inertia tensor is positive definite
    // For now, just check that diagonal elements are positive
    return mass_props_.inertia_tensor(0, 0) > 0.0 &&
           mass_props_.inertia_tensor(1, 1) > 0.0 &&
           mass_props_.inertia_tensor(2, 2) > 0.0 &&
           std::isfinite(mass_props_.inertia_tensor(0, 0)) &&
           std::isfinite(mass_props_.inertia_tensor(1, 1)) &&
           std::isfinite(mass_props_.inertia_tensor(2, 2));
}

bool DynamicsEngine::isCenterOfGravityValid() const {
    return std::isfinite(mass_props_.center_of_gravity.x()) &&
           std::isfinite(mass_props_.center_of_gravity.y()) &&
           std::isfinite(mass_props_.center_of_gravity.z());
}

} // namespace kite_sim