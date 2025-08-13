#pragma once

#include "math_types.h"
#include "kite_state.h"
#include "kite_geometry.h"

namespace kite_sim {

/**
 * @brief Forces and moments acting on the kite
 * 
 * Container for all external forces and moments that act on the kite,
 * expressed in the world coordinate frame.
 */
struct Forces {
    Vector3 force;    // Total force in world coordinates (N)
    Vector3 moment;   // Total moment about CG in world coordinates (N⋅m)
    
    Forces() : force(0.0, 0.0, 0.0), moment(0.0, 0.0, 0.0) {}
    Forces(const Vector3& f, const Vector3& m) : force(f), moment(m) {}
    
    Forces operator+(const Forces& other) const {
        return Forces(force + other.force, moment + other.moment);
    }
    
    Forces& operator+=(const Forces& other) {
        force += other.force;
        moment += other.moment;
        return *this;
    }
    
    Forces operator*(double scalar) const {
        return Forces(force * scalar, moment * scalar);
    }
    
    Forces& operator*=(double scalar) {
        force *= scalar;
        moment *= scalar;
        return *this;
    }
    
    void reset() {
        force = Vector3(0.0, 0.0, 0.0);
        moment = Vector3(0.0, 0.0, 0.0);
    }
    
    bool isValid() const;
};

/**
 * @brief State derivative for integration
 * 
 * Contains the time derivatives of all state variables for numerical integration.
 */
struct StateDerivative {
    Vector3 position_dot;        // Velocity (m/s)
    Vector3 velocity_dot;        // Acceleration (m/s²)
    Quaternion attitude_dot;     // Quaternion derivative (1/s)
    Vector3 angular_velocity_dot; // Angular acceleration (rad/s²)
    
    StateDerivative() 
        : position_dot(0.0, 0.0, 0.0)
        , velocity_dot(0.0, 0.0, 0.0)
        , attitude_dot(0.0, 0.0, 0.0, 0.0)
        , angular_velocity_dot(0.0, 0.0, 0.0) {}
    
    void toArray(double* array) const;
    void fromArray(const double* array);
    static constexpr int getSize() { return 13; }
    
    bool isValid() const;
};

/**
 * @brief Integration method enumeration
 */
enum class IntegrationMethod {
    EULER,           // Simple Euler integration (1st order)
    RUNGE_KUTTA_4    // 4th order Runge-Kutta integration
};

/**
 * @brief 6-DOF rigid body dynamics engine
 * 
 * Implements the equations of motion for a rigid body kite including:
 * - Translational dynamics: F = ma
 * - Rotational dynamics: M = I*α + ω × (I*ω)
 * - Quaternion-based attitude dynamics with proper normalization
 * - Numerical integration with multiple methods
 */
class DynamicsEngine {
public:
    // Constructor
    explicit DynamicsEngine(IntegrationMethod method = IntegrationMethod::RUNGE_KUTTA_4);
    
    // Copy constructor and assignment
    DynamicsEngine(const DynamicsEngine& other) = default;
    DynamicsEngine& operator=(const DynamicsEngine& other) = default;
    
    // Move constructor and assignment
    DynamicsEngine(DynamicsEngine&& other) noexcept = default;
    DynamicsEngine& operator=(DynamicsEngine&& other) noexcept = default;
    
    // Destructor
    ~DynamicsEngine() = default;
    
    // Configuration
    void setMassProperties(const MassProperties& mass_props);
    const MassProperties& getMassProperties() const { return mass_props_; }
    
    void setIntegrationMethod(IntegrationMethod method) { integration_method_ = method; }
    IntegrationMethod getIntegrationMethod() const { return integration_method_; }
    
    // Main integration function
    KiteState integrate(const KiteState& current_state,
                       const Forces& total_forces,
                       double delta_time) const;
    
    // State derivative calculation
    StateDerivative calculateDerivative(const KiteState& state,
                                       const Forces& forces) const;
    
    // Individual dynamics calculations
    Vector3 calculateTranslationalAcceleration(const Vector3& total_force) const;
    Vector3 calculateRotationalAcceleration(const Vector3& total_moment,
                                           const Vector3& angular_velocity) const;
    Quaternion calculateAttitudeDerivative(const Quaternion& attitude,
                                          const Vector3& angular_velocity) const;
    
    // Coordinate transformations
    Vector3 worldToBody(const Vector3& world_vector, const Quaternion& attitude) const;
    Vector3 bodyToWorld(const Vector3& body_vector, const Quaternion& attitude) const;
    
    // Validation
    bool isConfigured() const;
    bool validateState(const KiteState& state) const;
    bool validateForces(const Forces& forces) const;
    
    // Utility functions
    void reset();
    
private:
    // Mass properties
    MassProperties mass_props_;
    bool mass_props_set_;
    
    // Integration method
    IntegrationMethod integration_method_;
    
    // Integration implementations
    KiteState integrateEuler(const KiteState& state,
                            const Forces& forces,
                            double dt) const;
    
    KiteState integrateRungeKutta4(const KiteState& state,
                                  const Forces& forces,
                                  double dt) const;
    
    // Helper functions for RK4
    StateDerivative evaluateDerivative(const KiteState& state,
                                      const Forces& forces,
                                      double dt,
                                      const StateDerivative& derivative) const;
    
    KiteState addScaledDerivative(const KiteState& state,
                                 const StateDerivative& derivative,
                                 double scale) const;
    
    // Quaternion utilities
    void normalizeQuaternion(Quaternion& q) const;
    bool isQuaternionValid(const Quaternion& q) const;
    
    // Validation helpers
    bool isMassValid() const;
    bool isInertiaValid() const;
    bool isCenterOfGravityValid() const;
};

} // namespace kite_sim