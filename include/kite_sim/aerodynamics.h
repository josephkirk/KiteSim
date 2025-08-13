#pragma once

#include "math_types.h"
#include "kite_state.h"
#include "kite_geometry.h"
#include "wind_model.h"

namespace kite_sim {

/**
 * @brief Control surface deflections and inputs
 */
struct ControlInputs {
    double elevator = 0.0;    // Elevator deflection (rad)
    double rudder = 0.0;      // Rudder deflection (rad)
    double aileron = 0.0;     // Aileron deflection (rad)
    
    ControlInputs() = default;
    ControlInputs(double elev, double rud, double ail) 
        : elevator(elev), rudder(rud), aileron(ail) {}
    
    bool isValid() const;
};

/**
 * @brief Aerodynamic forces and moments acting on the kite
 */
struct AeroForces {
    Vector3 force;            // Total aerodynamic force in world coordinates (N)
    Vector3 moment;           // Total aerodynamic moment about CG in body coordinates (N⋅m)
    
    // Force components in body coordinates
    Vector3 force_body;       // Force in body coordinates (N)
    
    // Individual force components (for analysis)
    double lift = 0.0;        // Lift force magnitude (N)
    double drag = 0.0;        // Drag force magnitude (N)
    double side_force = 0.0;  // Side force magnitude (N)
    
    // Moment components
    double roll_moment = 0.0;   // Roll moment (N⋅m)
    double pitch_moment = 0.0;  // Pitch moment (N⋅m)
    double yaw_moment = 0.0;    // Yaw moment (N⋅m)
    
    // Flow angles (for analysis)
    double angle_of_attack = 0.0;  // Angle of attack (rad)
    double sideslip_angle = 0.0;   // Sideslip angle (rad)
    
    // Dynamic pressure
    double dynamic_pressure = 0.0; // Dynamic pressure (Pa)
    
    AeroForces() = default;
    
    bool isValid() const;
    void reset();
};

/**
 * @brief Aerodynamics calculator for kite forces and moments
 * 
 * Calculates aerodynamic forces and moments acting on a kite based on:
 * - Current kite state (position, velocity, attitude)
 * - Wind conditions
 * - Kite geometry and aerodynamic coefficients
 * - Control surface deflections
 */
class AerodynamicsCalculator {
public:
    // Constructor
    AerodynamicsCalculator();
    explicit AerodynamicsCalculator(const KiteGeometry& geometry);
    
    // Configuration
    void setGeometry(const KiteGeometry& geometry);
    const KiteGeometry& getGeometry() const { return geometry_; }
    
    // Main calculation interface
    AeroForces calculateForces(const KiteState& state, 
                              const WindVector& wind,
                              const ControlInputs& controls = ControlInputs()) const;
    
    // Individual calculation components
    Vector3 calculateRelativeWind(const KiteState& state, const WindVector& wind) const;
    std::pair<double, double> calculateFlowAngles(const Vector3& relative_wind_body) const;
    double calculateDynamicPressure(const Vector3& relative_wind) const;
    
    // Force and moment calculations
    Vector3 calculateAerodynamicForce(double alpha, double beta, double delta,
                                     double dynamic_pressure) const;
    Vector3 calculateAerodynamicMoment(double alpha, double beta, double delta,
                                      double dynamic_pressure) const;
    
    // Coordinate transformations
    Vector3 transformWindToBody(const Vector3& wind_vector, const Quaternion& attitude) const;
    Vector3 transformBodyToWorld(const Vector3& body_vector, const Quaternion& attitude) const;
    
    // Validation
    bool isConfigured() const { return geometry_configured_; }
    bool validateInputs(const KiteState& state, const WindVector& wind, 
                       const ControlInputs& controls) const;
    
    // Constants
    static constexpr double AIR_DENSITY = 1.225; // kg/m³ at sea level, 15°C
    
private:
    KiteGeometry geometry_;
    bool geometry_configured_;
    
    // Helper functions
    double radToDeg(double rad) const { return rad * 180.0 / M_PI; }
    double degToRad(double deg) const { return deg * M_PI / 180.0; }
    
    // Validation helpers
    bool isStateValid(const KiteState& state) const;
    bool isWindValid(const WindVector& wind) const;
    bool isControlValid(const ControlInputs& controls) const;
    
    // Calculation helpers
    Vector3 calculateLiftDragSide(double cl, double cd, double cs, 
                                 double dynamic_pressure,
                                 const Vector3& relative_wind_body) const;
    
    Vector3 calculateMomentComponents(double cl, double cd, double cm,
                                     double dynamic_pressure) const;
};

} // namespace kite_sim