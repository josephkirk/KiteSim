#include "kite_sim/aerodynamics.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace kite_sim {

// ControlInputs implementation
bool ControlInputs::isValid() const {
    const double max_deflection = M_PI / 4; // 45 degrees max
    return std::abs(elevator) <= max_deflection &&
           std::abs(rudder) <= max_deflection &&
           std::abs(aileron) <= max_deflection;
}

// AeroForces implementation
bool AeroForces::isValid() const {
    // Check for NaN or infinite values
    auto isFinite = [](double val) { return std::isfinite(val); };
    
    return isFinite(lift) && isFinite(drag) && isFinite(side_force) &&
           isFinite(roll_moment) && isFinite(pitch_moment) && isFinite(yaw_moment) &&
           isFinite(angle_of_attack) && isFinite(sideslip_angle) &&
           isFinite(dynamic_pressure) && dynamic_pressure >= 0.0;
}

void AeroForces::reset() {
    force = Vector3(0, 0, 0);
    moment = Vector3(0, 0, 0);
    force_body = Vector3(0, 0, 0);
    lift = drag = side_force = 0.0;
    roll_moment = pitch_moment = yaw_moment = 0.0;
    angle_of_attack = sideslip_angle = 0.0;
    dynamic_pressure = 0.0;
}

// AerodynamicsCalculator implementation
AerodynamicsCalculator::AerodynamicsCalculator() 
    : geometry_configured_(false) {
}

AerodynamicsCalculator::AerodynamicsCalculator(const KiteGeometry& geometry)
    : geometry_(geometry), geometry_configured_(true) {
    if (!geometry_.isValid()) {
        throw std::invalid_argument("Invalid kite geometry provided to AerodynamicsCalculator");
    }
}

void AerodynamicsCalculator::setGeometry(const KiteGeometry& geometry) {
    if (!geometry.isValid()) {
        throw std::invalid_argument("Invalid kite geometry provided to AerodynamicsCalculator");
    }
    geometry_ = geometry;
    geometry_configured_ = true;
}

AeroForces AerodynamicsCalculator::calculateForces(const KiteState& state, 
                                                  const WindVector& wind,
                                                  const ControlInputs& controls) const {
    if (!geometry_configured_) {
        throw std::runtime_error("AerodynamicsCalculator not configured with geometry");
    }
    
    if (!validateInputs(state, wind, controls)) {
        throw std::invalid_argument("Invalid inputs to calculateForces");
    }
    
    AeroForces result;
    
    // Calculate relative wind in body coordinates
    Vector3 relative_wind_world = calculateRelativeWind(state, wind);
    Vector3 relative_wind_body = transformWindToBody(relative_wind_world, state.attitude());
    
    // Calculate flow angles
    auto [alpha, beta] = calculateFlowAngles(relative_wind_body);
    result.angle_of_attack = alpha;
    result.sideslip_angle = beta;
    
    // Calculate dynamic pressure
    result.dynamic_pressure = calculateDynamicPressure(relative_wind_world);
    
    // Get control deflection (simplified - use elevator for now)
    double delta = controls.elevator;
    
    // Calculate aerodynamic coefficients
    double alpha_deg = radToDeg(alpha);
    double beta_deg = radToDeg(beta);
    double delta_deg = radToDeg(delta);
    
    const auto& coeffs = geometry_.aeroCoefficients();
    double cl = coeffs.getCL(alpha_deg, beta_deg, delta_deg);
    double cd = coeffs.getCD(alpha_deg, beta_deg, delta_deg);
    double cm = coeffs.getCM(alpha_deg, beta_deg, delta_deg);
    
    // Calculate forces in body coordinates
    result.force_body = calculateAerodynamicForce(alpha, beta, delta, result.dynamic_pressure);
    
    // Extract individual force components
    result.lift = cl * result.dynamic_pressure * geometry_.wingArea();
    result.drag = cd * result.dynamic_pressure * geometry_.wingArea();
    result.side_force = 0.0; // Simplified for now
    
    // Transform forces to world coordinates
    result.force = transformBodyToWorld(result.force_body, state.attitude());
    
    // Calculate moments in body coordinates
    result.moment = calculateAerodynamicMoment(alpha, beta, delta, result.dynamic_pressure);
    result.roll_moment = result.moment.x();
    result.pitch_moment = result.moment.y();
    result.yaw_moment = result.moment.z();
    
    return result;
}

Vector3 AerodynamicsCalculator::calculateRelativeWind(const KiteState& state, 
                                                     const WindVector& wind) const {
    // Relative wind = wind velocity - kite velocity
    return wind.velocity() - state.velocity();
}

std::pair<double, double> AerodynamicsCalculator::calculateFlowAngles(
    const Vector3& relative_wind_body) const {
    
    // In body coordinates: x-forward, y-right, z-down
    double u = relative_wind_body.x(); // Forward component
    double v = relative_wind_body.y(); // Right component  
    double w = relative_wind_body.z(); // Down component
    
    // Angle of attack: angle between relative wind and x-y plane
    double alpha = std::atan2(w, u);
    
    // Sideslip angle: angle between relative wind projection on x-z plane and x-axis
    double beta = std::atan2(v, std::sqrt(u*u + w*w));
    
    return {alpha, beta};
}

double AerodynamicsCalculator::calculateDynamicPressure(const Vector3& relative_wind) const {
    double airspeed = relative_wind.magnitude();
    return 0.5 * AIR_DENSITY * airspeed * airspeed;
}

Vector3 AerodynamicsCalculator::calculateAerodynamicForce(double alpha, double beta, double delta,
                                                         double dynamic_pressure) const {
    double alpha_deg = radToDeg(alpha);
    double beta_deg = radToDeg(beta);
    double delta_deg = radToDeg(delta);
    
    const auto& coeffs = geometry_.aeroCoefficients();
    double cl = coeffs.getCL(alpha_deg, beta_deg, delta_deg);
    double cd = coeffs.getCD(alpha_deg, beta_deg, delta_deg);
    
    double wing_area = geometry_.wingArea();
    double q_s = dynamic_pressure * wing_area;
    
    // Calculate lift and drag forces
    double lift_force = cl * q_s;
    double drag_force = cd * q_s;
    
    // Transform from wind axes to body axes
    // In wind axes: x-drag (opposite to relative wind), z-lift (perpendicular up)
    // Need to rotate by angle of attack and sideslip angle
    
    double cos_alpha = std::cos(alpha);
    double sin_alpha = std::sin(alpha);
    double cos_beta = std::cos(beta);
    double sin_beta = std::sin(beta);
    
    // Force in body coordinates (simplified transformation)
    double fx = -drag_force * cos_alpha * cos_beta - lift_force * sin_alpha * cos_beta;
    double fy = -drag_force * sin_beta;
    double fz = drag_force * sin_alpha * cos_beta - lift_force * cos_alpha * cos_beta;
    
    return Vector3(fx, fy, fz);
}

Vector3 AerodynamicsCalculator::calculateAerodynamicMoment(double alpha, double beta, double delta,
                                                          double dynamic_pressure) const {
    double alpha_deg = radToDeg(alpha);
    double beta_deg = radToDeg(beta);
    double delta_deg = radToDeg(delta);
    
    const auto& coeffs = geometry_.aeroCoefficients();
    double cm = coeffs.getCM(alpha_deg, beta_deg, delta_deg);
    
    double wing_area = geometry_.wingArea();
    double mean_chord = geometry_.meanChord();
    double q_s_c = dynamic_pressure * wing_area * mean_chord;
    
    // Pitching moment (simplified - only pitch moment for now)
    double pitch_moment = cm * q_s_c;
    
    // Roll and yaw moments would require additional coefficients
    double roll_moment = 0.0;
    double yaw_moment = 0.0;
    
    return Vector3(roll_moment, pitch_moment, yaw_moment);
}

Vector3 AerodynamicsCalculator::transformWindToBody(const Vector3& wind_vector, 
                                                   const Quaternion& attitude) const {
    // Transform from world coordinates to body coordinates
    // Use quaternion conjugate to transform from world to body
    return attitude.conjugate().rotate(wind_vector);
}

Vector3 AerodynamicsCalculator::transformBodyToWorld(const Vector3& body_vector, 
                                                    const Quaternion& attitude) const {
    // Transform from body coordinates to world coordinates
    return attitude.rotate(body_vector);
}

bool AerodynamicsCalculator::validateInputs(const KiteState& state, const WindVector& wind, 
                                           const ControlInputs& controls) const {
    return isStateValid(state) && isWindValid(wind) && isControlValid(controls);
}

bool AerodynamicsCalculator::isStateValid(const KiteState& state) const {
    return state.isValid();
}

bool AerodynamicsCalculator::isWindValid(const WindVector& wind) const {
    const Vector3& vel = wind.velocity();
    return std::isfinite(vel.x()) && std::isfinite(vel.y()) && std::isfinite(vel.z());
}

bool AerodynamicsCalculator::isControlValid(const ControlInputs& controls) const {
    return controls.isValid();
}

} // namespace kite_sim