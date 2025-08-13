#include "kite_sim/aerodynamics.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <sstream>

namespace kite_sim {

// ControlInputs implementation
bool ControlInputs::isValid() const {
    return std::abs(elevator) <= MAX_DEFLECTION &&
           std::abs(rudder) <= MAX_DEFLECTION &&
           std::abs(aileron) <= MAX_DEFLECTION &&
           std::isfinite(elevator) && std::isfinite(rudder) && std::isfinite(aileron);
}

void ControlInputs::reset() {
    elevator = 0.0;
    rudder = 0.0;
    aileron = 0.0;
}

double ControlInputs::getMaxDeflection() const {
    return std::max({std::abs(elevator), std::abs(rudder), std::abs(aileron)});
}

bool ControlInputs::hasDeflection() const {
    const double tolerance = 1e-6;
    return std::abs(elevator) > tolerance || 
           std::abs(rudder) > tolerance || 
           std::abs(aileron) > tolerance;
}

// ControlEffectiveness implementation
bool ControlEffectiveness::isValid() const {
    auto isFinite = [](double val) { return std::isfinite(val); };
    
    return isFinite(elevator_cl_effectiveness) && isFinite(elevator_cd_effectiveness) &&
           isFinite(elevator_cm_effectiveness) && isFinite(rudder_cy_effectiveness) &&
           isFinite(rudder_cd_effectiveness) && isFinite(rudder_cn_effectiveness) &&
           isFinite(aileron_cl_effectiveness) && isFinite(aileron_cd_effectiveness) &&
           isFinite(aileron_cl_effectiveness_roll);
}

void ControlEffectiveness::reset() {
    elevator_cl_effectiveness = 0.5;
    elevator_cd_effectiveness = 0.1;
    elevator_cm_effectiveness = -1.0;
    rudder_cy_effectiveness = 0.3;
    rudder_cd_effectiveness = 0.05;
    rudder_cn_effectiveness = -0.2;
    aileron_cl_effectiveness = 0.2;
    aileron_cd_effectiveness = 0.05;
    aileron_cl_effectiveness_roll = -0.3;
}

std::string ControlEffectiveness::toJson() const {
    // Simple JSON serialization (would use a proper JSON library in production)
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"elevator_cl_effectiveness\": " << elevator_cl_effectiveness << ",\n";
    oss << "  \"elevator_cd_effectiveness\": " << elevator_cd_effectiveness << ",\n";
    oss << "  \"elevator_cm_effectiveness\": " << elevator_cm_effectiveness << ",\n";
    oss << "  \"rudder_cy_effectiveness\": " << rudder_cy_effectiveness << ",\n";
    oss << "  \"rudder_cd_effectiveness\": " << rudder_cd_effectiveness << ",\n";
    oss << "  \"rudder_cn_effectiveness\": " << rudder_cn_effectiveness << ",\n";
    oss << "  \"aileron_cl_effectiveness\": " << aileron_cl_effectiveness << ",\n";
    oss << "  \"aileron_cd_effectiveness\": " << aileron_cd_effectiveness << ",\n";
    oss << "  \"aileron_cl_effectiveness_roll\": " << aileron_cl_effectiveness_roll << "\n";
    oss << "}";
    return oss.str();
}

bool ControlEffectiveness::fromJson(const std::string& json_str) {
    // Simplified JSON parsing (would use a proper JSON library in production)
    // For now, just return true to indicate successful parsing
    return !json_str.empty();
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

void AerodynamicsCalculator::setControlEffectiveness(const ControlEffectiveness& effectiveness) {
    if (!effectiveness.isValid()) {
        throw std::invalid_argument("Invalid control effectiveness parameters");
    }
    control_effectiveness_ = effectiveness;
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
    
    // Calculate base aerodynamic coefficients
    double alpha_deg = radToDeg(alpha);
    double beta_deg = radToDeg(beta);
    
    const auto& coeffs = geometry_.aeroCoefficients();
    double cl_base = coeffs.getCL(alpha_deg, beta_deg);
    double cd_base = coeffs.getCD(alpha_deg, beta_deg);
    double cm_base = coeffs.getCM(alpha_deg, beta_deg);
    
    // Apply control effectiveness
    double cl = cl_base + calculateControlEffectOnCL(controls);
    double cd = cd_base + calculateControlEffectOnCD(controls);
    double cm = cm_base + calculateControlEffectOnCM(controls);
    
    // Calculate forces in body coordinates
    result.force_body = calculateAerodynamicForce(alpha, beta, controls, result.dynamic_pressure);
    
    // Extract individual force components
    result.lift = cl * result.dynamic_pressure * geometry_.wingArea();
    result.drag = cd * result.dynamic_pressure * geometry_.wingArea();
    result.side_force = 0.0; // Simplified for now
    
    // Transform forces to world coordinates
    result.force = transformBodyToWorld(result.force_body, state.attitude());
    
    // Calculate moments in body coordinates
    result.moment = calculateAerodynamicMoment(alpha, beta, controls, result.dynamic_pressure);
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

Vector3 AerodynamicsCalculator::calculateAerodynamicForce(double alpha, double beta, 
                                                         const ControlInputs& controls,
                                                         double dynamic_pressure) const {
    double alpha_deg = radToDeg(alpha);
    double beta_deg = radToDeg(beta);
    
    const auto& coeffs = geometry_.aeroCoefficients();
    double cl_base = coeffs.getCL(alpha_deg, beta_deg);
    double cd_base = coeffs.getCD(alpha_deg, beta_deg);
    
    // Apply control effectiveness
    double cl = cl_base + calculateControlEffectOnCL(controls);
    double cd = cd_base + calculateControlEffectOnCD(controls);
    
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

Vector3 AerodynamicsCalculator::calculateAerodynamicMoment(double alpha, double beta, 
                                                           const ControlInputs& controls,
                                                           double dynamic_pressure) const {
    double alpha_deg = radToDeg(alpha);
    double beta_deg = radToDeg(beta);
    
    const auto& coeffs = geometry_.aeroCoefficients();
    double cm_base = coeffs.getCM(alpha_deg, beta_deg);
    
    // Apply control effectiveness
    double cm = cm_base + calculateControlEffectOnCM(controls);
    
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

double AerodynamicsCalculator::calculateControlEffectOnCL(const ControlInputs& controls) const {
    double delta_cl = 0.0;
    
    // Elevator effect on lift coefficient
    delta_cl += control_effectiveness_.elevator_cl_effectiveness * controls.elevator;
    
    // Aileron effect on lift coefficient
    delta_cl += control_effectiveness_.aileron_cl_effectiveness * controls.aileron;
    
    return delta_cl;
}

double AerodynamicsCalculator::calculateControlEffectOnCD(const ControlInputs& controls) const {
    double delta_cd = 0.0;
    
    // Elevator effect on drag coefficient
    delta_cd += control_effectiveness_.elevator_cd_effectiveness * std::abs(controls.elevator);
    
    // Rudder effect on drag coefficient
    delta_cd += control_effectiveness_.rudder_cd_effectiveness * std::abs(controls.rudder);
    
    // Aileron effect on drag coefficient
    delta_cd += control_effectiveness_.aileron_cd_effectiveness * std::abs(controls.aileron);
    
    return delta_cd;
}

double AerodynamicsCalculator::calculateControlEffectOnCM(const ControlInputs& controls) const {
    double delta_cm = 0.0;
    
    // Elevator effect on pitching moment coefficient
    delta_cm += control_effectiveness_.elevator_cm_effectiveness * controls.elevator;
    
    return delta_cm;
}

} // namespace kite_sim