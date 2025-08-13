#include "kite_sim/tether.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <algorithm>

namespace kite_sim {

// TetherProperties implementation
TetherProperties::TetherProperties()
    : length_(100.0)
    , mass_per_length_(0.01)
    , diameter_(0.005)
    , elastic_modulus_(0.0)
    , drag_coefficient_(1.2)
    , kite_attachment_(0.0, 0.0, 0.0)
    , ground_attachment_(0.0, 0.0, 0.0) {
}

TetherProperties::TetherProperties(double length, double mass_per_length, double diameter,
                                  double elastic_modulus, double drag_coefficient)
    : length_(length)
    , mass_per_length_(mass_per_length)
    , diameter_(diameter)
    , elastic_modulus_(elastic_modulus)
    , drag_coefficient_(drag_coefficient)
    , kite_attachment_(0.0, 0.0, 0.0)
    , ground_attachment_(0.0, 0.0, 0.0) {
}

double TetherProperties::crossSectionalArea() const {
    return M_PI * diameter_ * diameter_ / 4.0;
}

double TetherProperties::elasticConstant() const {
    if (!isElastic()) return 0.0;
    return elastic_modulus_ * crossSectionalArea() / length_;
}

bool TetherProperties::isValid() const {
    return length_ > 0.0 &&
           mass_per_length_ > 0.0 &&
           diameter_ > 0.0 &&
           elastic_modulus_ >= 0.0 &&
           drag_coefficient_ >= 0.0;
}

std::vector<std::string> TetherProperties::getValidationErrors() const {
    std::vector<std::string> errors;
    
    if (length_ <= 0.0) {
        errors.push_back("Tether length must be positive");
    }
    if (mass_per_length_ <= 0.0) {
        errors.push_back("Mass per length must be positive");
    }
    if (diameter_ <= 0.0) {
        errors.push_back("Tether diameter must be positive");
    }
    if (elastic_modulus_ < 0.0) {
        errors.push_back("Elastic modulus cannot be negative");
    }
    if (drag_coefficient_ < 0.0) {
        errors.push_back("Drag coefficient cannot be negative");
    }
    
    return errors;
}

std::string TetherProperties::toJson() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    
    oss << "{\n";
    oss << "  \"length\": " << length_ << ",\n";
    oss << "  \"mass_per_length\": " << mass_per_length_ << ",\n";
    oss << "  \"diameter\": " << diameter_ << ",\n";
    oss << "  \"elastic_modulus\": " << elastic_modulus_ << ",\n";
    oss << "  \"drag_coefficient\": " << drag_coefficient_ << ",\n";
    oss << "  \"kite_attachment\": [" << kite_attachment_.x() << ", " 
        << kite_attachment_.y() << ", " << kite_attachment_.z() << "],\n";
    oss << "  \"ground_attachment\": [" << ground_attachment_.x() << ", " 
        << ground_attachment_.y() << ", " << ground_attachment_.z() << "]\n";
    oss << "}";
    
    return oss.str();
}

bool TetherProperties::fromJson(const std::string& json_str) {
    // Simple JSON parsing - in production would use proper JSON library
    // For now, implement basic parsing for testing
    
    // Reset to defaults first
    *this = TetherProperties();
    
    // Extract numeric values using simple string parsing
    size_t pos = json_str.find("\"length\":");
    if (pos != std::string::npos) {
        size_t start = json_str.find(":", pos) + 1;
        size_t end = json_str.find(",", start);
        if (end == std::string::npos) end = json_str.find("}", start);
        if (start != std::string::npos && end != std::string::npos) {
            std::string value_str = json_str.substr(start, end - start);
            length_ = std::stod(value_str);
        }
    }
    
    return isValid();
}

bool TetherProperties::operator==(const TetherProperties& other) const {
    const double tolerance = 1e-9;
    
    return std::abs(length_ - other.length_) < tolerance &&
           std::abs(mass_per_length_ - other.mass_per_length_) < tolerance &&
           std::abs(diameter_ - other.diameter_) < tolerance &&
           std::abs(elastic_modulus_ - other.elastic_modulus_) < tolerance &&
           std::abs(drag_coefficient_ - other.drag_coefficient_) < tolerance;
}

bool TetherProperties::operator!=(const TetherProperties& other) const {
    return !(*this == other);
}

TetherProperties TetherProperties::createDefaultTether() {
    return TetherProperties(100.0, 0.01, 0.005, 0.0, 1.2);
}

TetherProperties TetherProperties::createElasticTether() {
    return TetherProperties(100.0, 0.01, 0.005, 1e9, 1.2);  // 1 GPa elastic modulus
}

TetherProperties TetherProperties::createInelasticTether() {
    return TetherProperties(100.0, 0.01, 0.005, 0.0, 1.2);
}

// TetherForces implementation
TetherForces::TetherForces()
    : force_(0.0, 0.0, 0.0)
    , moment_(0.0, 0.0, 0.0)
    , tension_(0.0)
    , current_length_(0.0)
    , constraint_active_(false) {
}

TetherForces::TetherForces(const Vector3& f, const Vector3& m, double t, double len, bool active)
    : force_(f)
    , moment_(m)
    , tension_(t)
    , current_length_(len)
    , constraint_active_(active) {
}

bool TetherForces::isValid() const {
    return std::isfinite(force_.x()) && std::isfinite(force_.y()) && std::isfinite(force_.z()) &&
           std::isfinite(moment_.x()) && std::isfinite(moment_.y()) && std::isfinite(moment_.z()) &&
           std::isfinite(tension_) && std::isfinite(current_length_) &&
           tension_ >= 0.0 && current_length_ >= 0.0;
}

void TetherForces::reset() {
    force_ = Vector3(0.0, 0.0, 0.0);
    moment_ = Vector3(0.0, 0.0, 0.0);
    tension_ = 0.0;
    current_length_ = 0.0;
    constraint_active_ = false;
}

// TetherModel implementation
TetherModel::TetherModel()
    : properties_()
    , air_density_(1.225) {  // Standard air density at sea level
}

TetherModel::TetherModel(const TetherProperties& properties)
    : properties_(properties)
    , air_density_(1.225) {
}

TetherForces TetherModel::calculateTetherForces(const Vector3& kite_position,
                                               const Quaternion& kite_attitude,
                                               const Vector3& kite_velocity,
                                               const Vector3& wind_velocity) const {
    if (!isValid()) {
        return TetherForces();
    }
    
    if (properties_.isElastic()) {
        return calculateElasticForces(kite_position, kite_attitude, kite_velocity, wind_velocity);
    } else {
        return calculateInelasticForces(kite_position, kite_attitude, kite_velocity, wind_velocity);
    }
}

bool TetherModel::checkConstraints(const Vector3& kite_position) const {
    if (!properties_.isElastic()) {
        // For inelastic tether, check if current length exceeds nominal length
        double current_length = getCurrentLength(kite_position);
        return current_length <= properties_.length() + 1e-6;  // Small tolerance for numerical errors
    }
    
    // Elastic tethers don't have hard constraints
    return true;
}

double TetherModel::getCurrentLength(const Vector3& kite_position) const {
    Vector3 tether_vector = getTetherVector(kite_position);
    return tether_vector.magnitude();
}

double TetherModel::getConstraintViolation(const Vector3& kite_position) const {
    if (properties_.isElastic()) {
        return 0.0;  // No constraint violation for elastic tethers
    }
    
    double current_length = getCurrentLength(kite_position);
    return std::max(0.0, current_length - properties_.length());
}

bool TetherModel::isTetherTaut(const Vector3& kite_position) const {
    double current_length = getCurrentLength(kite_position);
    return current_length >= properties_.length() - 1e-6;
}

Vector3 TetherModel::getTetherDirection(const Vector3& kite_position) const {
    Vector3 tether_vector = getTetherVector(kite_position);
    double length = tether_vector.magnitude();
    
    if (length < 1e-10) {
        return Vector3(0.0, 0.0, 1.0);  // Default direction if positions coincide
    }
    
    return tether_vector / length;
}

Vector3 TetherModel::getTetherVector(const Vector3& kite_position) const {
    // Vector from ground attachment to kite position
    return kite_position - properties_.groundAttachmentPoint();
}

bool TetherModel::isValid() const {
    return properties_.isValid() && air_density_ > 0.0;
}

TetherModel TetherModel::createElasticModel(double length, double mass_per_length, 
                                           double diameter, double elastic_modulus) {
    TetherProperties props(length, mass_per_length, diameter, elastic_modulus, 1.2);
    return TetherModel(props);
}

TetherModel TetherModel::createInelasticModel(double length, double mass_per_length, 
                                             double diameter) {
    TetherProperties props(length, mass_per_length, diameter, 0.0, 1.2);
    return TetherModel(props);
}

TetherForces TetherModel::calculateElasticForces(const Vector3& kite_position,
                                                const Quaternion& kite_attitude,
                                                const Vector3& kite_velocity,
                                                const Vector3& wind_velocity) const {
    Vector3 tether_vector = getTetherVector(kite_position);
    double current_length = tether_vector.magnitude();
    
    if (current_length < 1e-10) {
        return TetherForces();  // No forces if kite is at ground attachment point
    }
    
    Vector3 tether_direction = tether_vector / current_length;
    
    // Calculate elastic tension
    double extension = current_length - properties_.length();
    double elastic_constant = properties_.elasticConstant();
    double tension = std::max(0.0, elastic_constant * extension);
    
    // Tension force (always along tether, toward ground)
    Vector3 tension_force = -tension * tether_direction;
    
    // Add drag force
    Vector3 drag_force = calculateDragForce(kite_position, kite_velocity, wind_velocity);
    
    // Total force
    Vector3 total_force = tension_force + drag_force;
    
    // Calculate moment about kite center of gravity
    Vector3 moment_arm = calculateMomentArm(kite_position, kite_attitude);
    Vector3 moment = moment_arm.cross(total_force);
    
    return TetherForces(total_force, moment, tension, current_length, tension > 0.0);
}

TetherForces TetherModel::calculateInelasticForces(const Vector3& kite_position,
                                                  const Quaternion& kite_attitude,
                                                  const Vector3& kite_velocity,
                                                  const Vector3& wind_velocity) const {
    Vector3 tether_vector = getTetherVector(kite_position);
    double current_length = tether_vector.magnitude();
    
    if (current_length < 1e-10) {
        return TetherForces();
    }
    
    Vector3 tether_direction = tether_vector / current_length;
    
    // For inelastic tether, tension is determined by constraint forces
    // This is a simplified model - in reality would need constraint solver
    double tension = 0.0;
    bool constraint_active = false;
    
    if (current_length >= properties_.length()) {
        // Tether is taut - calculate constraint force
        constraint_active = true;
        
        // Simple approximation: tension proportional to constraint violation
        double violation = current_length - properties_.length();
        tension = 1e6 * violation;  // High stiffness to approximate constraint
    }
    
    // Tension force
    Vector3 tension_force = -tension * tether_direction;
    
    // Add drag force
    Vector3 drag_force = calculateDragForce(kite_position, kite_velocity, wind_velocity);
    
    // Total force
    Vector3 total_force = tension_force + drag_force;
    
    // Calculate moment
    Vector3 moment_arm = calculateMomentArm(kite_position, kite_attitude);
    Vector3 moment = moment_arm.cross(total_force);
    
    return TetherForces(total_force, moment, tension, current_length, constraint_active);
}

Vector3 TetherModel::calculateDragForce(const Vector3& kite_position,
                                       const Vector3& kite_velocity,
                                       const Vector3& wind_velocity) const {
    // Simplified tether drag model
    // In reality, would integrate drag along tether length
    
    Vector3 relative_velocity = kite_velocity - wind_velocity;
    Vector3 tether_direction = getTetherDirection(kite_position);
    
    // Component of velocity perpendicular to tether
    Vector3 perpendicular_velocity = relative_velocity - 
        tether_direction * relative_velocity.dot(tether_direction);
    
    double perpendicular_speed = perpendicular_velocity.magnitude();
    
    if (perpendicular_speed < 1e-10) {
        return Vector3(0.0, 0.0, 0.0);
    }
    
    // Drag force magnitude
    double drag_area = properties_.diameter() * properties_.length();  // Projected area
    double drag_magnitude = 0.5 * air_density_ * properties_.dragCoefficient() * 
                           drag_area * perpendicular_speed * perpendicular_speed;
    
    // Drag force direction (opposite to perpendicular velocity)
    Vector3 drag_direction = -perpendicular_velocity / perpendicular_speed;
    
    return drag_magnitude * drag_direction;
}

Vector3 TetherModel::calculateMomentArm(const Vector3& kite_position,
                                       const Quaternion& kite_attitude) const {
    // Transform tether attachment point from body to world coordinates
    Vector3 attachment_world = kite_attitude.rotate(properties_.kiteAttachmentPoint());
    
    // Moment arm is from kite CG to tether attachment point
    return attachment_world;
}

bool TetherModel::arePropertiesValid() const {
    return properties_.isValid();
}

bool TetherModel::isPositionValid(const Vector3& position) const {
    return std::isfinite(position.x()) && 
           std::isfinite(position.y()) && 
           std::isfinite(position.z());
}

} // namespace kite_sim