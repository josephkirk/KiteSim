#include "kite_sim/kite_state.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>

namespace kite_sim {

KiteState::KiteState() 
    : position_(0.0, 0.0, 0.0)
    , velocity_(0.0, 0.0, 0.0)
    , attitude_(Quaternion::identity())
    , angular_velocity_(0.0, 0.0, 0.0)
    , time_(0.0) {
}

KiteState::KiteState(const Vector3& position, const Vector3& velocity, 
                     const Quaternion& attitude, const Vector3& angular_velocity,
                     double time)
    : position_(position)
    , velocity_(velocity)
    , attitude_(attitude)
    , angular_velocity_(angular_velocity)
    , time_(time) {
    
    // Ensure quaternion is normalized
    attitude_.normalize();
}

void KiteState::toStateVector(double* state_vector) const {
    if (!state_vector) return;
    
    // Position (3 elements)
    state_vector[0] = position_.x();
    state_vector[1] = position_.y();
    state_vector[2] = position_.z();
    
    // Velocity (3 elements)
    state_vector[3] = velocity_.x();
    state_vector[4] = velocity_.y();
    state_vector[5] = velocity_.z();
    
    // Quaternion (4 elements)
    state_vector[6] = attitude_.w();
    state_vector[7] = attitude_.x();
    state_vector[8] = attitude_.y();
    state_vector[9] = attitude_.z();
    
    // Angular velocity (3 elements)
    state_vector[10] = angular_velocity_.x();
    state_vector[11] = angular_velocity_.y();
    state_vector[12] = angular_velocity_.z();
}

void KiteState::fromStateVector(const double* state_vector) {
    if (!state_vector) return;
    
    // Position
    position_ = Vector3(state_vector[0], state_vector[1], state_vector[2]);
    
    // Velocity
    velocity_ = Vector3(state_vector[3], state_vector[4], state_vector[5]);
    
    // Quaternion
    attitude_ = Quaternion(state_vector[6], state_vector[7], 
                          state_vector[8], state_vector[9]);
    attitude_.normalize(); // Ensure unit quaternion
    
    // Angular velocity
    angular_velocity_ = Vector3(state_vector[10], state_vector[11], state_vector[12]);
}

void KiteState::reset() {
    position_ = Vector3(0.0, 0.0, 0.0);
    velocity_ = Vector3(0.0, 0.0, 0.0);
    attitude_ = Quaternion::identity();
    angular_velocity_ = Vector3(0.0, 0.0, 0.0);
    time_ = 0.0;
}

bool KiteState::isValid() const {
    return isPositionValid() && 
           isVelocityValid() && 
           isAttitudeValid() && 
           isAngularVelocityValid() &&
           std::isfinite(time_);
}

bool KiteState::isPositionValid() const {
    return std::isfinite(position_.x()) && 
           std::isfinite(position_.y()) && 
           std::isfinite(position_.z());
}

bool KiteState::isVelocityValid() const {
    return std::isfinite(velocity_.x()) && 
           std::isfinite(velocity_.y()) && 
           std::isfinite(velocity_.z());
}

bool KiteState::isAttitudeValid() const {
    return std::isfinite(attitude_.w()) && 
           std::isfinite(attitude_.x()) && 
           std::isfinite(attitude_.y()) && 
           std::isfinite(attitude_.z()) &&
           attitude_.isUnit();
}

bool KiteState::isAngularVelocityValid() const {
    return std::isfinite(angular_velocity_.x()) && 
           std::isfinite(angular_velocity_.y()) && 
           std::isfinite(angular_velocity_.z());
}

std::string KiteState::toJson() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    
    oss << "{\n";
    oss << "  \"position\": [" << position_.x() << ", " << position_.y() << ", " << position_.z() << "],\n";
    oss << "  \"velocity\": [" << velocity_.x() << ", " << velocity_.y() << ", " << velocity_.z() << "],\n";
    oss << "  \"attitude\": [" << attitude_.w() << ", " << attitude_.x() << ", " << attitude_.y() << ", " << attitude_.z() << "],\n";
    oss << "  \"angular_velocity\": [" << angular_velocity_.x() << ", " << angular_velocity_.y() << ", " << angular_velocity_.z() << "],\n";
    oss << "  \"time\": " << time_ << "\n";
    oss << "}";
    
    return oss.str();
}

bool KiteState::fromJson(const std::string& json_str) {
    // Simple JSON parsing for basic structure
    // In a production system, would use a proper JSON library
    
    // For now, implement basic parsing that handles the expected format
    // This is a minimal implementation for testing purposes
    
    // Reset to default state first
    reset();
    
    // Look for position array
    size_t pos_start = json_str.find("\"position\":");
    if (pos_start != std::string::npos) {
        size_t bracket_start = json_str.find("[", pos_start);
        size_t bracket_end = json_str.find("]", bracket_start);
        if (bracket_start != std::string::npos && bracket_end != std::string::npos) {
            std::string pos_str = json_str.substr(bracket_start + 1, bracket_end - bracket_start - 1);
            std::istringstream iss(pos_str);
            std::string token;
            std::vector<double> pos_vals;
            while (std::getline(iss, token, ',')) {
                pos_vals.push_back(std::stod(token));
            }
            if (pos_vals.size() == 3) {
                position_ = Vector3(pos_vals[0], pos_vals[1], pos_vals[2]);
            }
        }
    }
    
    return isValid();
}

bool KiteState::operator==(const KiteState& other) const {
    const double tolerance = 1e-9;
    
    return (std::abs(position_.x() - other.position_.x()) < tolerance) &&
           (std::abs(position_.y() - other.position_.y()) < tolerance) &&
           (std::abs(position_.z() - other.position_.z()) < tolerance) &&
           (std::abs(velocity_.x() - other.velocity_.x()) < tolerance) &&
           (std::abs(velocity_.y() - other.velocity_.y()) < tolerance) &&
           (std::abs(velocity_.z() - other.velocity_.z()) < tolerance) &&
           (std::abs(attitude_.w() - other.attitude_.w()) < tolerance) &&
           (std::abs(attitude_.x() - other.attitude_.x()) < tolerance) &&
           (std::abs(attitude_.y() - other.attitude_.y()) < tolerance) &&
           (std::abs(attitude_.z() - other.attitude_.z()) < tolerance) &&
           (std::abs(angular_velocity_.x() - other.angular_velocity_.x()) < tolerance) &&
           (std::abs(angular_velocity_.y() - other.angular_velocity_.y()) < tolerance) &&
           (std::abs(angular_velocity_.z() - other.angular_velocity_.z()) < tolerance) &&
           (std::abs(time_ - other.time_) < tolerance);
}

bool KiteState::operator!=(const KiteState& other) const {
    return !(*this == other);
}

} // namespace kite_sim