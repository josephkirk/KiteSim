#include "kite_sim/wind_model.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace kite_sim {

// WindVector implementation
WindVector::WindVector() : velocity_(0.0, 0.0, 0.0) {}

WindVector::WindVector(const Vector3& velocity) : velocity_(velocity) {}

WindVector::WindVector(double u, double v, double w) : velocity_(u, v, w) {}

WindVector::WindVector(const Vector3& velocity, double speed, double direction) 
    : velocity_(velocity) {
    // Validate that the provided velocity matches speed and direction
    double computed_speed = velocity.magnitude();
    if (std::abs(computed_speed - speed) > 1e-6) {
        // Use the provided speed and direction instead
        setSpeedAndDirection(speed, direction);
    }
}

double WindVector::speed() const {
    return velocity_.magnitude();
}

double WindVector::direction() const {
    // Meteorological convention: direction wind is coming FROM
    // 0 = North, π/2 = East, π = South, 3π/2 = West
    double dir = std::atan2(velocity_.x(), velocity_.y());  // atan2(east, north)
    if (dir < 0) {
        dir += 2.0 * M_PI;  // Ensure positive angle
    }
    return dir;
}

void WindVector::setVelocity(const Vector3& velocity) {
    velocity_ = velocity;
}

void WindVector::setVelocity(double u, double v, double w) {
    velocity_ = Vector3(u, v, w);
}

void WindVector::setSpeedAndDirection(double speed, double direction) {
    if (speed < 0) {
        throw std::invalid_argument("Wind speed cannot be negative");
    }
    
    // Convert meteorological direction to velocity components
    // Direction is where wind comes FROM (meteorological convention)
    double u = speed * std::sin(direction);  // East component
    double v = speed * std::cos(direction);  // North component
    double w = 0.0;  // Assume no vertical component for horizontal wind
    
    velocity_ = Vector3(u, v, w);
}

bool WindVector::isCalm(double threshold) const {
    return speed() < threshold;
}

WindVector WindVector::rotated(double angle) const {
    // Rotate wind vector by angle (positive = counterclockwise)
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);
    
    Vector3 rotated_velocity(
        velocity_.x() * cos_a - velocity_.y() * sin_a,
        velocity_.x() * sin_a + velocity_.y() * cos_a,
        velocity_.z()  // Vertical component unchanged
    );
    
    return WindVector(rotated_velocity);
}

// WindShearParameters implementation
bool WindShearParameters::isValid() const {
    if (reference_height <= 0.0) return false;
    if (reference_speed < 0.0) return false;
    
    switch (profile) {
        case WindShearProfile::POWER_LAW:
            return power_law_exponent >= 0.0 && power_law_exponent <= 1.0;
            
        case WindShearProfile::LOGARITHMIC:
            return roughness_length > 0.0 && roughness_length < reference_height;
            
        case WindShearProfile::UNIFORM:
        default:
            return true;
    }
}

// WindModel base class implementation
WindVector WindModel::getWind(double x, double y, double z, double time) const {
    return getWind(Vector3(x, y, z), time);
}

void WindModel::setBaseWind(double speed, double direction) {
    // Base implementation - derived classes should override if needed
    shear_params_.reference_speed = speed;
    shear_params_.reference_direction = direction;
}

void WindModel::setWindShear(const WindShearParameters& params) {
    if (!params.isValid()) {
        throw std::invalid_argument("Invalid wind shear parameters");
    }
    shear_params_ = params;
}

WindVector WindModel::calculateWindShear(const Vector3& position, const WindVector& base_wind) const {
    if (shear_params_.profile == WindShearProfile::UNIFORM) {
        return base_wind;
    }
    
    double height = position.z();
    if (height <= 0.0) {
        // At or below ground level, assume minimal wind
        return WindVector(base_wind.velocity() * 0.1);
    }
    
    double shear_factor = calculateShearFactor(height);
    
    // Apply shear factor to horizontal wind components only
    Vector3 sheared_velocity = base_wind.velocity();
    sheared_velocity.x() *= shear_factor;
    sheared_velocity.y() *= shear_factor;
    // Keep vertical component unchanged
    
    return WindVector(sheared_velocity);
}

double WindModel::calculateShearFactor(double height) const {
    if (height <= 0.0) return 0.1;  // Minimal wind at ground level
    
    switch (shear_params_.profile) {
        case WindShearProfile::POWER_LAW: {
            double height_ratio = height / shear_params_.reference_height;
            return std::pow(height_ratio, shear_params_.power_law_exponent);
        }
        
        case WindShearProfile::LOGARITHMIC: {
            if (height <= shear_params_.roughness_length) {
                return 0.1;  // Minimal wind in roughness layer
            }
            
            double log_height = std::log(height / shear_params_.roughness_length);
            double log_ref = std::log(shear_params_.reference_height / shear_params_.roughness_length);
            
            if (log_ref <= 0.0) return 1.0;  // Avoid division by zero
            
            return log_height / log_ref;
        }
        
        case WindShearProfile::UNIFORM:
        default:
            return 1.0;
    }
}

// UniformWindModel implementation
UniformWindModel::UniformWindModel() : base_wind_(Vector3(10.0, 0.0, 0.0)) {
    // Default: 10 m/s wind from the west (positive x direction)
}

UniformWindModel::UniformWindModel(double speed, double direction) {
    setBaseWind(speed, direction);
}

UniformWindModel::UniformWindModel(const Vector3& wind_velocity) 
    : base_wind_(wind_velocity) {}

UniformWindModel::UniformWindModel(const WindVector& wind) 
    : base_wind_(wind) {}

WindVector UniformWindModel::getWind(const Vector3& position, double time) const {
    // Start with base wind
    WindVector wind = base_wind_;
    
    // Apply wind shear if configured
    if (hasWindShear()) {
        wind = calculateWindShear(position, wind);
    }
    
    return wind;
}

void UniformWindModel::setBaseWind(double speed, double direction) {
    WindModel::setBaseWind(speed, direction);  // Update shear parameters
    base_wind_.setSpeedAndDirection(speed, direction);
}

void UniformWindModel::setBaseWind(const Vector3& wind_velocity) {
    base_wind_.setVelocity(wind_velocity);
    
    // Update shear parameters to match
    shear_params_.reference_speed = base_wind_.speed();
    shear_params_.reference_direction = base_wind_.direction();
}

void UniformWindModel::setBaseWind(const WindVector& wind) {
    base_wind_ = wind;
    
    // Update shear parameters to match
    shear_params_.reference_speed = base_wind_.speed();
    shear_params_.reference_direction = base_wind_.direction();
}

// Factory functions
std::unique_ptr<WindModel> createUniformWindModel(double speed, double direction) {
    return std::make_unique<UniformWindModel>(speed, direction);
}

std::unique_ptr<WindModel> createUniformWindModel(const Vector3& wind_velocity) {
    return std::make_unique<UniformWindModel>(wind_velocity);
}

} // namespace kite_sim