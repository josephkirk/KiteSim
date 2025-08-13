#pragma once

#include "kite_sim/math_types.h"
#include <memory>

namespace kite_sim {

/**
 * @brief Wind velocity vector with additional metadata
 * 
 * Represents wind velocity at a specific position and time with
 * additional information about wind characteristics.
 */
class WindVector {
public:
    // Constructors
    WindVector();
    WindVector(const Vector3& velocity);
    WindVector(double u, double v, double w);
    WindVector(const Vector3& velocity, double speed, double direction);
    
    // Accessors
    const Vector3& velocity() const { return velocity_; }
    Vector3& velocity() { return velocity_; }
    
    double speed() const;
    double direction() const;  // Wind direction in radians (meteorological convention)
    
    // Component access
    double u() const { return velocity_.x(); }  // East-West component
    double v() const { return velocity_.y(); }  // North-South component  
    double w() const { return velocity_.z(); }  // Vertical component
    
    // Setters
    void setVelocity(const Vector3& velocity);
    void setVelocity(double u, double v, double w);
    void setSpeedAndDirection(double speed, double direction);
    
    // Utility
    bool isCalm(double threshold = 0.1) const;
    WindVector rotated(double angle) const;

private:
    Vector3 velocity_;  // Wind velocity vector [m/s]
    
    void updateFromVelocity();
};

/**
 * @brief Wind shear profile types
 */
enum class WindShearProfile {
    UNIFORM,        // No wind shear
    POWER_LAW,      // Power law profile: V(z) = V_ref * (z/z_ref)^alpha
    LOGARITHMIC     // Logarithmic profile: V(z) = V_ref * ln(z/z0) / ln(z_ref/z0)
};

/**
 * @brief Parameters for wind shear modeling
 */
struct WindShearParameters {
    WindShearProfile profile = WindShearProfile::UNIFORM;
    double reference_height = 10.0;    // Reference height [m]
    double reference_speed = 10.0;     // Wind speed at reference height [m/s]
    double reference_direction = 0.0;  // Wind direction at reference height [rad]
    
    // Power law parameters
    double power_law_exponent = 0.143; // Typical value for open terrain
    
    // Logarithmic profile parameters  
    double roughness_length = 0.03;    // Surface roughness length [m]
    
    // Validation
    bool isValid() const;
};

/**
 * @brief Base class for wind modeling
 * 
 * Provides interface for different wind models including uniform wind,
 * wind shear profiles, and potentially turbulence modeling.
 */
class WindModel {
public:
    WindModel() = default;
    virtual ~WindModel() = default;
    
    // Pure virtual interface
    virtual WindVector getWind(const Vector3& position, double time = 0.0) const = 0;
    virtual WindVector getWind(double x, double y, double z, double time = 0.0) const;
    
    // Configuration
    virtual void setBaseWind(double speed, double direction);
    virtual void setWindShear(const WindShearParameters& params);
    
    // Utility
    virtual bool hasWindShear() const { return shear_params_.profile != WindShearProfile::UNIFORM; }
    virtual bool hasTimeVariation() const { return false; }
    
protected:
    WindShearParameters shear_params_;
    
    // Helper functions for derived classes
    WindVector calculateWindShear(const Vector3& position, const WindVector& base_wind) const;
    double calculateShearFactor(double height) const;
};

/**
 * @brief Uniform wind field implementation
 * 
 * Provides constant wind velocity with optional wind shear profiles.
 * This is the most basic wind model suitable for initial testing and
 * simple simulations.
 */
class UniformWindModel : public WindModel {
public:
    // Constructors
    UniformWindModel();
    UniformWindModel(double speed, double direction);
    UniformWindModel(const Vector3& wind_velocity);
    UniformWindModel(const WindVector& wind);
    
    // WindModel interface implementation
    WindVector getWind(const Vector3& position, double time = 0.0) const override;
    
    // Configuration
    void setBaseWind(double speed, double direction) override;
    void setBaseWind(const Vector3& wind_velocity);
    void setBaseWind(const WindVector& wind);
    
    // Accessors
    const WindVector& getBaseWind() const { return base_wind_; }
    
private:
    WindVector base_wind_;  // Base wind vector (typically at reference height)
};

/**
 * @brief Factory function for creating wind models
 */
std::unique_ptr<WindModel> createUniformWindModel(double speed = 10.0, double direction = 0.0);
std::unique_ptr<WindModel> createUniformWindModel(const Vector3& wind_velocity);

} // namespace kite_sim