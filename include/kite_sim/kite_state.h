#pragma once

#include "math_types.h"
#include <string>

namespace kite_sim {

/**
 * @brief Complete state representation of a kite at any time instant
 * 
 * Contains position, velocity, attitude, and angular velocity information
 * organized for efficient integration and state management.
 */
class KiteState {
public:
    // Constructors
    KiteState();
    KiteState(const Vector3& position, const Vector3& velocity, 
              const Quaternion& attitude, const Vector3& angular_velocity,
              double time = 0.0);
    
    // Copy constructor and assignment
    KiteState(const KiteState& other) = default;
    KiteState& operator=(const KiteState& other) = default;
    
    // Move constructor and assignment
    KiteState(KiteState&& other) noexcept = default;
    KiteState& operator=(KiteState&& other) noexcept = default;
    
    // Destructor
    ~KiteState() = default;
    
    // Position access (world coordinates)
    Vector3& position() { return position_; }
    const Vector3& position() const { return position_; }
    void setPosition(const Vector3& pos) { position_ = pos; }
    
    // Velocity access (world coordinates)
    Vector3& velocity() { return velocity_; }
    const Vector3& velocity() const { return velocity_; }
    void setVelocity(const Vector3& vel) { velocity_ = vel; }
    
    // Attitude access (quaternion representation)
    Quaternion& attitude() { return attitude_; }
    const Quaternion& attitude() const { return attitude_; }
    void setAttitude(const Quaternion& att) { attitude_ = att; }
    
    // Angular velocity access (body coordinates)
    Vector3& angularVelocity() { return angular_velocity_; }
    const Vector3& angularVelocity() const { return angular_velocity_; }
    void setAngularVelocity(const Vector3& ang_vel) { angular_velocity_ = ang_vel; }
    
    // Time access
    double time() const { return time_; }
    void setTime(double t) { time_ = t; }
    
    // State vector operations for integration
    void toStateVector(double* state_vector) const;
    void fromStateVector(const double* state_vector);
    static constexpr int getStateSize() { return 13; } // 3+3+4+3
    
    // Utility functions
    void reset();
    bool isValid() const;
    
    // Serialization
    std::string toJson() const;
    bool fromJson(const std::string& json_str);
    
    // Comparison operators
    bool operator==(const KiteState& other) const;
    bool operator!=(const KiteState& other) const;

private:
    Vector3 position_;        // Position in world coordinates (m)
    Vector3 velocity_;        // Velocity in world coordinates (m/s)
    Quaternion attitude_;     // Attitude quaternion (world to body)
    Vector3 angular_velocity_; // Angular velocity in body coordinates (rad/s)
    double time_;             // Time stamp (s)
    
    // Helper functions for validation
    bool isPositionValid() const;
    bool isVelocityValid() const;
    bool isAttitudeValid() const;
    bool isAngularVelocityValid() const;
};

} // namespace kite_sim