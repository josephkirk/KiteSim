#pragma once

namespace kite_sim {

/**
 * @brief Physical and mathematical constants used in kite simulation
 */
namespace constants {

// Mathematical constants
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double HALF_PI = PI / 2.0;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

// Physical constants
constexpr double GRAVITY = 9.80665;  // m/s^2 (standard gravity)
constexpr double AIR_DENSITY_SEA_LEVEL = 1.225;  // kg/m^3 (ISA standard)

// Numerical tolerances
constexpr double EPSILON = 1e-12;
constexpr double SMALL_ANGLE = 1e-6;  // radians
constexpr double QUATERNION_TOLERANCE = 1e-6;
constexpr double ZERO_VELOCITY_THRESHOLD = 1e-3;  // m/s

// Integration defaults
constexpr double DEFAULT_TIME_STEP = 0.01;  // seconds
constexpr double MIN_TIME_STEP = 1e-6;      // seconds
constexpr double MAX_TIME_STEP = 0.1;       // seconds

// Aerodynamic limits
constexpr double MAX_ANGLE_OF_ATTACK = 90.0 * DEG_TO_RAD;  // radians
constexpr double STALL_ANGLE_TYPICAL = 15.0 * DEG_TO_RAD;  // radians
constexpr double MAX_DYNAMIC_PRESSURE = 1000.0;  // Pa

} // namespace constants
} // namespace kite_sim