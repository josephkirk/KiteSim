#pragma once

#include "math_types.h"

namespace kite_sim {

/**
 * @brief Coordinate transformation utilities
 * 
 * Provides functions for transforming between different coordinate frames
 * commonly used in kite simulation:
 * - World frame (NED: North-East-Down)
 * - Body frame (kite-fixed frame)
 * - Wind frame (aligned with relative wind)
 */
namespace transforms {

/**
 * @brief Transform vector from body frame to world frame
 * @param bodyVector Vector in body coordinates
 * @param attitude Quaternion representing body attitude in world frame
 * @return Vector in world coordinates
 */
Vector3 bodyToWorld(const Vector3& bodyVector, const Quaternion& attitude);

/**
 * @brief Transform vector from world frame to body frame
 * @param worldVector Vector in world coordinates
 * @param attitude Quaternion representing body attitude in world frame
 * @return Vector in body coordinates
 */
Vector3 worldToBody(const Vector3& worldVector, const Quaternion& attitude);

/**
 * @brief Calculate rotation matrix from body to world frame
 * @param attitude Quaternion representing body attitude
 * @return 3x3 rotation matrix for body-to-world transformation
 */
Matrix3x3 bodyToWorldMatrix(const Quaternion& attitude);

/**
 * @brief Calculate rotation matrix from world to body frame
 * @param attitude Quaternion representing body attitude
 * @return 3x3 rotation matrix for world-to-body transformation
 */
Matrix3x3 worldToBodyMatrix(const Quaternion& attitude);

/**
 * @brief Calculate angle of attack and sideslip from relative wind
 * @param relativeWind Wind velocity relative to kite in body frame
 * @param alpha Output angle of attack (radians)
 * @param beta Output sideslip angle (radians)
 */
void calculateAirflowAngles(const Vector3& relativeWind, double& alpha, double& beta);

/**
 * @brief Calculate relative wind vector in body frame
 * @param kiteVelocity Kite velocity in world frame
 * @param windVelocity Wind velocity in world frame
 * @param attitude Kite attitude quaternion
 * @return Relative wind vector in body frame
 */
Vector3 calculateRelativeWind(const Vector3& kiteVelocity, 
                             const Vector3& windVelocity,
                             const Quaternion& attitude);

/**
 * @brief Convert Euler angles to quaternion
 * @param roll Roll angle (radians)
 * @param pitch Pitch angle (radians) 
 * @param yaw Yaw angle (radians)
 * @return Quaternion representation
 */
Quaternion eulerToQuaternion(double roll, double pitch, double yaw);

/**
 * @brief Convert quaternion to Euler angles
 * @param q Quaternion
 * @param roll Output roll angle (radians)
 * @param pitch Output pitch angle (radians)
 * @param yaw Output yaw angle (radians)
 */
void quaternionToEuler(const Quaternion& q, double& roll, double& pitch, double& yaw);

/**
 * @brief Calculate angular velocity transformation matrix
 * 
 * Transforms angular velocity from Euler angle rates to body frame angular velocity
 * @param roll Current roll angle (radians)
 * @param pitch Current pitch angle (radians)
 * @return Transformation matrix
 */
Matrix3x3 eulerRateToBodyRate(double roll, double pitch);

/**
 * @brief Calculate quaternion derivative from angular velocity
 * @param q Current quaternion
 * @param omega Angular velocity in body frame (rad/s)
 * @return Quaternion derivative
 */
Quaternion quaternionDerivative(const Quaternion& q, const Vector3& omega);

} // namespace transforms
} // namespace kite_sim