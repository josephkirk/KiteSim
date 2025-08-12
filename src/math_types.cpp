#include "kite_sim/math_types.h"
#include <cmath>
#include <stdexcept>
#include <numeric>

namespace kite_sim {

// Vector3 implementation
Vector3::Vector3(double x, double y, double z) {
    data_[0] = x;
    data_[1] = y;
    data_[2] = z;
}

double& Vector3::operator[](int index) {
    if (index < 0 || index > 2) throw std::out_of_range("Vector3 index out of range");
    return data_[index];
}

const double& Vector3::operator[](int index) const {
    if (index < 0 || index > 2) throw std::out_of_range("Vector3 index out of range");
    return data_[index];
}

double& Vector3::x() { return data_[0]; }
double& Vector3::y() { return data_[1]; }
double& Vector3::z() { return data_[2]; }
const double& Vector3::x() const { return data_[0]; }
const double& Vector3::y() const { return data_[1]; }
const double& Vector3::z() const { return data_[2]; }

Vector3 Vector3::operator+(const Vector3& other) const {
    return Vector3(data_[0] + other.data_[0], data_[1] + other.data_[1], data_[2] + other.data_[2]);
}

Vector3 Vector3::operator-(const Vector3& other) const {
    return Vector3(data_[0] - other.data_[0], data_[1] - other.data_[1], data_[2] - other.data_[2]);
}

Vector3 Vector3::operator*(double scalar) const {
    return Vector3(data_[0] * scalar, data_[1] * scalar, data_[2] * scalar);
}

Vector3 Vector3::operator/(double scalar) const {
    if (std::abs(scalar) < 1e-15) throw std::runtime_error("Division by zero");
    return Vector3(data_[0] / scalar, data_[1] / scalar, data_[2] / scalar);
}

Vector3& Vector3::operator+=(const Vector3& other) {
    data_[0] += other.data_[0];
    data_[1] += other.data_[1];
    data_[2] += other.data_[2];
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& other) {
    data_[0] -= other.data_[0];
    data_[1] -= other.data_[1];
    data_[2] -= other.data_[2];
    return *this;
}

Vector3& Vector3::operator*=(double scalar) {
    data_[0] *= scalar;
    data_[1] *= scalar;
    data_[2] *= scalar;
    return *this;
}

Vector3& Vector3::operator/=(double scalar) {
    if (std::abs(scalar) < 1e-15) throw std::runtime_error("Division by zero");
    data_[0] /= scalar;
    data_[1] /= scalar;
    data_[2] /= scalar;
    return *this;
}

double Vector3::dot(const Vector3& other) const {
    return data_[0] * other.data_[0] + data_[1] * other.data_[1] + data_[2] * other.data_[2];
}

Vector3 Vector3::cross(const Vector3& other) const {
    return Vector3(
        data_[1] * other.data_[2] - data_[2] * other.data_[1],
        data_[2] * other.data_[0] - data_[0] * other.data_[2],
        data_[0] * other.data_[1] - data_[1] * other.data_[0]
    );
}

double Vector3::magnitude() const {
    return std::sqrt(magnitudeSquared());
}

double Vector3::magnitudeSquared() const {
    return data_[0] * data_[0] + data_[1] * data_[1] + data_[2] * data_[2];
}

Vector3 Vector3::normalized() const {
    double mag = magnitude();
    if (mag < 1e-15) return Vector3(0, 0, 0);
    return *this / mag;
}

void Vector3::normalize() {
    double mag = magnitude();
    if (mag > 1e-15) {
        *this /= mag;
    }
}

bool Vector3::isZero(double tolerance) const {
    return magnitude() < tolerance;
}

// Quaternion implementation
Quaternion::Quaternion(double w, double x, double y, double z) {
    data_[0] = w;
    data_[1] = x;
    data_[2] = y;
    data_[3] = z;
}

Quaternion Quaternion::identity() {
    return Quaternion(1.0, 0.0, 0.0, 0.0);
}

Quaternion Quaternion::fromAxisAngle(const Vector3& axis, double angle) {
    Vector3 normalized_axis = axis.normalized();
    double half_angle = angle * 0.5;
    double sin_half = std::sin(half_angle);
    double cos_half = std::cos(half_angle);
    
    return Quaternion(cos_half, 
                     normalized_axis.x() * sin_half,
                     normalized_axis.y() * sin_half,
                     normalized_axis.z() * sin_half);
}

Quaternion Quaternion::fromEulerAngles(double roll, double pitch, double yaw) {
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    
    return Quaternion(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    );
}

double& Quaternion::w() { return data_[0]; }
double& Quaternion::x() { return data_[1]; }
double& Quaternion::y() { return data_[2]; }
double& Quaternion::z() { return data_[3]; }
const double& Quaternion::w() const { return data_[0]; }
const double& Quaternion::x() const { return data_[1]; }
const double& Quaternion::y() const { return data_[2]; }
const double& Quaternion::z() const { return data_[3]; }

Quaternion Quaternion::operator*(const Quaternion& other) const {
    return Quaternion(
        data_[0] * other.data_[0] - data_[1] * other.data_[1] - data_[2] * other.data_[2] - data_[3] * other.data_[3],
        data_[0] * other.data_[1] + data_[1] * other.data_[0] + data_[2] * other.data_[3] - data_[3] * other.data_[2],
        data_[0] * other.data_[2] - data_[1] * other.data_[3] + data_[2] * other.data_[0] + data_[3] * other.data_[1],
        data_[0] * other.data_[3] + data_[1] * other.data_[2] - data_[2] * other.data_[1] + data_[3] * other.data_[0]
    );
}

Quaternion& Quaternion::operator*=(const Quaternion& other) {
    *this = *this * other;
    return *this;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(data_[0], -data_[1], -data_[2], -data_[3]);
}

Quaternion Quaternion::inverse() const {
    double mag_sq = magnitudeSquared();
    if (mag_sq < 1e-15) throw std::runtime_error("Cannot invert zero quaternion");
    
    Quaternion conj = conjugate();
    return Quaternion(conj.data_[0] / mag_sq, conj.data_[1] / mag_sq, 
                     conj.data_[2] / mag_sq, conj.data_[3] / mag_sq);
}

double Quaternion::magnitude() const {
    return std::sqrt(magnitudeSquared());
}

double Quaternion::magnitudeSquared() const {
    return data_[0] * data_[0] + data_[1] * data_[1] + data_[2] * data_[2] + data_[3] * data_[3];
}

Quaternion Quaternion::normalized() const {
    double mag = magnitude();
    if (mag < 1e-15) return identity();
    return Quaternion(data_[0] / mag, data_[1] / mag, data_[2] / mag, data_[3] / mag);
}

void Quaternion::normalize() {
    double mag = magnitude();
    if (mag > 1e-15) {
        data_[0] /= mag;
        data_[1] /= mag;
        data_[2] /= mag;
        data_[3] /= mag;
    } else {
        *this = identity();
    }
}

Vector3 Quaternion::rotate(const Vector3& vector) const {
    // q * v * q^-1 where v is treated as pure quaternion (0, vx, vy, vz)
    Quaternion v_quat(0, vector.x(), vector.y(), vector.z());
    Quaternion result = (*this) * v_quat * conjugate();
    return Vector3(result.x(), result.y(), result.z());
}

Vector3 Quaternion::toEulerAngles() const {
    // Convert to roll, pitch, yaw
    double roll = std::atan2(2 * (data_[0] * data_[1] + data_[2] * data_[3]),
                            1 - 2 * (data_[1] * data_[1] + data_[2] * data_[2]));
    
    double sin_pitch = 2 * (data_[0] * data_[2] - data_[3] * data_[1]);
    double pitch = std::abs(sin_pitch) >= 1 ? std::copysign(M_PI / 2, sin_pitch) : std::asin(sin_pitch);
    
    double yaw = std::atan2(2 * (data_[0] * data_[3] + data_[1] * data_[2]),
                           1 - 2 * (data_[2] * data_[2] + data_[3] * data_[3]));
    
    return Vector3(roll, pitch, yaw);
}

bool Quaternion::isUnit(double tolerance) const {
    return std::abs(magnitude() - 1.0) < tolerance;
}

// Matrix3x3 implementation
Matrix3x3::Matrix3x3(const double data[9]) {
    for (int i = 0; i < 9; ++i) {
        data_[i] = data[i];
    }
}

Matrix3x3 Matrix3x3::identity() {
    Matrix3x3 result;
    for (int i = 0; i < 9; ++i) result.data_[i] = 0.0;
    result.data_[0] = result.data_[4] = result.data_[8] = 1.0;
    return result;
}

Matrix3x3 Matrix3x3::zero() {
    Matrix3x3 result;
    for (int i = 0; i < 9; ++i) result.data_[i] = 0.0;
    return result;
}

Matrix3x3 Matrix3x3::fromQuaternion(const Quaternion& q) {
    double w = q.w(), x = q.x(), y = q.y(), z = q.z();
    
    Matrix3x3 result;
    result.data_[0] = 1 - 2*y*y - 2*z*z;  result.data_[1] = 2*x*y - 2*w*z;      result.data_[2] = 2*x*z + 2*w*y;
    result.data_[3] = 2*x*y + 2*w*z;      result.data_[4] = 1 - 2*x*x - 2*z*z;  result.data_[5] = 2*y*z - 2*w*x;
    result.data_[6] = 2*x*z - 2*w*y;      result.data_[7] = 2*y*z + 2*w*x;      result.data_[8] = 1 - 2*x*x - 2*y*y;
    
    return result;
}

Matrix3x3 Matrix3x3::skewSymmetric(const Vector3& v) {
    Matrix3x3 result;
    result.data_[0] = 0;       result.data_[1] = -v.z();  result.data_[2] = v.y();
    result.data_[3] = v.z();   result.data_[4] = 0;       result.data_[5] = -v.x();
    result.data_[6] = -v.y();  result.data_[7] = v.x();   result.data_[8] = 0;
    return result;
}

double& Matrix3x3::operator()(int row, int col) {
    if (row < 0 || row > 2 || col < 0 || col > 2) {
        throw std::out_of_range("Matrix3x3 index out of range");
    }
    return data_[row * 3 + col];
}

const double& Matrix3x3::operator()(int row, int col) const {
    if (row < 0 || row > 2 || col < 0 || col > 2) {
        throw std::out_of_range("Matrix3x3 index out of range");
    }
    return data_[row * 3 + col];
}

Matrix3x3 Matrix3x3::operator+(const Matrix3x3& other) const {
    Matrix3x3 result;
    for (int i = 0; i < 9; ++i) {
        result.data_[i] = data_[i] + other.data_[i];
    }
    return result;
}

Matrix3x3 Matrix3x3::operator-(const Matrix3x3& other) const {
    Matrix3x3 result;
    for (int i = 0; i < 9; ++i) {
        result.data_[i] = data_[i] - other.data_[i];
    }
    return result;
}

Matrix3x3 Matrix3x3::operator*(const Matrix3x3& other) const {
    Matrix3x3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result(i, j) = 0;
            for (int k = 0; k < 3; ++k) {
                result(i, j) += (*this)(i, k) * other(k, j);
            }
        }
    }
    return result;
}

Matrix3x3 Matrix3x3::operator*(double scalar) const {
    Matrix3x3 result;
    for (int i = 0; i < 9; ++i) {
        result.data_[i] = data_[i] * scalar;
    }
    return result;
}

Vector3 Matrix3x3::operator*(const Vector3& vector) const {
    return Vector3(
        data_[0] * vector.x() + data_[1] * vector.y() + data_[2] * vector.z(),
        data_[3] * vector.x() + data_[4] * vector.y() + data_[5] * vector.z(),
        data_[6] * vector.x() + data_[7] * vector.y() + data_[8] * vector.z()
    );
}

Matrix3x3& Matrix3x3::operator+=(const Matrix3x3& other) {
    for (int i = 0; i < 9; ++i) {
        data_[i] += other.data_[i];
    }
    return *this;
}

Matrix3x3& Matrix3x3::operator-=(const Matrix3x3& other) {
    for (int i = 0; i < 9; ++i) {
        data_[i] -= other.data_[i];
    }
    return *this;
}

Matrix3x3& Matrix3x3::operator*=(const Matrix3x3& other) {
    *this = *this * other;
    return *this;
}

Matrix3x3& Matrix3x3::operator*=(double scalar) {
    for (int i = 0; i < 9; ++i) {
        data_[i] *= scalar;
    }
    return *this;
}

Matrix3x3 Matrix3x3::transpose() const {
    Matrix3x3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result(j, i) = (*this)(i, j);
        }
    }
    return result;
}

double Matrix3x3::determinant() const {
    return data_[0] * (data_[4] * data_[8] - data_[5] * data_[7]) -
           data_[1] * (data_[3] * data_[8] - data_[5] * data_[6]) +
           data_[2] * (data_[3] * data_[7] - data_[4] * data_[6]);
}

Matrix3x3 Matrix3x3::inverse() const {
    double det = determinant();
    if (std::abs(det) < 1e-15) {
        throw std::runtime_error("Matrix is not invertible");
    }
    
    Matrix3x3 result;
    result.data_[0] = (data_[4] * data_[8] - data_[5] * data_[7]) / det;
    result.data_[1] = (data_[2] * data_[7] - data_[1] * data_[8]) / det;
    result.data_[2] = (data_[1] * data_[5] - data_[2] * data_[4]) / det;
    result.data_[3] = (data_[5] * data_[6] - data_[3] * data_[8]) / det;
    result.data_[4] = (data_[0] * data_[8] - data_[2] * data_[6]) / det;
    result.data_[5] = (data_[2] * data_[3] - data_[0] * data_[5]) / det;
    result.data_[6] = (data_[3] * data_[7] - data_[4] * data_[6]) / det;
    result.data_[7] = (data_[1] * data_[6] - data_[0] * data_[7]) / det;
    result.data_[8] = (data_[0] * data_[4] - data_[1] * data_[3]) / det;
    
    return result;
}

bool Matrix3x3::isOrthogonal(double tolerance) const {
    Matrix3x3 product = (*this) * transpose();
    Matrix3x3 identity_mat = identity();
    
    for (int i = 0; i < 9; ++i) {
        if (std::abs(product.data_[i] - identity_mat.data_[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

// Free functions
Vector3 operator*(double scalar, const Vector3& vector) {
    return vector * scalar;
}

Matrix3x3 operator*(double scalar, const Matrix3x3& matrix) {
    return matrix * scalar;
}

} // namespace kite_sim