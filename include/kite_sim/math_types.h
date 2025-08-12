#pragma once

namespace kite_sim {

/**
 * @brief 3D Vector interface - to be implemented by math library wrapper
 * 
 * This interface defines the expected operations for 3D vectors.
 * Implementation will wrap an external math library (e.g., Eigen, GLM, etc.)
 */
class Vector3 {
public:
    // Constructors
    Vector3() : data_{0.0, 0.0, 0.0} {}
    Vector3(double x, double y, double z);
    
    // Element access
    double& operator[](int index);
    const double& operator[](int index) const;
    
    double& x();
    double& y(); 
    double& z();
    const double& x() const;
    const double& y() const;
    const double& z() const;
    
    // Vector operations
    Vector3 operator+(const Vector3& other) const;
    Vector3 operator-(const Vector3& other) const;
    Vector3 operator*(double scalar) const;
    Vector3 operator/(double scalar) const;
    Vector3& operator+=(const Vector3& other);
    Vector3& operator-=(const Vector3& other);
    Vector3& operator*=(double scalar);
    Vector3& operator/=(double scalar);
    
    // Vector products
    double dot(const Vector3& other) const;
    Vector3 cross(const Vector3& other) const;
    
    // Magnitude operations
    double magnitude() const;
    double magnitudeSquared() const;
    Vector3 normalized() const;
    void normalize();
    
    // Utility
    bool isZero(double tolerance = 1e-10) const;

private:
    double data_[3];
};

/**
 * @brief Quaternion interface for attitude representation
 * 
 * Quaternion for rotation representation with w + xi + yj + zk convention
 */
class Quaternion {
public:
    // Constructors
    Quaternion() : data_{1.0, 0.0, 0.0, 0.0} {}
    Quaternion(double w, double x, double y, double z);
    
    // Static constructors
    static Quaternion identity();
    static Quaternion fromAxisAngle(const Vector3& axis, double angle);
    static Quaternion fromEulerAngles(double roll, double pitch, double yaw);
    
    // Element access
    double& w();
    double& x();
    double& y();
    double& z();
    const double& w() const;
    const double& x() const;
    const double& y() const;
    const double& z() const;
    
    // Quaternion operations
    Quaternion operator*(const Quaternion& other) const;
    Quaternion& operator*=(const Quaternion& other);
    Quaternion conjugate() const;
    Quaternion inverse() const;
    
    // Magnitude operations
    double magnitude() const;
    double magnitudeSquared() const;
    Quaternion normalized() const;
    void normalize();
    
    // Rotation operations
    Vector3 rotate(const Vector3& vector) const;
    Vector3 toEulerAngles() const;
    
    // Utility
    bool isUnit(double tolerance = 1e-6) const;

private:
    double data_[4]; // w, x, y, z
};

/**
 * @brief 3x3 Matrix interface for rotations and transformations
 */
class Matrix3x3 {
public:
    // Constructors
    Matrix3x3() : data_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} {}
    Matrix3x3(const double data[9]);
    
    // Static constructors
    static Matrix3x3 identity();
    static Matrix3x3 zero();
    static Matrix3x3 fromQuaternion(const Quaternion& q);
    static Matrix3x3 skewSymmetric(const Vector3& v);
    
    // Element access
    double& operator()(int row, int col);
    const double& operator()(int row, int col) const;
    
    // Matrix operations
    Matrix3x3 operator+(const Matrix3x3& other) const;
    Matrix3x3 operator-(const Matrix3x3& other) const;
    Matrix3x3 operator*(const Matrix3x3& other) const;
    Matrix3x3 operator*(double scalar) const;
    Vector3 operator*(const Vector3& vector) const;
    
    Matrix3x3& operator+=(const Matrix3x3& other);
    Matrix3x3& operator-=(const Matrix3x3& other);
    Matrix3x3& operator*=(const Matrix3x3& other);
    Matrix3x3& operator*=(double scalar);
    
    // Matrix properties
    Matrix3x3 transpose() const;
    double determinant() const;
    Matrix3x3 inverse() const;
    
    // Utility
    bool isOrthogonal(double tolerance = 1e-6) const;

private:
    double data_[9]; // Row-major order
};

// Free functions for scalar multiplication
Vector3 operator*(double scalar, const Vector3& vector);
Matrix3x3 operator*(double scalar, const Matrix3x3& matrix);

} // namespace kite_sim