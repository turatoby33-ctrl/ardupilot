#pragma once

#include <cmath>

template <typename T>
class Vector3 {
public:
    T x, y, z;

    // Constructors
    Vector3() : x(0), y(0), z(0) {}
    Vector3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

    // Basic operations
    Vector3<T> operator+(const Vector3<T>& v) const {
        return Vector3<T>(x + v.x, y + v.y, z + v.z);
    }

    Vector3<T> operator-(const Vector3<T>& v) const {
        return Vector3<T>(x - v.x, y - v.y, z - v.z);
    }

    Vector3<T> operator*(T scalar) const {
        return Vector3<T>(x * scalar, y * scalar, z * scalar);
    }

    Vector3<T> operator/(T scalar) const {
        return Vector3<T>(x / scalar, y / scalar, z / scalar);
    }

    Vector3<T>& operator+=(const Vector3<T>& v) {
        x += v.x; y += v.y; z += v.z;
        return *this;
    }

    Vector3<T>& operator-=(const Vector3<T>& v) {
        x -= v.x; y -= v.y; z -= v.z;
        return *this;
    }

    Vector3<T>& operator*=(T scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    // Dot product
    T dot(const Vector3<T>& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // Cross product
    Vector3<T> cross(const Vector3<T>& v) const {
        return Vector3<T>(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    // Length
    T length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    T length_squared() const {
        return x * x + y * y + z * z;
    }

    // Normalize
    void normalize() {
        T len = length();
        if (len > 0) {
            *this /= len;
        }
    }

    Vector3<T> normalized() const {
        T len = length();
        if (len > 0) {
            return *this / len;
        }
        return *this;
    }

    // Zero check
    bool is_zero() const {
        return (x == 0 && y == 0 && z == 0);
    }

    // Zero the vector
    void zero() {
        x = y = z = 0;
    }
};

typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;
