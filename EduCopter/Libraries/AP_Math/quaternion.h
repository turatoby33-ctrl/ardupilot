#pragma once

#include "vector3.h"
#include <cmath>

class Quaternion {
public:
    float q1, q2, q3, q4;  // w, x, y, z

    // Constructors
    Quaternion() : q1(1.0f), q2(0.0f), q3(0.0f), q4(0.0f) {}
    Quaternion(float w, float x, float y, float z) : q1(w), q2(x), q3(y), q4(z) {}

    // Create from Euler angles (roll, pitch, yaw in radians)
    void from_euler(float roll, float pitch, float yaw) {
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);

        q1 = cr * cp * cy + sr * sp * sy;
        q2 = sr * cp * cy - cr * sp * sy;
        q3 = cr * sp * cy + sr * cp * sy;
        q4 = cr * cp * sy - sr * sp * cy;
    }

    // Convert to Euler angles (roll, pitch, yaw in radians)
    void to_euler(float& roll, float& pitch, float& yaw) const {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (q1 * q2 + q3 * q4);
        float cosr_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        roll = atan2f(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (q1 * q3 - q4 * q2);
        if (fabsf(sinp) >= 1.0f)
            pitch = copysignf(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
        else
            pitch = asinf(sinp);

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (q1 * q4 + q2 * q3);
        float cosy_cosp = 1.0f - 2.0f * (q3 * q3 + q4 * q4);
        yaw = atan2f(siny_cosp, cosy_cosp);
    }

    // Normalize
    void normalize() {
        float length = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        if (length > 0.0f) {
            q1 /= length;
            q2 /= length;
            q3 /= length;
            q4 /= length;
        }
    }

    // Quaternion multiplication
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            q1 * q.q1 - q2 * q.q2 - q3 * q.q3 - q4 * q.q4,
            q1 * q.q2 + q2 * q.q1 + q3 * q.q4 - q4 * q.q3,
            q1 * q.q3 - q2 * q.q4 + q3 * q.q1 + q4 * q.q2,
            q1 * q.q4 + q2 * q.q3 - q3 * q.q2 + q4 * q.q1
        );
    }

    // Rotate a vector
    Vector3f rotate(const Vector3f& v) const {
        Quaternion v_quat(0, v.x, v.y, v.z);
        Quaternion q_conj(q1, -q2, -q3, -q4);
        Quaternion result = (*this) * v_quat * q_conj;
        return Vector3f(result.q2, result.q3, result.q4);
    }

    // Get rotation matrix
    void to_rotation_matrix(float R[3][3]) const {
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;
        float q4q4 = q4 * q4;

        R[0][0] = q1q1 + q2q2 - q3q3 - q4q4;
        R[0][1] = 2.0f * (q2 * q3 - q1 * q4);
        R[0][2] = 2.0f * (q2 * q4 + q1 * q3);
        R[1][0] = 2.0f * (q2 * q3 + q1 * q4);
        R[1][1] = q1q1 - q2q2 + q3q3 - q4q4;
        R[1][2] = 2.0f * (q3 * q4 - q1 * q2);
        R[2][0] = 2.0f * (q2 * q4 - q1 * q3);
        R[2][1] = 2.0f * (q3 * q4 + q1 * q2);
        R[2][2] = q1q1 - q2q2 - q3q3 + q4q4;
    }
};
