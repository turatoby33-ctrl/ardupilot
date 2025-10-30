#pragma once

#include <cstdint>
#include <cmath>

namespace CustomCopter {

// ============================================================================
// Vector3f - 3D floating point vector
// ============================================================================
struct Vector3f {
    float x, y, z;

    Vector3f() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // Vector operations
    float length() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    float length_squared() const {
        return x*x + y*y + z*z;
    }

    void normalize() {
        float len = length();
        if (len > 0.0f) {
            x /= len;
            y /= len;
            z /= len;
        }
    }

    Vector3f normalized() const {
        Vector3f result = *this;
        result.normalize();
        return result;
    }

    // Operators
    Vector3f operator+(const Vector3f& v) const {
        return Vector3f(x + v.x, y + v.y, z + v.z);
    }

    Vector3f operator-(const Vector3f& v) const {
        return Vector3f(x - v.x, y - v.y, z - v.z);
    }

    Vector3f operator*(float scalar) const {
        return Vector3f(x * scalar, y * scalar, z * scalar);
    }

    Vector3f operator/(float scalar) const {
        return Vector3f(x / scalar, y / scalar, z / scalar);
    }

    Vector3f& operator+=(const Vector3f& v) {
        x += v.x; y += v.y; z += v.z;
        return *this;
    }

    Vector3f& operator-=(const Vector3f& v) {
        x -= v.x; y -= v.y; z -= v.z;
        return *this;
    }

    float dot(const Vector3f& v) const {
        return x*v.x + y*v.y + z*v.z;
    }

    Vector3f cross(const Vector3f& v) const {
        return Vector3f(
            y*v.z - z*v.y,
            z*v.x - x*v.z,
            x*v.y - y*v.x
        );
    }
};

// ============================================================================
// Vector2f - 2D floating point vector
// ============================================================================
struct Vector2f {
    float x, y;

    Vector2f() : x(0.0f), y(0.0f) {}
    Vector2f(float x_, float y_) : x(x_), y(y_) {}

    float length() const {
        return std::sqrt(x*x + y*y);
    }

    float length_squared() const {
        return x*x + y*y;
    }

    void normalize() {
        float len = length();
        if (len > 0.0f) {
            x /= len;
            y /= len;
        }
    }

    Vector2f operator+(const Vector2f& v) const {
        return Vector2f(x + v.x, y + v.y);
    }

    Vector2f operator-(const Vector2f& v) const {
        return Vector2f(x - v.x, y - v.y);
    }

    Vector2f operator*(float scalar) const {
        return Vector2f(x * scalar, y * scalar);
    }
};

// ============================================================================
// Quaternion - Attitude representation
// ============================================================================
struct Quaternion {
    float w, x, y, z;

    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w_, float x_, float y_, float z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    void normalize() {
        float len = std::sqrt(w*w + x*x + y*y + z*z);
        if (len > 0.0f) {
            w /= len; x /= len; y /= len; z /= len;
        }
    }

    // Convert to Euler angles (roll, pitch, yaw in radians)
    void to_euler(float& roll, float& pitch, float& yaw) const {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        if (std::abs(sinp) >= 1.0f)
            pitch = std::copysign(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    // Create from Euler angles
    static Quaternion from_euler(float roll, float pitch, float yaw) {
        float cr = std::cos(roll * 0.5f);
        float sr = std::sin(roll * 0.5f);
        float cp = std::cos(pitch * 0.5f);
        float sp = std::sin(pitch * 0.5f);
        float cy = std::cos(yaw * 0.5f);
        float sy = std::sin(yaw * 0.5f);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }
};

// ============================================================================
// Location - GPS position
// ============================================================================
struct Location {
    int32_t lat;        // Latitude in 1e-7 degrees
    int32_t lon;        // Longitude in 1e-7 degrees
    int32_t alt;        // Altitude in centimeters (relative to home)

    Location() : lat(0), lon(0), alt(0) {}
    Location(int32_t lat_, int32_t lon_, int32_t alt_)
        : lat(lat_), lon(lon_), alt(alt_) {}

    // Get distance to another location in meters
    float distance_to(const Location& loc2) const {
        const float EARTH_RADIUS = 6371000.0f; // meters

        float lat1_rad = lat * 1e-7f * M_PI / 180.0f;
        float lat2_rad = loc2.lat * 1e-7f * M_PI / 180.0f;
        float dLat = (loc2.lat - lat) * 1e-7f * M_PI / 180.0f;
        float dLon = (loc2.lon - lon) * 1e-7f * M_PI / 180.0f;

        float a = std::sin(dLat/2) * std::sin(dLat/2) +
                  std::cos(lat1_rad) * std::cos(lat2_rad) *
                  std::sin(dLon/2) * std::sin(dLon/2);
        float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));

        return EARTH_RADIUS * c;
    }

    // Get bearing to another location in radians
    float bearing_to(const Location& loc2) const {
        float lat1_rad = lat * 1e-7f * M_PI / 180.0f;
        float lat2_rad = loc2.lat * 1e-7f * M_PI / 180.0f;
        float dLon = (loc2.lon - lon) * 1e-7f * M_PI / 180.0f;

        float y = std::sin(dLon) * std::cos(lat2_rad);
        float x = std::cos(lat1_rad) * std::sin(lat2_rad) -
                  std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dLon);

        return std::atan2(y, x);
    }
};

// ============================================================================
// Attitude - Roll, Pitch, Yaw in radians
// ============================================================================
struct Attitude {
    float roll;
    float pitch;
    float yaw;

    Attitude() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
    Attitude(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
};

// ============================================================================
// Motor Output - Roll, Pitch, Yaw, Throttle commands
// ============================================================================
struct MotorOutput {
    float roll;        // -1.0 to +1.0
    float pitch;       // -1.0 to +1.0
    float yaw;         // -1.0 to +1.0
    float throttle;    // 0.0 to 1.0

    MotorOutput() : roll(0.0f), pitch(0.0f), yaw(0.0f), throttle(0.0f) {}
};

// ============================================================================
// Vehicle State Flags
// ============================================================================
struct VehicleState {
    bool armed;
    bool land_complete;
    bool throttle_zero;
    bool new_radio_frame;
    bool position_ok;
    bool ekf_ok;

    VehicleState()
        : armed(false)
        , land_complete(true)
        , throttle_zero(true)
        , new_radio_frame(false)
        , position_ok(false)
        , ekf_ok(false)
    {}
};

// ============================================================================
// Sensor Data Structures
// ============================================================================
struct IMUData {
    Vector3f accel_ms2;      // Acceleration in m/s²
    Vector3f gyro_rads;      // Angular velocity in rad/s
    uint64_t timestamp_us;   // Timestamp in microseconds

    IMUData() : timestamp_us(0) {}
};

struct GPSData {
    Location location;
    Vector3f velocity_ms;    // Velocity in m/s (NED frame)
    uint8_t num_sats;
    uint8_t fix_type;        // 0=no fix, 2=2D, 3=3D, 4=DGPS, 5=RTK
    float hdop;              // Horizontal dilution of precision
    uint64_t timestamp_us;

    GPSData() : num_sats(0), fix_type(0), hdop(99.9f), timestamp_us(0) {}
};

struct BaroData {
    float altitude_m;        // Altitude in meters
    float pressure_pa;       // Pressure in Pascals
    float temperature_c;     // Temperature in Celsius
    uint64_t timestamp_us;

    BaroData() : altitude_m(0.0f), pressure_pa(0.0f),
                 temperature_c(0.0f), timestamp_us(0) {}
};

struct CompassData {
    Vector3f mag_field;      // Magnetic field in Gauss
    float heading_rad;       // Heading in radians
    uint64_t timestamp_us;

    CompassData() : heading_rad(0.0f), timestamp_us(0) {}
};

// ============================================================================
// Utility Functions
// ============================================================================
inline float constrain_float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline int32_t constrain_int32(int32_t value, int32_t min, int32_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline float wrap_PI(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

inline float wrap_360(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

inline float degrees(float radians) {
    return radians * 180.0f / M_PI;
}

inline float radians(float degrees) {
    return degrees * M_PI / 180.0f;
}

} // namespace CustomCopter
