#pragma once
#include <cmath>

namespace auv {

    struct Vec3 {
        double x{0.0};
        double y{0.0};
        double z{0.0};

        double norm() const {return std::sqrt(x * x + y * y + z * z); }

        Vec3& operator+=(const Vec3& other) {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }
    };

    inline Vec3 operator+(const Vec3& a, const Vec3& b) {return {a.x + b.x, a.y + b.y, a.z + b.z}; }
    inline Vec3 operator-(const Vec3& a, const Vec3& b) {return {a.x - b.x, a.y - b.y, a.z - b.z}; }
    inline Vec3 operator*(double s, const Vec3&v)       {return {s * v.x, s * v.y, s*v.z}; }
    inline Vec3 operator*(const Vec3& v, double s)      {return s*v; }
    inline Vec3 operator/(const Vec3& v, double s)      {return {v.x / s, v.y / s, v.z / s}; }
    inline double dot(const Vec3& a, const Vec3& b)     {return a.x * b.x + a.y * b.y + a.z * b.z; }
    
}