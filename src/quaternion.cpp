#include "auv/quaternion.hpp"

#include <cmath>

namespace auv {
    Quaternion Quaternion::fromAxisAngle(const Vec3& axis, double angleRad) {
        const Vec3 u = axis / axis.norm();
        const double s = std::sin(angleRad / 2.0);
        return {std::cos(angleRad / 2.0), s * u.x, s * u.y, s * u.z};
    }

    double Quaternion::norm() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    Quaternion Quaternion::normalized() const {
        const double n = norm();
        return {w / n, x / n, y / n, z / n};
    }

    Vec3 Quaternion::rotate(const Vec3& v) const {
        const Quaternion p{0.0, v.x, v.y, v.z};
        const Quaternion r = (*this) * p * conjugate();
        return {r.x, r.y, r.z};
    }

    double Quaternion::yaw() const {
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    Quaternion operator*(const Quaternion& a, const Quaternion& b) {
        return {
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        };
    }

    Quaternion operator+(const Quaternion& a, const Quaternion& b) {
        return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z};
    }

    Quaternion operator*(double s, const Quaternion& q){
        return {s * q.w, s * q.x, s * q.y, s * q.z};
    }
}