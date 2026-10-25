#pragma once
#include "auv/vec3.hpp"

namespace auv {
    struct Quaternion {
        double w{1.0};
        double x{0.0};
        double y{0.0};
        double z{0.0};

        static Quaternion fromAxisAngle(const Vec3& axis, double angleRad);

        double norm() const;
        Quaternion normalized() const;
        Quaternion conjugate() const {return {w, -x, -y, -z}; }
        Vec3 rotate(const Vec3& v) const;
        double yaw() const;
    };

    Quaternion operator*(const Quaternion& a, const Quaternion& b);
    Quaternion operator+(const Quaternion& a, const Quaternion& b);
    Quaternion operator*(double s, const Quaternion& q);

}