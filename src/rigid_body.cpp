#include "auv/rigid_body.hpp"
#include "auv/constants.hpp"

#include <cmath>
#include <stdexcept>

namespace auv {

    using constants::kGravity;
    using constants::kRhoSeawater;

    RigidBody::RigidBody(double mass, double volume, double cdA, const Vec3& inertia)
        : mass_{mass}, volume_{volume}, cdA_{cdA}, inertia_{inertia} {
            if (mass_ <= 0.0 || volume_ <= 0.0 || cdA_ < 0.0) {
                throw std::invalid_argument("RigidBody: mass and volume must be > 0, cdA >= 0");
            }
            const double ix = inertia_.x;
            const double iy = inertia_.y;
            const double iz = inertia_.z;
            if (ix <= 0 || iy <= 0 || iz <= 0){
                throw std::invalid_argument("RigidBody: Principal inertias must be ? 0");
            }
            if (ix + iy < iz || iy + iz < ix || iz + ix < iy) {
                throw std::invalid_argument("RigidBody : inertias voilate the triangle inequality");
            }
        }

    StateDerivative RigidBody::derivative(const State& s) const {
        const Vec3 weight{0.0, 0.0, -mass_ * kGravity};
        const Vec3 buoyancy{0.0, 0.0, kRhoSeawater * volume_ * kGravity};
        const Vec3 drag = -0.5 * kRhoSeawater * cdA_ * s.vel.norm() * s.vel;
        const Quaternion omegaQ{0.0, s.omega.x, s.omega.y, s.omega.z};
        const Quaternion attDot = 0.5 * (s.att * omegaQ); // Attitude kinematics: q_dot = 0.5 * q ⊗ (0, ω)

        const Vec3 torque{};
        const Vec3 gyroscopic = cross(s.omega, cwiseProduct(inertia_, s.omega));
        const Vec3 omegaDot = cwiseQuotient(torque - gyroscopic, inertia_);

        return {s.vel, (weight + buoyancy + drag) / mass_, attDot, omegaDot};
    }

    double RigidBody::rotationalEnergy(const State& s) const {
        return 0.5 * dot(s.omega, cwiseProduct(inertia_, s.omega));
    }

    Vec3 RigidBody::angularMomentumWorld(const State& s) const {
        return s.att.rotate(cwiseProduct(inertia_, s.omega));
    }

    double RigidBody::terminalSpeed() const {
        const double netweight = (mass_ - kRhoSeawater * volume_) * kGravity;
        return std::sqrt(2.0 * std::abs(netweight) / (kRhoSeawater * cdA_));
    }
}