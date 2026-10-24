#include "auv/rigid_body.hpp"
#include "auv/constants.hpp"

#include <cmath>
#include <stdexcept>

namespace auv {

    using constants::kGravity;
    using constants::kRhoSeawater;

    RigidBody::RigidBody(double mass, double volume, double cdA)
        : mass_{mass}, volume_{volume}, cdA_{cdA} {
            if (mass_ <= 0.0 || volume_ <= 0.0 || cdA_ < 0.0) {
                throw std::invalid_argument("RigidBody: mass and volume must be > 0, cdA >= 0");
            }
        }

    StateDerivative RigidBody::derivative(const State& s) const {
        const Vec3 weight{0.0, 0.0, -mass_ * kGravity};
        const Vec3 buoyancy{0.0, 0.0, kRhoSeawater * volume_ * kGravity};
        const Vec3 drag = -0.5 * kRhoSeawater * cdA_ * s.vel.norm() * s.vel;
        return {s.vel, (weight + buoyancy + drag) / mass_};
    }

    double RigidBody::terminalSpeed() const {
        const double netweight = (mass_ - kRhoSeawater * volume_) * kGravity;
        return std::sqrt(2.0 * std::abs(netweight) / (kRhoSeawater * cdA_));
    }
}