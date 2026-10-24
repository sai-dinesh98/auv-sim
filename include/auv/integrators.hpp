#pragma once
#include "auv/rigid_body.hpp"

namespace auv {
    State eulerStep(const RigidBody& body, const State& s, double dt);
    State rk4Step(const RigidBody& body, const State& s, double dt);
}