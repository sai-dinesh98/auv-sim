#pragma once
#include "auv/vec3.hpp"

namespace auv {

    struct State {
        Vec3 pos;
        Vec3 vel;
    };

    struct StateDerivative {
        Vec3 dpos;
        Vec3 dvel;
    };

    class RigidBody {

        public:
            RigidBody(double mass, double volume, double cdA);
            StateDerivative derivative(const State& s) const;
            double terminalSpeed() const;
            double mass() const {return mass_;}
        
        private:
            double mass_;
            double volume_;
            double cdA_;
    };

}