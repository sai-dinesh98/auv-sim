#pragma once
#include "auv/vec3.hpp"
#include "auv/quaternion.hpp"

namespace auv {

    struct State {
        Vec3 pos;
        Vec3 vel;
        Quaternion att;
        Vec3 omega;
    };

    struct StateDerivative {
        Vec3 dpos;
        Vec3 dvel;
        Quaternion datt;
        Vec3 domega;
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