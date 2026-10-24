#include "auv/integrators.hpp"

namespace auv {

    namespace{
        State advance(const State& s, const StateDerivative& d, double h) {
            return {s.pos + h * d.dpos, s.vel + h * d.dvel};
        }
    }

    State eulerStep(const RigidBody& body, const State& s, double dt){
        return advance(s, body.derivative(s), dt);
    }

    State rk4Step(const RigidBody& body, const State& s , double dt) {
        const StateDerivative k1 = body.derivative(s);
        const StateDerivative k2 = body.derivative(advance(s, k1, dt/2.0));
        const StateDerivative k3 = body.derivative(advance(s, k2, dt/2.0));
        const StateDerivative k4 = body.derivative(advance(s, k3, dt));

        const StateDerivative weighted{
            (k1.dpos + 2.0 * k2.dpos + 2.0 * k3.dpos + k4.dpos) / 6.0,
            (k1.dvel + 2.0 * k2.dvel + 2.0 * k3.dvel + k4.dvel) / 6.0,
        };
        return advance(s, weighted, dt);
    }
}