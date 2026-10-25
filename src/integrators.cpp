#include "auv/integrators.hpp"

namespace auv {

    namespace{
        State advance(const State& s, const StateDerivative& d, double h) {
            return {s.pos + h * d.dpos, s.vel + h * d.dvel, s.att + h * d.datt, s.omega + h * d.domega};
        }
    }

    State eulerStep(const RigidBody& body, const State& s, double dt){

        State next = advance(s, body.derivative(s), dt);
        next.att = next.att.normalized();
        return next;
    }

    State rk4Step(const RigidBody& body, const State& s , double dt) {
        const StateDerivative k1 = body.derivative(s);
        const StateDerivative k2 = body.derivative(advance(s, k1, dt/2.0));
        const StateDerivative k3 = body.derivative(advance(s, k2, dt/2.0));
        const StateDerivative k4 = body.derivative(advance(s, k3, dt));

        const StateDerivative weighted{
            (k1.dpos + 2.0 * k2.dpos + 2.0 * k3.dpos + k4.dpos) / 6.0,
            (k1.dvel + 2.0 * k2.dvel + 2.0 * k3.dvel + k4.dvel) / 6.0,
            (1.0 / 6.0) * (k1.datt + 2.0 * k2.datt + 2.0 * k3.datt + k4.datt),
            (k1.domega + 2.0 * k2.domega + 2.0 * k3.domega + k4.domega) / 6.0,
        };

        State next = advance(s, weighted, dt);
        next.att = next.att.normalized();

        return next;
    }
}