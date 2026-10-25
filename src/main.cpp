#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>

#include "auv/integrators.hpp"
#include "auv/rigid_body.hpp"

int main(int argc, char** argv) {
    const double dt = (argc > 1) ? std::atof(argv[1]) : 0.01;
    constexpr double tEnd{30.0};
    const int steps = static_cast<int>(std::round(tEnd / dt));

    // Ix < Iy < Iz  ->  y is the INTERMEDIATE axis
    const auv::RigidBody hull{30.0, 0.028, 0.3, {0.4, 1.2, 1.4}};

    auv::State initial{};
    initial.omega = {0.01, 2.0, 0.01};   // spin about y, plus a tiny disturbance

    std::vector<auv::State> euler{initial};
    std::vector<auv::State> rk4{initial};
    for (int i = 0; i < steps; ++i) {
        euler.push_back(auv::eulerStep(hull, euler.back(), dt));
        rk4.push_back(auv::rk4Step(hull, rk4.back(), dt));
    }

    const double e0 = hull.rotationalEnergy(initial);
    const double h0 = hull.angularMomentumWorld(initial).norm();

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  t [s]   wy RK4   E/E0 RK4   |H|/|H0| RK4   E/E0 Euler\n";
    const int printEvery = std::max(1, static_cast<int>(std::round(1.5 / dt)));
    for (std::size_t i = 0; i < rk4.size(); i += printEvery) {
        std::cout << std::setw(7) << i * dt << "  " << std::setw(7) << rk4[i].omega.y << "   "
                  << hull.rotationalEnergy(rk4[i]) / e0 << "     "
                  << hull.angularMomentumWorld(rk4[i]).norm() / h0 << "         "
                  << hull.rotationalEnergy(euler[i]) / e0 << '\n';
    }
}