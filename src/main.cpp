#include <cstdlib>
#include <vector>
#include <iostream>

#include "auv/rigid_body.hpp"
#include "auv/integrators.hpp"

int main(int argc, char** argv){

    const double dt = (argc > 1) ? std::atof(argv[1]) : 0.5;
    constexpr double tEnd{5.0};
    const int steps = static_cast<int>(tEnd/dt);

    const auv::RigidBody hull{30.0, 0.028, 0.3}; // mass , volume, CdA

    std::vector<auv::State> euler;
    std::vector<auv::State> rk4;

    euler.reserve(steps + 1);
    rk4.reserve(steps+1);

    euler.push_back(auv::State{});
    rk4.push_back(auv::State{});

    for (int i = 0; i < steps; ++i){
        euler.push_back(auv::eulerStep(hull, euler.back(), dt));
        rk4.push_back(auv::rk4Step(hull, rk4.back(), dt));
    }

    std::cout << "dt = " << dt << " s, analytic terminal speed = " << hull.terminalSpeed() << " m/s\n";
    std::cout << "t [s]\tv_z Euler\tv_z RK4\n";
    for (std::size_t i = 0; i < euler.size(); ++i){
        std::cout << i * dt << "\t" << euler[i].vel.z << "\t\t" << rk4[i].vel.z << "\n";
    }
}