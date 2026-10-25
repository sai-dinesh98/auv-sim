#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <vector>

#include <auv/integrators.hpp>
#include <auv/rigid_body.hpp>

int main(int argc, char** argv){
    const double dt = (argc > 1) ? std::atof(argv[1]) : 0.1;
    constexpr double tEnd{4.0};
    const int steps = static_cast<int>(std::round(tEnd / dt));
    constexpr double kRadToDeg = 180.0 / std::numbers::pi;

    const auv::RigidBody hull{30.0, 0.028, 0.3};

    auv::State initial{};
    initial.omega = {0.0, 0.0, std::numbers::pi / 2.0};

    std::vector<auv::State> euler{initial};
    std::vector<auv::State> rk4{initial};

    for (int i = 0; i < steps; ++i) {
        euler.push_back(auv::eulerStep(hull, euler.back(), dt));
        rk4.push_back(auv::rk4Step(hull, rk4.back(), dt));
    }

    std::cout << "t [s]\tyaw Euler [deg]\tyaw RK4 [deg]\t |q| Euler\n";
    const int printEvery = std::max(1, static_cast<int>(std::round(0.5 / dt)));
    for (std::size_t i = 0; i < euler.size(); i += printEvery) {
        std::cout << i * dt << "\t" << euler[i].att.yaw() * kRadToDeg << "\t\t" << rk4[i].att.yaw() * kRadToDeg << "\t\t" << euler[i].att.norm() << "\n";
    }

    const std::size_t oneSecond = static_cast<std::size_t>(std::round(1.0 / dt));
    const auv::Vec3 nose = rk4.at(oneSecond).att.rotate({1.0, 0.0, 0.0});
    std::cout << "nose after 1 s (RK4): (" << nose.x << ", " << nose.y << ", " << nose.z << ")\n";
    return 0;
}