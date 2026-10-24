#include <algorithm>
#include <cmath>
#include <iostream>
#include "auv/vec3.hpp"

// Physical Parameters (SI Units). z axis points UP, so depth = -z
constexpr double kRho       = 1025.0;   //seawater density      [kg/m^3]
constexpr double kG         = 9.81;     //gravity               [m/s^2]
constexpr double kMass      = 30;       //Vehicle Mass          [kg]
constexpr double kVolume    = 0.028;    //displaced volume      [m^3] -> slightly negatively buoyant
constexpr double kCdA       = 0.3;     // drag coeff x area     [m^2]

auv::Vec3 netForce(const auv::Vec3& vel){
    const auv::Vec3 weight      {0.0, 0.0, -kMass*kG};
    const auv::Vec3 buoyancy    {0.0, 0.0, kRho*kVolume*kG};
    const auv::Vec3 drag = -0.5 * kRho * kCdA * vel.norm() * vel;
    return weight + buoyancy + drag;
}

int main() {
    constexpr double dt     {0.5};
    constexpr double tEnd   {5.0};
    const int steps = static_cast<int>(tEnd/dt);
    const int printEvery = std::max(1, static_cast<int>(0.5 / dt));   // print every 0.5 s

    auv::Vec3 pos{};
    auv::Vec3 vel{};

    for (int step = 0; step <= steps; ++step){
        if (step % printEvery == 0){
            std::cout << "t = " << step * dt << " s   depth = " << -pos.z
                      << " m   v_z = " << vel.z << " m/s\n";
        }
        const auv::Vec3 acc = netForce(vel) / kMass;
        pos += vel * dt;
        vel += acc * dt;
    }

    // Terminal speed: drag balances net weight -> 0.5*rho*CdA*v^2 = (m - rho*V)*g
    const double vTerminal = std::sqrt(2.0 * (kMass - kRho * kVolume) * kG / (kRho * kCdA));
    std::cout << "analytic terminal speed = " << vTerminal << " m/s\n";
    return 0;
}