#include "auv/hydro.hpp"

double drag(double v) {
    constexpr double rho = 1025.0; // density, kg/m3
    constexpr double CdA = 0.25; // Drag coefficient x area, m^2
    return 0.5 * rho * CdA * v * v;
}