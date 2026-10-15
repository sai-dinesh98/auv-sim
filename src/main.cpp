#include <iostream>
#include "auv/hydro.hpp"

int main() {
    for (double v = 0.0; v <= 2.0; v+= 0.5) {
        std::cout << "v = " << v << " m/s, drag = " << drag(v) << " N\n";
    }
    return 0;
}