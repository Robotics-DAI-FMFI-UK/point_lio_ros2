#pragma once

#include <array>
#include <cmath>

namespace scan_reference {
// Row-major body-to-world rotation and world position of the body origin.
struct Pose {
    std::array<double, 9> rotation;
    std::array<double, 3> position;
};

inline std::array<double, 3> worldToBody(
    const Pose &pose, double x, double y, double z) {
    const std::array<double, 3> delta{{
        x - pose.position[0], y - pose.position[1], z - pose.position[2]}};
    std::array<double, 3> body{{0, 0, 0}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            body[i] += pose.rotation[3 * j + i] * delta[j];
    return body;
}
}  // namespace scan_reference
