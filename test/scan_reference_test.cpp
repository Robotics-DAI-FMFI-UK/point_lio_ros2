#include "scan_reference.hpp"
#include <cassert>
#include <cmath>

int main() {
    // A translated body rotated 90 degrees about z: body (2,3,4) -> world (7,22,34).
    scan_reference::Pose pose{{{0,-1,0, 1,0,0, 0,0,1}}, {{10,20,30}}};
    auto body = scan_reference::worldToBody(pose, 7,22,34);
    assert(std::abs(body[0]-2) < 1e-12);
    assert(std::abs(body[1]-3) < 1e-12);
    assert(std::abs(body[2]-4) < 1e-12);
    // Combined pitch/yaw: exercise all axes and catch transposition/translation errors.
    for (int n = 0; n < 100; ++n) {
        const double a = n * .031, b = n * .017;
        const double c = std::cos(a), s = std::sin(a), C = std::cos(b), S = std::sin(b);
        pose.rotation = {{c*C,-s,c*S, s*C,c,s*S, -S,0,C}};
        const std::array<double, 3> p{{n*.13, -n*.07, .8}};
        std::array<double, 3> w = pose.position;
        for (int i=0; i<3; ++i)
            for (int j=0; j<3; ++j) w[i] += pose.rotation[3*i+j]*p[j];
        body = scan_reference::worldToBody(pose,w[0],w[1],w[2]);
        for (int i=0; i<3; ++i) assert(std::abs(body[i]-p[i]) < 1e-11);
    }
}
