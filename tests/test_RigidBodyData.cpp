#include "../include/RigidBody.hpp" 
#include "../include/MathTypes.hpp"

using math::Vec3;
using math::Mat3;
using math::Quat;
using RBS = RigidBody::State;

int main() {

    RBS state {
        .position = Vec3(1, 5, 9),
        .velocity = Vec3(2, 9, -3),
        .orientation = Quat(1, 5, 2.34, 0),
        .angularVelocity = Vec3(-1, -3, -8)
    };

    RigidBody System(10, Mat3::Identity(), state);
    System.logState();

}