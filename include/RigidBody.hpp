#pragma once

#include <Eigen/Dense>

class RigidBody{
public:

    struct State{
        Eigen::Vector3d poisiton;
        Eigen::Vector3d velocity;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d angularVelocity;
    };

private:

    State m_state;
    Eigen::Matrix3d m_inertia;
    double m_mass;
    
public:

    RigidBody(Eigen::Vector3d position, Eigien::Vector3d velocity, )
};