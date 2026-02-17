#pragma once

#include <Eigen/Eigen/Dense>

class Input{
private:

    Eigen::Vector3d m_force;
    Eigen::Vector3d m_torque;
    
public:

    Input(const Eigen::Vector3d& force = Eigen::Vector3d::Zero(), const Eigen::Vector3d& torque = Eigen::Vector3d::Zero());

    void setForce(const Eigen::Vector3d& force);
    void setTorque(const Eigen::Vector3d& torque);

    Eigen::Vector3d& getForce();
    Eigen::Vector3d& getTorque();

};