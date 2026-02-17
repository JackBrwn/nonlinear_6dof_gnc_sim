#pragma once

#include "MathTypes.hpp"
#include "Input.hpp"

class RigidBody{
public:

    struct State{
        math::Vec3 position = math::Vec3::Zero();
        math::Vec3 velocity = math::Vec3::Zero();
        math::Quat orientation = math::Quat::Identity();
        math::Vec3 angularVelocity = math::Vec3::Zero();
    };

private:

    State m_state;
    State m_initialState;
    math::Mat3 m_inertia;
    double m_mass;
    
public:

    RigidBody( double mass, const math::Mat3& inertia);
    RigidBody( double mass, const math::Mat3& inertia, const State& state);

    const math::Vec3& getPosition() const;
    const math::Vec3& getVelocity() const;
    const math::Quat& getOrientation() const;
    const math::Vec3& getAngularVelocity() const;
    const double& getMass() const;
    const math::Mat3& getInertia() const;

    void setPosition(const math::Vec3& position);
    void setVelocity(const math::Vec3& velocity);
    void setOrientation(const math::Quat& orientation);
    void setAngularVelocity(const math::Vec3& angularVelocity);
    void setMass(const double mass);
    void setInertia(const math::Mat3& inertia);

    void step(double dt,const Input& u);
    void resetState();

    void applyForce(const math::Vec3& force);
    void applyTorque(const math::Vec3& torque);

    void logState() const;

private:

    void setState(const State& state);
    void normalizeOrientation();
    State derivative(const State& state, const Input& u) const;
};