#include <iostream>
#include "Input.hpp"
#include "MathTypes.hpp"
#include "RigidBody.hpp"

using math::Vec3;
using math::Mat3;
using math::Quat;

// Constructors
RigidBody::RigidBody(double mass, const Mat3& inertia)
:   m_state(),
    m_initialState(),
    m_inertia(inertia),
    m_mass(mass)
{}

RigidBody::RigidBody(double mass, const Mat3& inertia, const State& state)
:   m_state(state),
    m_initialState(state),
    m_inertia(inertia),
    m_mass(mass)
{}

void RigidBody::resetState() { m_state = m_initialState; }

void RigidBody::logState() const {
   std::cout << "Mass: " << m_mass << " (kg)\n"
             << "Position: " << m_state.position.transpose() << " (m)\n"
             << "Velocity: " << m_state.velocity.transpose() << " (m/s)\n"
             << "Angular Velocity: " << m_state.angularVelocity.transpose() << " (rad/s)\n"
             << "Orientation (Quat): " << m_state.orientation << " \n"
             << "Orientation (rpy): " << m_state.orientation.toRotationMatrix().canonicalEulerAngles(0, 1, 2).transpose() << " (rad)\n"
             << "Inertia: \n" << m_inertia << " (kg m^2)" << std::endl;
}

// Getters
const math::Vec3& RigidBody::getPosition() const { return m_state.position; }
const math::Vec3& RigidBody::getVelocity() const { return m_state.velocity; }
const math::Quat& RigidBody::getOrientation() const { return m_state.orientation; }
const math::Vec3& RigidBody::getAngularVelocity() const { return m_state.angularVelocity; }
const double& RigidBody::getMass() const { return m_mass; }
const math::Mat3& RigidBody::getInertia() const { return m_inertia; }

// Setters
void RigidBody::setPosition(const math::Vec3& position) { m_state.position = position; }
void RigidBody::setVelocity(const math::Vec3& velocity) { m_state.velocity = velocity; }
void RigidBody::setOrientation(const math::Quat& orientation) { m_state.orientation = orientation; }
void RigidBody::setAngularVelocity(const math::Vec3& angularVelocity) { m_state.angularVelocity = angularVelocity; }
void RigidBody::setMass(const double mass) { m_mass = mass; }
void RigidBody::setInertia(const math::Mat3& inertia) { m_inertia = inertia; }

void RigidBody::step(double dt,const Input& u) {}

void RigidBody::applyForce(const math::Vec3& force) {}
void RigidBody::applyTorque(const math::Vec3& torque) {}