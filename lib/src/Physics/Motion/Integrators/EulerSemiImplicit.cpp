#include <Physics/Motion/Integrators/EulerSemiImplicit.h>
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>
#include <cmath>

namespace epl
{
	void EulerSemiImplicit::integrate(Registry& registry, float dt, float damping)
	{
		float currentStepDampingFactor = powf(1.f - damping, dt);
		integrateLinearAcceleration(registry, dt);
		integrateLinearVelocity(registry, dt, currentStepDampingFactor);
		integrateAngularAcceleration(registry, dt);
		integrateAngularVelocity(registry, dt, currentStepDampingFactor);
	}

	void EulerSemiImplicit::integrateLinearAcceleration(Registry& registry, float dt)
	{
		for (auto [entity, velocity] : registry.iterate<LinearVelocity>())
		{
			Force& force = registry.getComponent<Force>(entity);
			const Mass& mass = registry.getComponent<Mass>(entity);
			velocity.value += force.value * (mass.inverseMass * dt);
			force.value = Vector3::zero(); // reset the force sum in the same iteration
		}
	}
	void EulerSemiImplicit::integrateLinearVelocity(Registry& registry, float dt, float damping)
	{
		for (auto [entity, velocity] : registry.iterate<LinearVelocity>())
		{
			Position& position = registry.getComponent<Position>(entity);
			position.value += velocity.value * dt;
			velocity.value *= damping; // apply damping in the same iteration
		}
	}
	void EulerSemiImplicit::integrateAngularAcceleration(Registry& registry, float dt)
	{
		//TODO: inertia
		for (auto [entity, angularVelocity] : registry.iterate<AngularVelocity>())
		{
			Torque& torque = registry.getComponent<Torque>(entity);
			angularVelocity.value += torque.value * dt;
			torque.value = Vector3::zero();
		}
	}

	void EulerSemiImplicit::integrateAngularVelocity(Registry& registry, float dt, float damping)
	{
		for (auto [entity, angularVelocity] : registry.iterate<AngularVelocity>())
		{
			Rotation& rotation = registry.getComponent<Rotation>(entity);
			Quaternion deltaRotation = 
				Quaternion{ 0, angularVelocity.value.x, angularVelocity.value.y, angularVelocity.value.z } * rotation.value;
			rotation.value += deltaRotation * (0.5f * dt);
			rotation.value = Quaternion::normalize(rotation.value);
			angularVelocity.value *= damping; // apply damping in the same iteration
		}
	}
}