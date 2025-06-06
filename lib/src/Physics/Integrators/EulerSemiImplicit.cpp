#include <Physics/Integrators/EulerSemiImplicit.h>
#include <ECS/Registry.h>

namespace epl
{
	void EulerSemiImplicit::integrate(Registry& registry, float dt)
	{
		integrateLinearVelocity(registry, dt);
		integratePosition(registry, dt);
		integrateAngularVelocity(registry, dt);
		integrateRotation(registry, dt);
	}

	void EulerSemiImplicit::integrateLinearVelocity(Registry& registry, float dt)
	{
		for (auto [entity, velocity] : registry.iterate<LinearVelocity>())
		{
			ForceSum& forceSum = registry.getComponent<ForceSum>(entity);
			const Mass& mass = registry.getComponent<Mass>(entity);
			velocity.value += forceSum.value * (mass.inverseMass * dt);
			forceSum.value = Vector3::zero();
		}
	}
	void EulerSemiImplicit::integratePosition(Registry& registry, float dt)
	{
		for (const auto [entity, velocity] : registry.iterate<LinearVelocity>())
		{
			Position& position = registry.getComponent<Position>(entity);
			position.value += velocity.value * dt;
		}
	}
	void EulerSemiImplicit::integrateAngularVelocity(Registry& registry, float dt)
	{
		//TODO: inertia
		for (auto [entity, angularVelocity] : registry.iterate<AngularVelocity>())
		{
			TorqueSum& torqueSum = registry.getComponent<TorqueSum>(entity);
			angularVelocity.value += torqueSum.value * dt;
			torqueSum.value = Vector3::zero();
		}
	}

	void EulerSemiImplicit::integrateRotation(Registry& registry, float dt)
	{
		for (const auto [entity, angularVelocity] : registry.iterate<AngularVelocity>())
		{
			Rotation& rotation = registry.getComponent<Rotation>(entity);
			Quaternion deltaRotation = Quaternion{ 0, angularVelocity.value.x, angularVelocity.value.y, angularVelocity.value.z } * rotation.value;
			rotation.value += deltaRotation * (0.5f * dt);
			rotation.value = Quaternion::normalize(rotation.value);
		}
	}
}