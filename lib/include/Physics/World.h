#pragma once
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>
#include <Physics/Motion/GravitySystem.h>
#include <Physics/Motion/Integrators/IIntegrationSystem.h>
#include <Physics/Collision/ColliderComponents.h>
#include <Physics/Collision/Detection/ICollisionDetectionSystem.h>
#include <memory>
namespace epl
{
	class World
	{
	public:
		World(size_t maxEntities);
		World(std::shared_ptr<Registry> registry);

		Registry& getRegistry() { return *m_registry; }
		const Registry& getRegistry() const { return *m_registry; }

		Entity createDynamicBody(float mass = 1.f, Vector3 position = Vector3::zero(), 
			Quaternion rotation = Quaternion::identity(), Vector3 gravity = Gravity::earth());

		Entity createKinematicBody(Vector3 position = Vector3::zero(), Quaternion rotation = Quaternion::identity());

		void step(float timeStep, unsigned int substeps = 1);

	private:
		void registerPhysicsComponents();
	private:
		std::shared_ptr<Registry> m_registry;
		std::unique_ptr<IIntegrationSystem> m_integrationSystem;
		std::unique_ptr<GravitySystem> m_gravitySystem;
		std::unique_ptr<ICollisionDetectionSystem> m_collisionDetectionSystem;
	};

}