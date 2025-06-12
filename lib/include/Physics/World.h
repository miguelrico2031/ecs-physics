#pragma once
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>
#include <Physics/Motion/GravitySystem.h>
#include <Physics/Motion/Integrators/IIntegrationSystem.h>
#include <Physics/Colliders/ColliderRegistry.h>
#include <Physics/Colliders/SphereCollider.h>
#include <Physics/Colliders/AABBCollider.h>
#include <Physics/Collision/Detection/ICollisionDetectionSystem.h>
#include <Physics/Collision/Collision.h>
#include <Physics/Raycast/IRaycastSystem.h>
#include <memory>
#include <vector>
namespace epl
{
	class World
	{
	public:
		World(size_t maxEntities, float damping = 0.f);
		World(std::shared_ptr<Registry> registry, float damping = 0.f);

		Registry& getRegistry() { return *m_registry; }
		const Registry& getRegistry() const { return *m_registry; }

		Entity createDynamicBody(float mass = 1.f, Vector3 position = Vector3::zero(), 
			Quaternion rotation = Quaternion::identity(), Vector3 gravity = Gravity::earth());

		Entity createKinematicBody(Vector3 position = Vector3::zero(), Quaternion rotation = Quaternion::identity());

		void step(float timeStep, unsigned int substeps = 1);


		bool raycast(const Ray& ray, RayHit& hit) const;
		void raycastMultiple(const Ray& ray, std::vector<RayHit>& hits, size_t maxHits) const;

	private:
		void registerPhysicsComponents();
		void registerBuiltInColliders();
	private:
		std::shared_ptr<Registry> m_registry;
		std::unique_ptr<ColliderRegistry> m_colliderRegistry;
		std::unique_ptr<IIntegrationSystem> m_integrationSystem;
		std::unique_ptr<GravitySystem> m_gravitySystem;
		std::unique_ptr<ICollisionDetectionSystem> m_collisionDetectionSystem;
		std::unique_ptr<IRaycastSystem> m_raycastSystem;
	public:
		std::vector<Collision> m_collisions;
		float m_damping;
	};

}