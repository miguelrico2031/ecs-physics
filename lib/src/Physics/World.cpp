#include <Physics/World.h>
#include <Physics/Motion/Integrators/EulerSemiImplicit.h>
#include <Physics/Collision/Detection/DoubleIteration.h>
#include <Physics/Raycast/IterativeRaycaster.h>
namespace epl
{
	World::World(size_t maxEntities, float damping)
		: m_registry(std::make_shared<Registry>(maxEntities)),
		m_colliderRegistry(std::make_unique<ColliderRegistry>()),
		m_integrationSystem(std::make_unique<EulerSemiImplicit>()),
		m_gravitySystem(std::make_unique<GravitySystem>()),
		m_collisionDetectionSystem(std::make_unique<DoubleIteration>()),
		m_raycastSystem(std::make_unique<IterativeRaycaster>()),
		m_damping(damping)
	{
		m_collisions.reserve(maxEntities / 2);
		registerPhysicsComponents();
		registerBuiltInColliders();
	}

	World::World(std::shared_ptr<Registry> registry, float damping)
		: m_registry(registry),
		m_colliderRegistry(std::make_unique<ColliderRegistry>()),
		m_integrationSystem(std::make_unique<EulerSemiImplicit>()),
		m_gravitySystem(std::make_unique<GravitySystem>()),
		m_collisionDetectionSystem(std::make_unique<DoubleIteration>()),
		m_raycastSystem(std::make_unique<IterativeRaycaster>()),
		m_damping(damping)
	{
		m_collisions.reserve(m_registry->getMaxEntities() / 2);
		registerBuiltInColliders();
		registerPhysicsComponents();
	}

	Entity World::createDynamicBody(float mass, Vector3 position, Quaternion rotation, Vector3 gravity)
	{
		Entity e = m_registry->createEntity();
		m_registry->addComponent<Mass>(e, mass);
		m_registry->addComponent<Position>(e, position);
		m_registry->addComponent<LinearVelocity>(e, Vector3::zero());
		m_registry->addComponent<Force>(e, Vector3::zero());
		m_registry->addComponent<Rotation>(e, rotation);
		m_registry->addComponent<AngularVelocity>(e, Vector3::zero());
		m_registry->addComponent<Torque>(e, Vector3::zero());
		if (gravity != Vector3::zero())
		{
			m_registry->addComponent<Gravity>(e, gravity);
		}
		return e;
	}

	Entity World::createKinematicBody(Vector3 position, Quaternion rotation)
	{
		Entity e = m_registry->createEntity();
		m_registry->addComponent<Position>(e, position);
		m_registry->addComponent<Rotation>(e, rotation);
		m_registry->addComponent<Kinematic>(e);
		return e;
	}

	void World::step(float timeStep, unsigned int substeps)
	{
		assert(substeps > 0 && "Substeps must be greater than zero.");
		float timeStepPerSubstep = timeStep / substeps;

		for (size_t i = 0; i < substeps; i++)
		{
			//apply forces
			m_gravitySystem->applyGravity(*m_registry);
			//update positions and velocities
			m_integrationSystem->integrate(*m_registry, timeStepPerSubstep, m_damping);
			//detect collisions
			m_collisions.clear();
			m_collisionDetectionSystem->detectCollisions(*m_registry, *m_colliderRegistry, m_collisions);
			//solve collisions
		}
	}

	bool World::raycast(const Ray& ray, RayHit& hit) const
	{
		return m_raycastSystem->raycast(ray, *m_registry, *m_colliderRegistry, hit);
	}

	void World::raycastMultiple(const Ray& ray, std::vector<RayHit>& hits, size_t maxHits) const
	{
		return m_raycastSystem->raycastMultiple(ray, *m_registry, *m_colliderRegistry, hits, maxHits);
	}


	void World::registerPhysicsComponents()
	{
		m_registry->registerComponentType<Mass>();
		m_registry->registerComponentType<Position>();
		m_registry->registerComponentType<LinearVelocity>();
		m_registry->registerComponentType<Force>();
		m_registry->registerComponentType<Rotation>();
		m_registry->registerComponentType<AngularVelocity>();
		m_registry->registerComponentType<Torque>();
		m_registry->registerComponentType<Gravity>();
		m_registry->registerComponentType<Kinematic>();


		m_registry->registerComponentType<SphereCollider>();
		m_registry->registerComponentType<AABBCollider>();
		//m_registry->registerComponentType<IsColliding>();
	}

	void World::registerBuiltInColliders()
	{
		m_colliderRegistry->registerColliderType<SphereCollider>();
		m_colliderRegistry->registerColliderType<AABBCollider>();

		m_colliderRegistry->registerCollisionCheck<SphereCollider, SphereCollider>(SphereColliderFuncs::isCollidingSphereSphere);
		m_colliderRegistry->registerCollisionCheck<AABBCollider, AABBCollider>(AABBColliderFuncs::isCollidingAABBAABB);
		m_colliderRegistry->registerCollisionCheck<SphereCollider, AABBCollider>(SphereColliderFuncs::isCollidingSphereAABB);
		m_colliderRegistry->registerRayIntersectionCheck<SphereCollider>(SphereColliderFuncs::isIntersectingSphere);
		m_colliderRegistry->registerRayIntersectionCheck<AABBCollider>(AABBColliderFuncs::isIntersectingAABB);
	}
}
