#include <Physics/World.h>
#include <Physics/Motion/Integrators/EulerSemiImplicit.h>
#include <Physics/Collision/IterativeCollisionDetection.h>
#include <Physics/Raycast/IterativeRaycaster.h>
namespace epl
{
	World::World(size_t maxEntities, float damping)
		: m_registry(std::make_shared<Registry>(maxEntities)),
		m_colliderRegistry(std::make_unique<ColliderRegistry>()),
		m_integrationSystem(std::make_unique<EulerSemiImplicit>()),
		m_gravitySystem(std::make_unique<GravitySystem>()),
		m_collisionDetectionSystem(std::make_unique<IterativeCollisionDetection>()),
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
		m_collisionDetectionSystem(std::make_unique<IterativeCollisionDetection>()),
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
		m_registry->addComponent<Position>(e, position);
		m_registry->addComponent<LinearVelocity>(e, Vector3::zero());
		m_registry->addComponent<Force>(e, Vector3::zero());
		m_registry->addComponent<Rotation>(e, rotation);
		m_registry->addComponent<AngularVelocity>(e, Vector3::zero());
		m_registry->addComponent<Torque>(e, Vector3::zero());
		m_registry->addComponent<Mass>(e, mass);
		m_registry->addComponent<InverseInertia>(e, Matrix3x3::zero()); //uninitialized
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
		return e;
	}

	SphereCollider& World::addSphereColliderToBody(Entity entity, float radius, Vector3 offset)
	{
		auto& col = m_registry->addComponent<SphereCollider>(entity, radius, offset);
		//inertia calc if dynamic
		if (auto& massOpt = m_registry->tryGetComponent<Mass>(entity))
		{
			auto invInertia = SphereColliderFuncs::calculateInverseInertiaTensor(radius, massOpt->inverseMass);
			m_registry->getComponent<InverseInertia>(entity).tensor = invInertia;
		}
		return col;
	}

	BoxCollider& World::addBoxColliderToBody(Entity entity, Vector3 halfSize, Vector3 offset)
	{
		auto& col = m_registry->addComponent<BoxCollider>(entity, halfSize, offset);
		//inertia calc if dynamic
		if (auto& massOpt = m_registry->tryGetComponent<Mass>(entity))
		{
			auto invInertia = AABBColliderFuncs::calculateInverseInertiaTensor(halfSize, massOpt->inverseMass);
			m_registry->getComponent<InverseInertia>(entity).tensor = invInertia;
		}
		return col;
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


	void World::addForce(Entity entity, Vector3 force)
	{
		m_registry->getComponent<Force>(entity).value += force;
	}

	void World::addForce(Force& forceComponent, Vector3 force)
	{
		forceComponent.value += force;
	}

	void World::addForceAtPoint(Entity entity, Vector3 force, Vector3 point)
	{
		m_registry->getComponent<Force>(entity).value += force;
		Position position = m_registry->getComponent<Position>(entity);
		Vector3 pointLocalPos = point - position.value;
		Vector3 torque = Vector3::cross(pointLocalPos, force);
		m_registry->getComponent<Torque>(entity).value += torque;
	}

	void World::addAcceleration(Entity entity, Vector3 acceleration)
	{
		float mass = m_registry->getComponent<Mass>(entity).mass;
		m_registry->getComponent<Force>(entity).value += mass * acceleration;
	}

	void World::addAcceleration(Force& forceComponent, Mass& massComponent, Vector3 acceleration)
	{
		forceComponent.value += massComponent.mass * acceleration;
	}

	void World::addTorque(Entity entity, Vector3 torque)
	{
		m_registry->getComponent<Torque>(entity).value += torque;
	}

	void World::addTorque(Torque& torqueComponent, Vector3 torque)
	{
		torqueComponent.value += torque;
	}


	void World::changeSphereColliderRadius(Entity entity, float newRadius)
	{
		auto& col = m_registry->getComponent<SphereCollider>(entity);
		col.radius = newRadius;
		//update inertia tensor if dynamic
		if (auto& massOpt = m_registry->tryGetComponent<Mass>(entity))
		{
			auto invInertia = SphereColliderFuncs::calculateInverseInertiaTensor(newRadius, (*massOpt).inverseMass);
			m_registry->getComponent<InverseInertia>(entity).tensor = invInertia;
		}
	}

	void World::changeBoxColliderHalfSize(Entity entity, Vector3 newHalfSize)
	{
		auto& col = m_registry->getComponent<BoxCollider>(entity);
		col.halfSize = newHalfSize;
		//update inertia tensor if dynamic
		if (auto& massOpt = m_registry->tryGetComponent<Mass>(entity))
		{
			auto invInertia = AABBColliderFuncs::calculateInverseInertiaTensor(newHalfSize, (*massOpt).inverseMass);
			m_registry->getComponent<InverseInertia>(entity).tensor = invInertia;
		}
	}


	void World::registerPhysicsComponents()
	{
		m_registry->registerComponentType<Position>();
		m_registry->registerComponentType<LinearVelocity>();
		m_registry->registerComponentType<Force>();
		m_registry->registerComponentType<Rotation>();
		m_registry->registerComponentType<AngularVelocity>();
		m_registry->registerComponentType<Torque>();
		m_registry->registerComponentType<Gravity>();
		m_registry->registerComponentType<Mass>();
		m_registry->registerComponentType<InverseInertia>();
	}

	void World::registerBuiltInColliders()
	{
		m_registry->registerComponentType<AABBCollider>();
		m_colliderRegistry->registerColliderType<AABBCollider>();
		m_colliderRegistry->registerCollisionCheck<AABBCollider, AABBCollider>(AABBColliderFuncs::isCollidingAABBAABB);
		m_colliderRegistry->registerRayIntersectionCheck<AABBCollider>(AABBColliderFuncs::isIntersectingAABB);

		m_registry->registerComponentType<SphereCollider>();
		m_colliderRegistry->registerColliderType<SphereCollider>();
		m_colliderRegistry->registerCollisionCheck<SphereCollider, SphereCollider>(SphereColliderFuncs::isCollidingSphereSphere);
		m_colliderRegistry->registerCollisionCheck<SphereCollider, AABBCollider>(SphereColliderFuncs::isCollidingSphereAABB);
		m_colliderRegistry->registerRayIntersectionCheck<SphereCollider>(SphereColliderFuncs::isIntersectingSphere);


		m_registry->registerComponentType<OBBCollider>();
		m_colliderRegistry->registerColliderType<OBBCollider>();
		m_colliderRegistry->registerCollisionCheck<OBBCollider, OBBCollider>(OBBColliderFuncs::isCollidingOBBOBB);
		m_colliderRegistry->registerCollisionCheck<OBBCollider, AABBCollider>(OBBColliderFuncs::isCollidingOBBAABB);
		m_colliderRegistry->registerCollisionCheck<OBBCollider, SphereCollider>(OBBColliderFuncs::isCollidingOBBSphere);
		m_colliderRegistry->registerRayIntersectionCheck<OBBCollider>(OBBColliderFuncs::isIntersectingOBB);
	}
}
