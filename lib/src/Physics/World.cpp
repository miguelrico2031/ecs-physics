#include <Physics/World.h>
#include <Physics/Motion/Integrators/EulerSemiImplicit.h>
#include <Physics/Collision/IterativeCollisionDetection.h>
#include <Physics/Collision/ProjectionAndImpulseSolver.h>
#include <Physics/Raycast/IterativeRaycaster.h>
#include <Physics/Colliders/ColliderFuncs.h>
namespace epl
{
	World::World(size_t maxEntities, float damping)
		: m_registry(std::make_shared<Registry>(maxEntities)),
		m_integrationSystem(std::make_unique<EulerSemiImplicit>()),
		m_gravitySystem(std::make_unique<GravitySystem>()),
		m_collisionDetectionSystem(std::make_unique<IterativeCollisionDetection>()),
		m_collisionResolutionSystem(std::make_unique<ProjectionAndImpulseSolver>()),
		m_raycastSystem(std::make_unique<IterativeRaycaster>()),
		m_damping(damping)
	{
		m_collisions.reserve(maxEntities / 2);
		registerPhysicsComponents();
	}

	World::World(std::shared_ptr<Registry> registry, float damping)
		: m_registry(registry),
		m_integrationSystem(std::make_unique<EulerSemiImplicit>()),
		m_gravitySystem(std::make_unique<GravitySystem>()),
		m_collisionDetectionSystem(std::make_unique<IterativeCollisionDetection>()),
		m_collisionResolutionSystem(std::make_unique<ProjectionAndImpulseSolver>()),
		m_raycastSystem(std::make_unique<IterativeRaycaster>()),
		m_damping(damping)
	{
		m_collisions.reserve(m_registry->getMaxEntities() / 2);
		registerPhysicsComponents();
	}

	Entity World::createDynamicBody(float mass, Vector3 position, Quaternion rotation, Vector3 gravity)
	{
		assert(mass > Math::epsilon() && "Mass of dynamic body must be greater than zero.");
		Entity e = m_registry->createEntity();
		m_registry->addComponent<DynamicBody>(e);
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
		m_registry->addComponent<PhysicMaterial>(e, 0.f, .5f);
		return e;
	}

	Entity World::createKinematicBody(Vector3 position, Quaternion rotation)
	{
		Entity e = m_registry->createEntity();
		m_registry->addComponent<Position>(e, position);
		m_registry->addComponent<Rotation>(e, rotation);
		return e;
	}

	void World::destroyBody(Entity entity)
	{
		//TODO
	}


	const SphereCollider& World::addSphereColliderToBody(Entity entity, float radius)
	{
		auto& bounds = m_registry->addComponent<ColliderBounds>(entity, Vector3::zero(), Vector3::zero());
		ColliderFuncs::calculateSphereBounds(radius, bounds);

		auto& col = m_registry->addComponent<SphereCollider>(entity, radius);
		//inertia calc if dynamic
		if (m_registry->hasComponent<DynamicBody>(entity))
		{
			const Mass& mass = m_registry->getComponent<Mass>(entity);
			auto invInertia = ColliderFuncs::calculateSphereInverseInertiaTensor(radius, mass.inverseMass);
			m_registry->getComponent<InverseInertia>(entity).tensor = invInertia;
		}
		return col;
	}

	const BoxCollider& World::addBoxColliderToBody(Entity entity, Vector3 halfSize)
	{
		auto& bounds = m_registry->addComponent<ColliderBounds>(entity, Vector3::zero(), Vector3::zero());
		ColliderFuncs::calculateBoxBounds(halfSize, bounds);

		auto& col = m_registry->addComponent<BoxCollider>(entity, halfSize);
		//inertia calc if dynamic
		if (m_registry->hasComponent<DynamicBody>(entity))
		{
			const Mass& mass = m_registry->getComponent<Mass>(entity);
			auto localInvInertia = ColliderFuncs::calculateBoxInverseInertiaTensor(halfSize, mass.inverseMass);
			m_registry->addComponent<LocalInverseInertia>(entity, localInvInertia);
			const Quaternion& rotation = m_registry->getComponent<Rotation>(entity).value;
			auto rotatedInvInertia = ColliderFuncs::calculateRotatedBoxInverseInertiaTensor(localInvInertia, rotation);
			m_registry->getComponent<InverseInertia>(entity).tensor = rotatedInvInertia;
		}
		return col;
	}

	/*
	const AABBCollider& World::addAABBColliderToBody(Entity entity, Vector3 halfSize)
	{
		assert(!m_registry->hasComponent<DynamicBody>(entity) && "Cannot add AABB to dynamic body.");
		auto& col = m_registry->addComponent<AABBCollider>(entity, halfSize);

		return col;
	}
	*/

	void World::removeColliderFromBody(Entity entity)
	{
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
			m_collisionDetectionSystem->detectCollisions(*m_registry, m_collisions);
			//solve collisions
			if (!m_collisions.empty())
			{
				m_collisionResolutionSystem->resolveCollisions(*m_registry, m_collisions);
			}
		}
	}

	bool World::raycast(const Ray& ray, RayHit& hit) const
	{
		return m_raycastSystem->raycast(ray, *m_registry, hit);
	}

	void World::raycastMultiple(const Ray& ray, std::vector<RayHit>& hits, size_t maxHits) const
	{
		return m_raycastSystem->raycastMultiple(ray, *m_registry, hits, maxHits);
	}


	void World::addForce(Entity entity, Vector3 force)
	{
		assert(m_registry->hasComponent<DynamicBody>(entity) && "Kinematic bodies cannot move with accelerations.");
		m_registry->getComponent<Force>(entity).value += force;
	}

	void World::addForce(Force& forceComponent, Vector3 force)
	{
		forceComponent.value += force;
	}

	void World::addForceAtPoint(Entity entity, Vector3 force, Vector3 point)
	{
		assert(m_registry->hasComponent<DynamicBody>(entity) && "Kinematic bodies cannot move with accelerations.");
		m_registry->getComponent<Force>(entity).value += force;
		Position position = m_registry->getComponent<Position>(entity);
		Vector3 pointLocalPos = point - position.value;
		Vector3 torque = Vector3::cross(pointLocalPos, force);
		m_registry->getComponent<Torque>(entity).value += torque;
	}

	void World::addAcceleration(Entity entity, Vector3 acceleration)
	{
		assert(m_registry->hasComponent<DynamicBody>(entity) && "Kinematic bodies cannot move with accelerations.");
		float mass = m_registry->getComponent<Mass>(entity).mass;
		m_registry->getComponent<Force>(entity).value += mass * acceleration;
	}

	void World::addAcceleration(Force& forceComponent, Mass& massComponent, Vector3 acceleration)
	{
		forceComponent.value += massComponent.mass * acceleration;
	}

	void World::addTorque(Entity entity, Vector3 torque)
	{
		assert(m_registry->hasComponent<DynamicBody>(entity) && "Kinematic bodies cannot move with accelerations.");
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
		if (m_registry->hasComponent<DynamicBody>(entity))
		{
			const Mass& mass = m_registry->getComponent<Mass>(entity);
			auto invInertia = ColliderFuncs::calculateSphereInverseInertiaTensor(newRadius, mass.inverseMass);
			m_registry->getComponent<InverseInertia>(entity).tensor = invInertia;
		}
	}

	void World::changeBoxColliderHalfSize(Entity entity, Vector3 newHalfSize)
	{
		auto& col = m_registry->getComponent<BoxCollider>(entity);
		col.halfSize = newHalfSize;
		//update inertia tensor if dynamic
		if (m_registry->hasComponent<DynamicBody>(entity))
		{
			const Mass& mass = m_registry->getComponent<Mass>(entity);
			auto localInvInertia = ColliderFuncs::calculateBoxInverseInertiaTensor(newHalfSize, mass.inverseMass);
			m_registry->getComponent<LocalInverseInertia>(entity).tensor = localInvInertia;
			const Quaternion& rotation = m_registry->getComponent<Rotation>(entity).value;
			auto rotatedInvInertia = ColliderFuncs::calculateRotatedBoxInverseInertiaTensor(localInvInertia, rotation);
			m_registry->getComponent<InverseInertia>(entity).tensor = rotatedInvInertia;
		}
	}


	void World::changeDynamicBodyMass(Entity entity, float newMass)
	{
		assert(newMass > Math::epsilon() && "Mass of dynamic body must be greater than zero.");
		assert(m_registry->hasComponent<DynamicBody>(entity) && "Cannot change mass of kinematic body.");
		Mass& mass = m_registry->addOrSetComponent<Mass>(entity, newMass); //set so cosntructor is called and inv mass is computed

		if (const auto& sphereOpt = m_registry->tryGetComponent<SphereCollider>(entity))
		{
			auto invInertia = ColliderFuncs::calculateSphereInverseInertiaTensor(sphereOpt->radius, mass.inverseMass);
			m_registry->getComponent<InverseInertia>(entity).tensor = invInertia;
		}

		else if (const auto& boxOpt = m_registry->tryGetComponent<BoxCollider>(entity))
		{
			auto localInvInertia = ColliderFuncs::calculateBoxInverseInertiaTensor(boxOpt->halfSize, mass.inverseMass);
			m_registry->getComponent<LocalInverseInertia>(entity).tensor = localInvInertia;
			const Quaternion& rotation = m_registry->getComponent<Rotation>(entity).value;
			auto rotatedInvInertia = ColliderFuncs::calculateRotatedBoxInverseInertiaTensor(localInvInertia, rotation);
			m_registry->getComponent<InverseInertia>(entity).tensor = rotatedInvInertia;
		}
	}


	void World::setDynamic(Entity entity, bool setToDynamic)
	{
		bool dynamic = m_registry->hasComponent<DynamicBody>(entity);
		if ((dynamic && setToDynamic) || (!dynamic && !setToDynamic))
		{
			return;
		}

		if (!setToDynamic) //change dynamic body to being kinematic
		{
			m_registry->removeComponent<DynamicBody>(entity);
			Mass& mass = m_registry->getComponent<Mass>(entity);
			m_dynamicBodiesMasses[entity] = mass.mass;
			mass.mass = 0.f;
			mass.inverseMass = 0.f;
			m_registry->getComponent<LinearVelocity>(entity).value = Vector3::zero();
			m_registry->getComponent<AngularVelocity>(entity).value = Vector3::zero();
		}
		else
		{
			assert(m_dynamicBodiesMasses.find(entity) != m_dynamicBodiesMasses.end() && "Cannot set to dynamic a body that was not created dynamic.");
			m_registry->addComponent<DynamicBody>(entity);
			m_registry->addOrSetComponent<Mass>(entity, m_dynamicBodiesMasses[entity]);
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
		m_registry->registerComponentType<LocalInverseInertia>();
		m_registry->registerComponentType<DynamicBody>();
		m_registry->registerComponentType<PhysicMaterial>();
		m_registry->registerComponentType<AABBCollider>();
		m_registry->registerComponentType<SphereCollider>();
		m_registry->registerComponentType<OBBCollider>();
		m_registry->registerComponentType<ColliderBounds>();
	}
}
