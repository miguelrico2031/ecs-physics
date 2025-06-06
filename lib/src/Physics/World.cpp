#include <Physics/World.h>
#include <Physics/Integrators/EulerSemiImplicit.h>

namespace epl
{
	World::World(size_t maxEntities)
		: m_registry(std::make_shared<Registry>(maxEntities)),
		m_integrationSystem(std::make_unique<EulerSemiImplicit>()),
		m_gravitySystem(std::make_unique<GravitySystem>())
	{
		registerPhysicsComponents();
	}

	World::World(std::shared_ptr<Registry> registry)
		: m_registry(registry),
		m_integrationSystem(std::make_unique<EulerSemiImplicit>()),
		m_gravitySystem(std::make_unique<GravitySystem>())
	{
		registerPhysicsComponents();
	}

	Entity World::createDynamicBody(float mass, Vector3 position, Quaternion rotation, Vector3 gravity)
	{
		Entity e = m_registry->createEntity();
		m_registry->addComponent<Mass>(e, mass);
		m_registry->addComponent<Position>(e, position);
		m_registry->addComponent<LinearVelocity>(e, Vector3::zero());
		m_registry->addComponent<ForceSum>(e, Vector3::zero());
		m_registry->addComponent<Rotation>(e, rotation);
		m_registry->addComponent<AngularVelocity>(e, Vector3::zero());
		m_registry->addComponent<TorqueSum>(e, Vector3::zero());
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
			m_gravitySystem->applyGravity(*m_registry);
			m_integrationSystem->integrate(*m_registry, timeStepPerSubstep);
		}
	}


	void World::registerPhysicsComponents()
	{
		m_registry->registerComponentType<Mass>();
		m_registry->registerComponentType<Position>();
		m_registry->registerComponentType<LinearVelocity>();
		m_registry->registerComponentType<ForceSum>();
		m_registry->registerComponentType<Rotation>();
		m_registry->registerComponentType<AngularVelocity>();
		m_registry->registerComponentType<TorqueSum>();
		m_registry->registerComponentType<Gravity>();
		m_registry->registerComponentType<Kinematic>();
	}
}
