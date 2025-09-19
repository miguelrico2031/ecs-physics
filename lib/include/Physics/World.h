#pragma once
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>
#include <Physics/Motion/GravitySystem.h>
#include <Physics/Motion/Integrators/IIntegrationSystem.h>
#include <Physics/Colliders/SphereCollider.h>
#include <Physics/Colliders/AABBCollider.h>
#include <Physics/Colliders/OBBCollider.h>
#include <Physics/Colliders/ColliderBounds.h>
#include <Physics/Collision/ICollisionDetectionSystem.h>
#include <Physics/Collision/ICollisionResolutionSystem.h>
#include <Physics/Collision/Collision.h>
#include <Physics/Raycast/IRaycastSystem.h>
#include <memory>
#include <vector>
#include <unordered_map>
namespace epl
{
	class World
	{
	public:
		World(size_t maxEntities, float damping = 0.f);
		World(std::shared_ptr<Registry> registry, float damping = 0.f);

#pragma region GETTERS_SETTERS
		Registry& getRegistry() { return *m_registry; }
		const Registry& getRegistry() const { return *m_registry; }

		float getDamping() const { return m_damping; }
		void setDamping(float damping) { m_damping = damping; }

		size_t getMaxEntities() const { return m_registry->getMaxEntities(); }

		const std::vector<Collision>& getAllCollisions() const { return m_collisions; }
#pragma endregion

#pragma region UPDATE_PHYSICS
		void step(float timeStep, unsigned int substeps = 1);
#pragma endregion

#pragma region BODY_LIFETIME
		//creates an entity with all the necessary components to act as a dynamic rigid body
		Entity createDynamicBody(float mass = 1.f, Vector3 position = Vector3::zero(), 
			Quaternion rotation = Quaternion::identity(), Vector3 gravity = Gravity::earth());
		//creates an entity with all the necessary components to act as a kinematic rigid body
		Entity createKinematicBody(Vector3 position = Vector3::zero(), Quaternion rotation = Quaternion::identity());
		
		void destroyBody(Entity entity);
#pragma endregion

#pragma region COLLIDER_LIFETIME
		//adds a sphere collider to a body and if it is dynamic, calculates it's inertia tensor
		const SphereCollider& addSphereColliderToBody(Entity entity, float radius);
		//adds a box collider to a body and if it is dynamic, calculates it's inertia tensor
		const BoxCollider& addBoxColliderToBody(Entity entity, Vector3 halfSize);
		//const AABBCollider& addAABBColliderToBody(Entity entity, Vector3 halfSize);

		void removeColliderFromBody(Entity entity);
#pragma endregion

#pragma region RAYCAST_HELPERS
		bool raycast(const Ray& ray, RayHit& hit) const;
		void raycastMultiple(const Ray& ray, std::vector<RayHit>& hits, size_t maxHits) const;
#pragma endregion

#pragma region FORCE_TORQUE_HELPERS
		void addForce(Entity entity, Vector3 force);
		void addForce(Force& forceComponent, Vector3 force);
		//adds a force and calculates the torque created on that point
		void addForceAtPoint(Entity entity, Vector3 force, Vector3 point);
		void addAcceleration(Entity entity, Vector3 acceleration);
		void addAcceleration(Force& forceComponent, Mass& massComponent, Vector3 acceleration);
		void addTorque(Entity entity, Vector3 torque);
		void addTorque(Torque& torqueComponent, Vector3 torque);
#pragma endregion

#pragma region COLLIDER_HELPERS
		//changes the collider radius and updates the inertia tensor if this is adynamic body
		void changeSphereColliderRadius(Entity entity, float newRadius);
		//changes the collider size and updates the inertia tensor if this is adynamic body
		void changeBoxColliderHalfSize(Entity entity, Vector3 newHalfSize);
#pragma endregion

#pragma region BODY_HELPERS
		void changeDynamicBodyMass(Entity entity, float newMass);
		//this only works properly if the entity started its lifetime as a dynamic body (created by createDynamicBody())
		//and it needs to alternate between being on and off the physics simulation.
		//a createKinematicBody() entity will cause undefined behaviour in this method.
		//when switching from dynamic to kinematic (setToDynamic = false),
		//this does not remove the dynamic components such as LinearVelocity, AngularVelocity and Mass, but removes the DynamicBody component.
		//that component's absence prevents any system in this library from accessing the entity's dynamic components
		//so if an entity becomes kinematic it will neither be moved by the integrator, nor by the collision solver.
		void setDynamic(Entity entity, bool setToDynamic);

	private:
		void registerPhysicsComponents();
	private:
		std::shared_ptr<Registry> m_registry;
		std::unique_ptr<IIntegrationSystem> m_integrationSystem;
		std::unique_ptr<GravitySystem> m_gravitySystem;
		std::unique_ptr<ICollisionDetectionSystem> m_collisionDetectionSystem;
		std::unique_ptr<ICollisionResolutionSystem> m_collisionResolutionSystem;
		std::unique_ptr<IRaycastSystem> m_raycastSystem;
		std::vector<Collision> m_collisions;
		float m_damping;

		std::unordered_map<Entity, float> m_dynamicBodiesMasses;
	};

}