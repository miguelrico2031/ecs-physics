#pragma once

#include <Physics/Colliders/ColliderType.h>
#include <Physics/Colliders/BaseCollider.h>
#include <Physics/Raycast/Ray.h>
#include <Physics/Raycast/RayHit.h>
#include <Physics/Collision/Collision.h>
#include <ECS/Registry.h>
#include <functional>
#include <cassert>
#include <vector>

namespace epl
{

	using CollisionCheck = std::function<bool(const Registry&, const BaseCollider&, const BaseCollider&, 
		Entity/*col1 entity*/, Entity/*col2 entity*/, Collision&)>;

	using RayIntersectionCheck = std::function<bool(const Registry&, const Ray&, const BaseCollider&, Entity, RayHit&)>;

	using ColliderForEach = std::function<void(Entity, const BaseCollider&)>;
	using ColliderIteratorFunc = std::function<void(const Registry&, ColliderForEach)>;
	
	struct ColliderTypeInfo
	{
		ColliderTypeID id;
		ColliderIteratorFunc forEachColliderOfThisType;
	};

	class ColliderRegistry
	{
	public:
		ColliderRegistry()
		{
			m_colliderTypesInfo.reserve(MAX_COLLIDER_TYPES);
		}

		//constexpr size_t maxColliderTypes() const { return MAX_COLLIDER_TYPES; }

		template<class Collider_T>
		void registerColliderType()
		{
			ColliderTypeID id = ColliderType::getColliderTypeID<Collider_T>();
			ColliderIteratorFunc func = [](const Registry& reg, ColliderForEach forEachFunc)
				{
					for (const auto& [entity, collider] : reg.iterate<Collider_T>())
					{
						forEachFunc(entity, collider);
					}
				};
			m_colliderTypesInfo.emplace_back(ColliderTypeInfo{ id, func });
		}

		template<class Collider_A, class Collider_B>
		void registerCollisionCheck(std::function<bool(const Registry&, const Collider_A&, const Collider_B&, Entity, Entity, Collision&)> func)
		{
			ColliderTypeID aId = ColliderType::getColliderTypeID<Collider_A>();
			ColliderTypeID bId = ColliderType::getColliderTypeID<Collider_B>();

			//function wrappers to static cast the BaseCollider refs to Collider_A and Colider_B refs
			//they work in both senses (AB and BA)
			auto abFunc = [func](const Registry& reg, const BaseCollider& c1, const BaseCollider& c2, Entity e1, Entity e2, Collision& col) ->
				bool
				{
					const Collider_A& colA1 = static_cast<const Collider_A&>(c1);
					const Collider_B& colB2 = static_cast<const Collider_B&>(c2);
					return func(reg, colA1, colB2, e1, e2, col);
				};

			auto baFunc = [func](const Registry& reg, const BaseCollider& c1, const BaseCollider& c2, Entity e1, Entity e2, Collision& col) ->
				bool
				{
					const Collider_A& colA2 = static_cast<const Collider_A&>(c2);
					const Collider_B& colB1 = static_cast<const Collider_B&>(c1);
					bool result = func(reg, colA2, colB1, e2, e1, col); //invert colliders and positions
					col.normal *= -1; //invert normal so it points from A -> B
					std::swap(col.entity1, col.entity2); //swap entities so they are in the correct order
					return result;
				};

			m_collisionCheckFunctions[aId][bId] = abFunc;
			m_collisionCheckFunctions[bId][aId] = baFunc;
		}

		template<class Collider_T>
		void registerRayIntersectionCheck(std::function<bool(const Registry&, const Ray&, const Collider_T&, Entity, RayHit&)> func)
		{
			ColliderTypeID id = ColliderType::getColliderTypeID<Collider_T>();

			//function wrapper to static cast the BaseCollider ref to a Collider_T ref
			auto tFunc = [func](const Registry& reg, const Ray& ray, const BaseCollider& col, Entity e, RayHit& hit) ->
				bool
				{
					const Collider_T& colT = static_cast<const Collider_T&>(col);
					return func(reg, ray, colT, e, hit);
				};
			m_rayIntersectionCheckFunctions[id] = tFunc;
		}


		template<class Collider_A, class Collider_B>
		const std::function<bool(const Registry&, const Collider_A&, const Collider_B&, Entity, Entity, Collision&)>*
			getCollisionCheck() const 
		{
			ColliderTypeID aId = ColliderType::getColliderTypeID<Collider_A>();
			ColliderTypeID bId = ColliderType::getColliderTypeID<Collider_B>();

			auto& func = m_collisionCheckFunctions[aID][bID];
			return func ? &func : nullptr;
		}

		const CollisionCheck* getCollisionCheck(ColliderTypeID aID, ColliderTypeID bID) const
		{
			auto& func = m_collisionCheckFunctions[aID][bID];
			return func ? &func : nullptr;
		}


		template<class Collider_T>
		const std::function<bool(const Registry&, const Ray&, const Collider_T&, Entity, RayHit&)>*
			getRayIntersectionCheck() const
		{
			ColliderTypeID id = ColliderType::getColliderTypeID<Collider_T>();
			auto& func = m_rayIntersectionCheckFunctions[id];
			return func ? &func : nullptr;
		}

		const RayIntersectionCheck* getRayIntersectionCheck(ColliderTypeID id) const
		{
			auto& func = m_rayIntersectionCheckFunctions[id];
			return func ? &func : nullptr;
		}

		const std::vector<ColliderTypeInfo>& getAllTypes() const { return m_colliderTypesInfo; }


	private:
		std::vector<ColliderTypeInfo> m_colliderTypesInfo;
		CollisionCheck m_collisionCheckFunctions[MAX_COLLIDER_TYPES][MAX_COLLIDER_TYPES];
		RayIntersectionCheck m_rayIntersectionCheckFunctions[MAX_COLLIDER_TYPES];
	};
}