#pragma once

#include <Physics/Colliders/ColliderType.h>
#include <Physics/Colliders/BaseCollider.h>
#include <Physics/Raycast/Ray.h>
#include <Physics/Raycast/RayHit.h>
#include <ECS/Registry.h>
#include <functional>
#include <cassert>
#include <vector>

namespace epl
{

	using CollisionCheck = std::function<bool(const BaseCollider&, const BaseCollider&, const Vector3&/*col1 pos*/, const Vector3&/*col2 pos*/,
		Vector3&/*normal (from A -> B*/, float&/*depth*/)>;

	using RayIntersectionCheck = std::function<bool(const Ray&, const BaseCollider&, const Vector3&/*position*/, RayHit&)>;

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
		void registerCollisionCheck(std::function<bool(const Collider_A&, const Collider_B&, const Vector3&, const Vector3&, Vector3&, float&)> func)
		{
			ColliderTypeID aId = ColliderType::getColliderTypeID<Collider_A>();
			ColliderTypeID bId = ColliderType::getColliderTypeID<Collider_B>();

			//function wrappers to static cast the BaseCollider refs to Collider_A and Colider_B refs
			//they work in both senses (AB and BA)
			auto abFunc = [func](const BaseCollider& c1, const BaseCollider& c2, const Vector3& p1, const Vector3& p2, Vector3& normal, float& depth) ->
				bool
				{
					const Collider_A& colA = static_cast<const Collider_A&>(c1);
					const Collider_B& colB = static_cast<const Collider_B&>(c2);
					return func(colA, colB, p1, p2, normal, depth);
				};

			auto baFunc = [func](const BaseCollider& c1, const BaseCollider& c2, const Vector3& p1, const Vector3& p2, Vector3& normal, float& depth) ->
				bool
				{
					const Collider_A& colA = static_cast<const Collider_A&>(c2);
					const Collider_B& colB = static_cast<const Collider_B&>(c1);
					bool result = func(colA, colB, p2, p1, normal, depth); //invert colliders and positions
					normal = -normal; //invert normal so it points from A -> B
					return result;
				};

			m_collisionCheckFunctions[aId][bId] = abFunc;
			m_collisionCheckFunctions[bId][aId] = baFunc;
		}

		template<class Collider_T>
		void registerRayIntersectionCheck(std::function<bool(const Ray&, const Collider_T&, const Vector3&, RayHit&)> func)
		{
			ColliderTypeID id = ColliderType::getColliderTypeID<Collider_T>();


			//function wrapper to static cast the BaseCollider ref to a Collider_T ref
			auto tFunc = [func](const Ray& ray, const BaseCollider& col, const Vector3& position, RayHit& hit) ->
				bool
				{
					const Collider_T& colT = static_cast<const Collider_T&>(col);
					return func(ray, colT, position, hit);
				};
			m_rayIntersectionCheckFunctions[id] = tFunc;
		}


		template<class Collider_A, class Collider_B>
		const std::function<bool(const Collider_A&, const Collider_B&, const Vector3&, const Vector3&, Vector3&, float&)>*
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
		const std::function<bool(const Ray&, const Collider_T&, const Vector3&, RayHit&)>*
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



	//private:
	//	template<class Collider_T>
	//	ColliderTypeID getColliderTypeID()
	//	{
	//		static ColliderTypeID id = m_nextID++;
	//		assert(id < MAX_COLLIDER_TYPES && "Too many collider types used."
	//			"To be able to use more types, do #define MAX_COLLIDER_TYPES <new number> before any #include of this file.");
	//			return id;
	//	}
	private:
		std::vector<ColliderTypeInfo> m_colliderTypesInfo;
		CollisionCheck m_collisionCheckFunctions[MAX_COLLIDER_TYPES][MAX_COLLIDER_TYPES];
		RayIntersectionCheck m_rayIntersectionCheckFunctions[MAX_COLLIDER_TYPES];
	};
}