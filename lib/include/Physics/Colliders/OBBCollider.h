#pragma once 
#include <Physics/Colliders/BaseCollider.h>

namespace epl
{
	class Registry;
	struct Ray;
	struct RayHit;
	struct SphereCollider;
	struct AABBCollider;

	struct OBBCollider : public BaseCollider
	{
		Vector3 halfSize;
		Vector3 offset;
		OBBCollider(const Vector3& halfSize_, const Vector3& offset_ = Vector3::zero())
			: BaseCollider(ColliderType::getColliderTypeID<OBBCollider>()), halfSize(halfSize_), offset(offset_)
		{
		}
	};

	namespace OBBColliderFuncs
	{
		bool isCollidingOBBOBB(const Registry& reg, const OBBCollider& c1, const OBBCollider& c2, Entity e1, Entity e2, Collision& col);

		bool isCollidingOBBAABB(const Registry& reg, const OBBCollider& c1, const AABBCollider& c2, Entity e1, Entity e2, Collision& col);

		bool isCollidingOBBSphere(const Registry& reg, const OBBCollider& c1, const SphereCollider& c2, Entity e1, Entity e2, Collision& col);

		bool isIntersectingOBB(const Registry& reg, const Ray& ray, const OBBCollider& collider, Entity entity, RayHit& hit);
	}

	using BoxCollider = OBBCollider; //default box collider to use in gameplay (AABB is used for broadphase and other internal calculations)
}