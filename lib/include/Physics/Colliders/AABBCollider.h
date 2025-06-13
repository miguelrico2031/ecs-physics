#pragma once 
#include <Physics/Colliders/BaseCollider.h>


namespace epl
{
	class Registry;
	struct Ray;
	struct RayHit;
	struct AABBCollider : public BaseCollider
	{
		Vector3 halfSize;
		Vector3 offset;
		AABBCollider(const Vector3& halfSize_, const Vector3& offset_ = Vector3::zero())
			: BaseCollider(ColliderType::getColliderTypeID<AABBCollider>()), halfSize(halfSize_), offset(offset_) {}
	};

	namespace AABBColliderFuncs
	{

		bool isCollidingAABBAABB(const Registry& reg, const AABBCollider& c1, const AABBCollider& c2, Entity e1, Entity e2, Collision& col);

		bool isIntersectingBox(const Ray& ray, const Vector3& position, const Vector3& halfSize, RayHit& hit);

		bool isIntersectingAABB(const Registry& reg, const Ray& ray, const AABBCollider& collider, Entity entity, RayHit& hit);

		Matrix3x3 calculateInverseInertiaTensor(const Vector3& halfSize, float inverseMass);
	}
}