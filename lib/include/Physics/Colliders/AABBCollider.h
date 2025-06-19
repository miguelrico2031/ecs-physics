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
		AABBCollider(const Vector3& halfSize_)
			: BaseCollider(ColliderType::getColliderTypeID<AABBCollider>()), halfSize(halfSize_)
		{
		}
	};

	namespace AABBColliderFuncs
	{

		bool isCollidingAABBAABB(const Registry& reg, const AABBCollider& c1, const AABBCollider& c2, Entity e1, Entity e2, Collision& col);

		bool isIntersectingBox(const Ray& ray, const Vector3& position, const Vector3& halfSize, RayHit& hit);

		bool isIntersectingAABB(const Registry& reg, const Ray& ray, const AABBCollider& collider, Entity entity, RayHit& hit);

		Matrix3x3 calculateBoxInverseInertiaTensor(const Vector3& halfSize, float inverseMass);
	}
}