#pragma once 
#include <Physics/Colliders/BaseCollider.h>
#include <Math/Vector3.h>
namespace epl
{
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

		bool isCollidingAABBAABB(const AABBCollider& c1, const AABBCollider& c2, const Vector3& p1, const Vector3& p2,
			Vector3& normal, float& depth);

		bool isIntersectingBox(const Ray& ray, const Vector3& position, const Vector3& halfSize, RayHit& hit);

		bool isIntersectingAABB(const Ray& ray, const AABBCollider& collider, const Vector3& position, RayHit& hit);

	}
}