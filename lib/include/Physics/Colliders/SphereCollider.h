#pragma once
#include <Physics/Colliders/BaseCollider.h>
#include <Math/Vector3.h>

namespace epl
{
	struct Ray;
	struct RayHit;
	struct AABBCollider;

	struct SphereCollider : public BaseCollider
	{
		float radius;
		Vector3 offset;
		SphereCollider(float radius_, const Vector3& offset_ = Vector3::zero())
			: BaseCollider(ColliderType::getColliderTypeID<SphereCollider>()), radius(radius_), offset(offset_) {}
	};

	namespace SphereColliderFuncs
	{

	bool isCollidingSphereSphere(const SphereCollider& c1, const SphereCollider& c2, const Vector3& p1, const Vector3& p2,
		Vector3& normal, float& depth);

	bool isCollidingSphereAABB(const SphereCollider& c1, const AABBCollider& c2, const Vector3& p1, const Vector3& p2,
		Vector3& normal, float& depth);

	bool isIntersectingSphere(const Ray& ray, const SphereCollider& collider, const Vector3& position, RayHit& hit);
	}

}