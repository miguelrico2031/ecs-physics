#pragma once
#include <Physics/Colliders/BaseCollider.h>


namespace epl
{
	class Registry;
	struct Ray;
	struct RayHit;
	struct AABBCollider;

	struct SphereCollider : public BaseCollider
	{
		float radius;
		SphereCollider(float radius_, const Vector3& offset_ = Vector3::zero())
			: BaseCollider(ColliderType::getColliderTypeID<SphereCollider>()), radius(radius_)
		{
		}
	};

	namespace SphereColliderFuncs
	{

		bool isCollidingSphereSphere(const Registry& reg, const SphereCollider& c1, const SphereCollider& c2, Entity e1, Entity e2, Collision& col);

		bool isCollidingSphereAABB(const Registry& reg, const SphereCollider& c1, const AABBCollider& c2, Entity e1, Entity e2, Collision& col);

		bool isCollidingSphereBox(const Vector3& spherePosition, float sphereRadius, const Vector3& boxPosition, const Vector3& boxHalfSize,
			Collision& col);

		bool isIntersectingSphere(const Registry& reg, const Ray& ray, const SphereCollider& collider, Entity entity, RayHit& hit);

		Matrix3x3 calculateInverseInertiaTensor(float radius, float inverseMass);

	}

}