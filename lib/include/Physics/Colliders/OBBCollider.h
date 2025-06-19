#pragma once 
#include <Physics/Colliders/BaseCollider.h>
#include <Math/Quaternion.h>
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
		OBBCollider(const Vector3& halfSize_)
			: BaseCollider(ColliderType::getColliderTypeID<OBBCollider>()), halfSize(halfSize_)
		{
		}
	};

	namespace OBBColliderFuncs
	{
		bool isCollidingOBBOBB(const Registry& reg, const OBBCollider& c1, const OBBCollider& c2, Entity e1, Entity e2, Collision& col);

		bool isCollidingOBBAABB(const Registry& reg, const OBBCollider& c1, const AABBCollider& c2, Entity e1, Entity e2, Collision& col);

		//this check has sphere as the collider 1 because SphereColliderFuncs::isCollidingSphereBox has that order too
		bool isCollidingSphereOBB(const Registry& reg, const SphereCollider& c1, const OBBCollider& c2, Entity e1, Entity e2, Collision& col);

		bool isIntersectingOBB(const Registry& reg, const Ray& ray, const OBBCollider& collider, Entity entity, RayHit& hit);
	

		bool testSeparatingAxis(Vector3 axis, const Vector3 box1Axes[3],
			const Vector3& box1HalfSize, const Vector3 box2Axes[3],
			const Vector3& box2HalfSize, const Vector3& direction, Collision& col);
		
		bool testAllSeparatingAxes(const Vector3 box1Axes[3], const Vector3 box2Axes[3], const Vector3& box1HalfSize,
			const Vector3& box2HalfSize, const Vector3& direction, Collision& col);

		float projectBox(const Vector3& axisToProject, const Vector3 boxAxes[3], const Vector3& boxHalfSize);


		Matrix3x3 calculateRotatedInverseInertiaTensor(const Matrix3x3& localInvInertia, const Quaternion& rotation);

	}

	using BoxCollider = OBBCollider; //default box collider to use in gameplay (AABB is used for broadphase and other internal calculations)
}