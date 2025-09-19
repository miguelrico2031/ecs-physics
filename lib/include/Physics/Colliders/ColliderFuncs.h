#include <Physics/Colliders/ColliderBounds.h>
#include <Physics/Colliders/OBBCollider.h>
#include <Physics/Colliders/SphereCollider.h>
#include <Physics/Colliders/AABBCollider.h>
#include <ECS/Entity.h>
#include <Math/Matrix3x3.h>

namespace epl
{
	class Registry;
	struct Ray;
	struct RayHit;
	struct Collision;

	namespace ColliderFuncs
	{
#pragma region HELPERS
		bool isIntersectingBox(const Ray& ray, const Vector3& position, const Vector3& halfSize, RayHit& hit);
		bool isPointInsideBox(const Vector3& point, const Vector3& halfSize);
		bool testSeparatingAxis(Vector3 axis, const Vector3 box1Axes[3],
			const Vector3& box1HalfSize, const Vector3 box2Axes[3],
			const Vector3& box2HalfSize, const Vector3& direction, Collision& col);
		bool testAllSeparatingAxes(const Vector3 box1Axes[3], const Vector3 box2Axes[3], const Vector3& box1HalfSize,
			const Vector3& box2HalfSize, const Vector3& direction, Collision& col);
		float projectBox(const Vector3& axisToProject, const Vector3 boxAxes[3], const Vector3& boxHalfSize);
		void getBoxVertices(const Vector3& position, const Vector3& halfSize, const Vector3 axes[3], Vector3 vertices[8]);
		bool isCollidingSphereBox(const Vector3& spherePosition, float sphereRadius, const Vector3& boxPosition, const Vector3& boxHalfSize,
			Collision& col);
#pragma endregion	

#pragma region INTERSECTION_COLLISION
		bool isIntersectingOBB(const Registry& reg, const Ray& ray, const OBBCollider& collider, Entity entity, RayHit& hit);
		bool isIntersectingSphere(const Registry& reg, const Ray& ray, const SphereCollider& collider, Entity entity, RayHit& hit);
		bool isIntersectingAABB(const Registry& reg, const Ray& ray, const AABBCollider& collider, Entity entity, RayHit& hit);
		
		bool isCollidingOBBOBB(const Registry& reg, const OBBCollider& c1, const OBBCollider& c2, Entity e1, Entity e2, Collision& col);
		bool isCollidingSphereOBB(const Registry& reg, const SphereCollider& c1, const OBBCollider& c2, Entity e1, Entity e2, Collision& col);
		bool isCollidingOBBAABB(const Registry& reg, const OBBCollider& c1, const AABBCollider& c2, Entity e1, Entity e2, Collision& col);
		bool isCollidingSphereSphere(const Registry& reg, const SphereCollider& c1, const SphereCollider& c2, Entity e1, Entity e2, Collision& col);
		bool isCollidingSphereAABB(const Registry& reg, const SphereCollider& c1, const AABBCollider& c2, Entity e1, Entity e2, Collision& col);
		bool isCollidingAABBAABB(const Registry& reg, const AABBCollider& c1, const AABBCollider& c2, Entity e1, Entity e2, Collision& col);
#pragma endregion
		
#pragma region INERTIA_TENSOR
		Matrix3x3 calculateBoxInverseInertiaTensor(const Vector3& halfSize, float inverseMass);
		Matrix3x3 calculateRotatedBoxInverseInertiaTensor(const Matrix3x3& localInvInertia, const Quaternion& rotation);
		Matrix3x3 calculateSphereInverseInertiaTensor(float radius, float inverseMass);
#pragma endregion

#pragma region BOUNDS
		void calculateSphereBounds(float radius, ColliderBounds& bounds);
		void calculateBoxBounds(Vector3 halfSize, ColliderBounds& bounds);
#pragma endregion
	}
}