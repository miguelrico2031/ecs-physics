#include <Physics/Colliders/OBBCollider.h>
#include <Physics/Colliders/SphereCollider.h>
#include <Physics/Colliders/AABBCollider.h>
#include <Physics/Raycast/Ray.h>
#include <Physics/Raycast/RayHit.h>
#include <Physics/Motion/MotionComponents.h>
#include <ECS/Registry.h>
#include <Math/Matrix3x3.h>
#include <iostream>

namespace epl
{
	bool OBBColliderFuncs::isCollidingOBBOBB(const Registry& reg, const OBBCollider& c1, const OBBCollider& c2, 
		Entity e1, Entity e2, Collision& col)
	{
		std::cerr << "error: collision not implemented: isCollidingOBBOBB.\n";
		return false;
	}


	bool OBBColliderFuncs::isCollidingOBBAABB(const Registry& reg, const OBBCollider& c1, const AABBCollider& c2, 
		Entity e1, Entity e2, Collision& col)
	{
		std::cerr << "error: collision not implemented: isCollidingOBBAABB.\n";
		return false;
	}


	bool OBBColliderFuncs::isCollidingOBBSphere(const Registry& reg, const OBBCollider& c1, const SphereCollider& c2, 
		Entity e1, Entity e2, Collision& col)
	{
		std::cerr << "error: collision not implemented: isCollidingOBBSphere.\n";
		return false;
	}



	bool OBBColliderFuncs::isIntersectingOBB(const Registry& reg, const Ray& ray, const OBBCollider& collider, Entity entity, RayHit& hit)
	{
		//this method does a vector space basis change to align the axes with the box roation and place the box at local 0 0 0.
		//doing this allows us to treat the box as an AABB and use it's intersection check function.
		//to avoid using 4x4 transform matrices we use 3x3s to perform the rotations of the basis change, and for the translation transform
		//we translate the ray origin by subtracting the box origin, and then adding it again to the hit point
		//scale is not supported (yet) so this way we do the same as 4x4 transform matrices with 3x3s and a vector 
		
		Vector3 position = reg.getComponent<Position>(entity).value + collider.offset;
		Quaternion rotation = reg.getComponent<Rotation>(entity).value;
		Matrix3x3 localToWorldSpaceTransform = Matrix3x3(rotation);
		Matrix3x3 worldToLocalSpaceTransform = Matrix3x3(Quaternion::conjugate(rotation));

		Vector3 localRayOrigin = ray.origin - position;

		Ray rayInLocalSpace = { worldToLocalSpaceTransform * localRayOrigin, worldToLocalSpaceTransform * ray.direction };
		bool intersected = AABBColliderFuncs::isIntersectingBox(rayInLocalSpace, Vector3::zero(), collider.halfSize, hit);

		if (intersected)
		{
			hit.point = (localToWorldSpaceTransform * hit.point) + position;
		}
		return intersected;
	}
}