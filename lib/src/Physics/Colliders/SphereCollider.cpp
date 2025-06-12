#include <Physics/Colliders/SphereCollider.h>
#include <Physics/Colliders/AABBCollider.h>
#include <Physics/Raycast/Ray.h>
#include <Physics/Raycast/RayHit.h>

namespace epl
{
	bool SphereColliderFuncs::isCollidingSphereSphere(const SphereCollider& c1, const SphereCollider& c2, const Vector3& p1, const Vector3& p2,
		Vector3& normal, float& depth)
	{
		Vector3 pos1 = p1 + c1.offset;
		Vector3 pos2 = p2 + c2.offset;
		Vector3 delta = pos2 - pos1;
		float distance = Vector3::magnitude(delta);
		float totalRadius = c1.radius + c2.radius;
		if (distance >= totalRadius)
		{
			return false;
		}

		if (distance > Math::epsilon())
		{
			normal = delta / distance;
			depth = totalRadius - distance;
		}
		else
		{
			normal = { 0, 1, 0 };
			depth = totalRadius;
		}

		return true;
	}

	bool SphereColliderFuncs::isCollidingSphereAABB(const SphereCollider& sphere, const AABBCollider& aabb, const Vector3& pSphere, const Vector3& pAabb,
		Vector3& normal, float& depth)
	{
		Vector3 spherePosition = pSphere + sphere.offset;
		Vector3 aabbPosition = pAabb + aabb.offset;
		Vector3 aabbMin = aabbPosition - aabb.halfSize;
		Vector3 aabbMax = aabbPosition + aabb.halfSize;
		Vector3 closestPoint = Vector3::clamp(spherePosition, aabbMin, aabbMax);
		Vector3 delta = closestPoint - spherePosition;
		float distSquared = Vector3::squaredMagnitude(delta);
		if (distSquared > sphere.radius * sphere.radius)
		{
			return false;
		}

		float distance = Math::sqrt(distSquared);

		if (distance > Math::epsilon())
		{
			normal = delta / distance;
			depth = sphere.radius - distance;
		}
		else //sphere center is inside the AABB
		{
			Vector3 directionFromCenter = spherePosition - aabbPosition;
			Vector3 absDirectionFromCenter = Vector3::abs(directionFromCenter);

			if (absDirectionFromCenter.x > absDirectionFromCenter.y && absDirectionFromCenter.x > absDirectionFromCenter.z)
			{
				normal = { directionFromCenter.x > 0.f ? 1.f : -1.f, 0, 0 };
			}
			else if (absDirectionFromCenter.y > absDirectionFromCenter.z)
			{
				normal = { 0, directionFromCenter.y > 0.f ? 1.f : -1.f, 0 };
			}
			else
			{
				normal = { 0, 0, directionFromCenter.z > 0.f ? 1.f : -1.f };
			}

			depth = sphere.radius;
		}
		return true;
	}


	bool SphereColliderFuncs::isIntersectingSphere(const Ray& ray, const SphereCollider& collider, const Vector3& position, RayHit& hit)
	{
		Vector3 spherePos = position + collider.offset;
		Vector3 direction = spherePos - ray.origin; //from ray origin to sphere center
		float projected = Vector3::dot(direction, ray.direction); //direction projected in the ray direction

		if (projected < 0.f)
		{
			return false; //the sphere is behind the ray
		}

		Vector3 closestPointInsideRay = ray.origin + (ray.direction * projected);

		float distanceToSphereCenter = Vector3::distance(closestPointInsideRay, spherePos);

		if (distanceToSphereCenter > collider.radius)
		{
			return false; //closest point is outside the sphere
		}

		float offset = Math::sqrt((collider.radius * collider.radius) - (distanceToSphereCenter * distanceToSphereCenter));
		//the 2 points are at ray distance = projected + -offset, the closest one is the smallest (-)
		hit.distanceFromRayOrigin = projected - offset;
		hit.point = ray.origin + (ray.direction * hit.distanceFromRayOrigin);
		//this method does not assign the entity to the RayHit struct
		return true;
	}
}