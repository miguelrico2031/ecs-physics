#include <Physics/Colliders/SphereCollider.h>
#include <Physics/Colliders/AABBCollider.h>
#include <Physics/Raycast/Ray.h>
#include <Physics/Raycast/RayHit.h>
#include <Physics/Motion/MotionComponents.h>
#include <ECS/Registry.h>


namespace epl
{
	bool SphereColliderFuncs::isCollidingSphereSphere(const Registry& reg, const SphereCollider& c1, const SphereCollider& c2, 
		Entity e1, Entity e2, Collision& col)
	{
		Position p1 = reg.getComponent<Position>(e1);
		Position p2 = reg.getComponent<Position>(e2);
		Vector3 pos1 = p1.value + c1.offset;
		Vector3 pos2 = p2.value + c2.offset;
		Vector3 delta = pos2 - pos1;
		float distance = Vector3::magnitude(delta);
		float totalRadius = c1.radius + c2.radius;
		if (distance >= totalRadius)
		{
			return false;
		}

		if (distance > Math::epsilon())
		{
			col.normal = delta / distance;
			col.depth = totalRadius - distance;
		}
		else
		{
			col.normal = { 0, 1, 0 };
			col.depth = totalRadius;
		}

		return true;
	}

	bool SphereColliderFuncs::isCollidingSphereAABB(const Registry& reg, const SphereCollider& c1, const AABBCollider& c2, Entity e1, Entity e2, Collision& col)
	{
		Position pSphere = reg.getComponent<Position>(e1);
		Position pAabb = reg.getComponent<Position>(e2);
		Vector3 spherePosition = pSphere.value + c1.offset;
		Vector3 aabbPosition = pAabb.value + c2.offset;
		Vector3 aabbMin = aabbPosition - c2.halfSize;
		Vector3 aabbMax = aabbPosition + c2.halfSize;
		Vector3 closestPoint = Vector3::clamp(spherePosition, aabbMin, aabbMax);
		Vector3 delta = closestPoint - spherePosition;
		float distSquared = Vector3::squaredMagnitude(delta);
		if (distSquared > c1.radius * c1.radius)
		{
			return false;
		}

		float distance = Math::sqrt(distSquared);

		if (distance > Math::epsilon())
		{
			col.normal = delta / distance;
			col.depth = c1.radius - distance;
		}
		else //sphere center is inside the AABB
		{
			Vector3 directionFromCenter = spherePosition - aabbPosition;
			Vector3 absDirectionFromCenter = Vector3::abs(directionFromCenter);

			if (absDirectionFromCenter.x > absDirectionFromCenter.y && absDirectionFromCenter.x > absDirectionFromCenter.z)
			{
				col.normal = { directionFromCenter.x > 0.f ? 1.f : -1.f, 0, 0 };
			}
			else if (absDirectionFromCenter.y > absDirectionFromCenter.z)
			{
				col.normal = { 0, directionFromCenter.y > 0.f ? 1.f : -1.f, 0 };
			}
			else
			{
				col.normal = { 0, 0, directionFromCenter.z > 0.f ? 1.f : -1.f };
			}

			col.depth = c1.radius;
		}
		return true;
	}


	bool SphereColliderFuncs::isIntersectingSphere(const Registry& reg, const Ray& ray, const SphereCollider& collider, 
		Entity entity, RayHit& hit)
	{
		Position position = reg.getComponent<Position>(entity);
		Vector3 spherePos = position.value + collider.offset;
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