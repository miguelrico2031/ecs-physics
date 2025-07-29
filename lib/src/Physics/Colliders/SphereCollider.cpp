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
		Vector3 pos1 = reg.getComponent<Position>(e1).value;
		Vector3 pos2 = reg.getComponent<Position>(e2).value;
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
			Vector3 contactPoint1 = pos1 + col.normal * c1.radius;
			Vector3 contactPoint2 = pos2 - col.normal * c2.radius;
			col.contactPoint = (contactPoint1 + contactPoint2) * .5f;
			col.depth = totalRadius - distance;
		}
		else //the spheres at almost entirely overlapped
		{
			col.normal = { 0, 1, 0 }; //arbitrary normal
			col.contactPoint = (pos1 + pos2) * .5f;
			col.depth = totalRadius;

		}
		col.entity1 = e1;
		col.entity2 = e2;
		return true;
	}

	bool SphereColliderFuncs::isCollidingSphereAABB(const Registry& reg, const SphereCollider& c1, const AABBCollider& c2, 
		Entity e1, Entity e2, Collision& col)
	{
		Vector3 spherePosition = reg.getComponent<Position>(e1).value;
		Vector3 aabbPosition = reg.getComponent<Position>(e2).value;
		
		if (!isCollidingSphereBox(spherePosition, c1.radius, aabbPosition, c2.halfSize, col))
		{
			return false;
		}
		col.entity1 = e1;
		col.entity2 = e2;
		return true;
	}


	bool SphereColliderFuncs::isCollidingSphereBox(const Vector3& spherePosition, float sphereRadius, const Vector3& boxPosition,
		const Vector3& boxHalfSize, Collision& col)
	{
		Vector3 aabbMin = boxPosition - boxHalfSize;
		Vector3 aabbMax = boxPosition + boxHalfSize;
		Vector3 closestPoint = Vector3::clamp(spherePosition, aabbMin, aabbMax);
		Vector3 delta = closestPoint - spherePosition;
		float distSquared = Vector3::squaredMagnitude(delta);
		if (distSquared > sphereRadius * sphereRadius)
		{
			return false;
		}

		float distance = Math::sqrt(distSquared);

		if (distance > Math::epsilon())
		{
			col.normal = delta / distance;
			col.depth = sphereRadius - distance;
			col.contactPoint = closestPoint;
		}
		else //sphere center is inside the AABB
		{
			Vector3 directionFromCenter = spherePosition - boxPosition;
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

			col.depth = sphereRadius;
			col.contactPoint = spherePosition + col.normal * sphereRadius;
		}
		return true;
	}


	bool SphereColliderFuncs::isIntersectingSphere(const Registry& reg, const Ray& ray, const SphereCollider& collider, 
		Entity entity, RayHit& hit)
	{
		Vector3 spherePos = reg.getComponent<Position>(entity).value;
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


	Matrix3x3 SphereColliderFuncs::calculateInverseInertiaTensor(float radius, float inverseMass)
	{
		float diagonal = (5 * inverseMass) / (2 * radius * radius);
		return Matrix3x3(Vector3{diagonal, diagonal, diagonal});
	}
}