#include <Physics/Raycast/RaycastUtil.h>

namespace epl
{

	bool RaycastUtil::isIntersecting(const Ray& ray, const SphereCollider& collider, Vector3 position, RayHit& hit)
	{
		position += collider.offset;
		Vector3 direction = position - ray.origin; //from ray origin to sphere center
		float projected = Vector3::dot(direction, ray.direction); //direction projected in the ray direction

		if (projected < 0.f)
		{
			return false; //the sphere is behind the ray
		}

		Vector3 closestPointInsideRay = ray.origin + (ray.direction * projected);

		float distanceToSphereCenter = Vector3::distance(closestPointInsideRay, position);

		if (distanceToSphereCenter > collider.radius)
		{
			return false; //closest point is outside the sphere
		}

		float offset = Math::sqrt((collider.radius * collider.radius) - (distanceToSphereCenter * distanceToSphereCenter));
		//the 2 points are at ray distance = projected + -offset, the closest one is the smallest (-)
		hit.distanceFromRayOrigin = projected - offset;
		hit.point = ray.origin + (ray.direction * hit.distanceFromRayOrigin);
		hit.colliderType = ColliderType::Sphere;
		//this method does not assign the entity to the RayHit struct
		return true;
	}

	bool RaycastUtil::isIntersecting(const Ray& ray, Vector3 position, Vector3 halfSize, RayHit& hit)
	{
		Vector3 minPos = position - halfSize;
		Vector3 maxPos = position + halfSize;

		Vector3 rayIntersectionDistances = { -1, -1, -1 };

		//this is an axis aligned bounding box. that means it consists of 6 planes, 2 for every axis
		//the ray intersects with each plane, either inside or outside of the box
		//first we discard, for every axis, the intersection with the furthest plane (2 planes per axis -> 2 intersections per axis)

		if (ray.direction.x > 0)
		{
			rayIntersectionDistances.x = (minPos.x - ray.origin.x) / ray.direction.x; //this is the closest for x plane if x ray dir is positive
		}
		else if (ray.direction.x < 0)
		{
			rayIntersectionDistances.x = (maxPos.x - ray.origin.x) / ray.direction.x; //this is the closest if x ray dir is negative
		}

		if (ray.direction.y > 0)
		{
			rayIntersectionDistances.y = (minPos.y - ray.origin.y) / ray.direction.y; //this is the closest for y plane if y ray dir is positive
		}
		else if (ray.direction.y < 0)
		{
			rayIntersectionDistances.y = (maxPos.y - ray.origin.y) / ray.direction.y; //this is the closest if y ray dir is negative
		}

		if (ray.direction.z > 0)
		{
			rayIntersectionDistances.z = (minPos.z - ray.origin.z) / ray.direction.z; //this is the closest for z plane if z ray dir is positive
		}
		else if (ray.direction.z < 0)
		{
			rayIntersectionDistances.z = (maxPos.z - ray.origin.z) / ray.direction.z; //this is the closest if z ray dir is negative
		}

		//now we have 3 intersection distances with 3 planes
		//with these and the ray's origin and direction vectors we can get the 3 intersection points
		//but we don't need the 3, because we know the furthest point from those 3 is the only one that could be on the box's surface
		//how do we know it? hard to explain, if you draw a 2d box and a ray is easier to understand visually
		float possibleIntersectionDistance = Math::max(Math::max(
			rayIntersectionDistances.x,
			rayIntersectionDistances.y),
			rayIntersectionDistances.z);
		if (possibleIntersectionDistance < 0.f)
		{
			return false; //the box is behind the ray
		}
		Vector3 possibleIntersectionPoint = ray.origin + (ray.direction * possibleIntersectionDistance);

		const float epsilon = Math::epsilon();
		if ((possibleIntersectionPoint.x + epsilon < minPos.x || possibleIntersectionPoint.x - epsilon > maxPos.x) ||
			(possibleIntersectionPoint.y + epsilon < minPos.y || possibleIntersectionPoint.y - epsilon > maxPos.y) ||
			(possibleIntersectionPoint.z + epsilon < minPos.z || possibleIntersectionPoint.z - epsilon > maxPos.z))
		{
			return false; //the intersection point is outside the box
		}

		hit.point = possibleIntersectionPoint;
		hit.distanceFromRayOrigin = possibleIntersectionDistance;
		return true;
	}

	bool RaycastUtil::isIntersecting(const Ray& ray, const AABBCollider& collider, Vector3 position, RayHit& hit)
	{
		return isIntersecting(ray, position + collider.offset, collider.halfSize, hit);
	}
}

