#pragma once
#include <Physics/Colliders/AABBCollider.h>
#include <Physics/Raycast/Ray.h>
#include <Physics/Raycast/RayHit.h>
namespace epl
{
	bool AABBColliderFuncs::isCollidingAABBAABB(const AABBCollider& c1, const AABBCollider& c2, const Vector3& p1, const Vector3& p2,
		Vector3& normal, float& depth)
	{
		Vector3 pos1 = p1 + c1.offset;
		Vector3 pos2 = p2 + c2.offset;
		Vector3 min1 = pos1 - c1.halfSize;
		Vector3 max1 = pos1 + c1.halfSize;
		Vector3 min2 = pos2 - c2.halfSize;
		Vector3 max2 = pos2 + c2.halfSize;

		//return (min1.x <= max2.x && max1.x >= min2.x) &&
		//	(min1.y <= max2.y && max1.y >= min2.y) &&
		//	(min1.z <= max2.z && max1.z >= min2.z);

		Vector3 overlap = Vector3::min(max1, max2) - Vector3::max(min1, min2);

		if (overlap.x <= 0 || overlap.y <= 0 || overlap.z <= 0)
		{
			return false;
		}

		float minOverlap = overlap.x;
		normal = { 1, 0, 0 };
		if (overlap.y < minOverlap)
		{
			minOverlap = overlap.y;
			normal = { 0, 1, 0 };
		}
		if (overlap.z < minOverlap)
		{
			minOverlap = overlap.z;
			normal = { 0, 0, 1 };
		}

		Vector3 delta = pos2 - pos1;
		if (Vector3::dot(delta, normal) < 0)
		{
			normal = -normal; // Make sure normal points from c1 to c2
		}
		depth = minOverlap;
		return true;
	}

	bool AABBColliderFuncs::isIntersectingBox(const Ray& ray, const Vector3& position, const Vector3& halfSize, RayHit& hit)
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

	bool AABBColliderFuncs::isIntersectingAABB(const Ray& ray, const AABBCollider& collider, const Vector3& position, RayHit& hit)
	{
		return isIntersectingBox(ray, position + collider.offset, collider.halfSize, hit);
	}
}