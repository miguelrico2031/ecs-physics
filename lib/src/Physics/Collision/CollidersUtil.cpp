#include <Physics/Collision/CollidersUtil.h>
#include <Math/Math.h>
#include <algorithm>
namespace epl
{
	bool CollidersUtil::isColliding(const SphereCollider& c1, const SphereCollider& c2, Vector3 p1, Vector3 p2, 
		Vector3& normal, float& depth)
	{
		p1 += c1.offset;
		p2 += c2.offset;
		Vector3 delta = p2 - p1;
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

	bool CollidersUtil::isColliding(const AABBCollider& c1, const AABBCollider& c2, Vector3 p1, Vector3 p2,
		Vector3& normal, float& depth)
	{
		p1 += c1.offset;
		p2 += c2.offset;
		Vector3 min1 = p1 - c1.halfSize;
		Vector3 max1 = p1 + c1.halfSize;
		Vector3 min2 = p2 - c2.halfSize;
		Vector3 max2 = p2 + c2.halfSize;

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

		Vector3 delta = p2 - p1;
		if (Vector3::dot(delta, normal) < 0)
		{
			normal = -normal; // Make sure normal points from c1 to c2
		}
		depth = minOverlap;
		return true;
	}

	bool CollidersUtil::isColliding(const SphereCollider& sphere, const AABBCollider& aabb, Vector3 pSphere, Vector3 pAabb, 
		Vector3& normal, float& depth)
	{
		pSphere += sphere.offset;
		pAabb += aabb.offset;
		Vector3 aabbMin = pAabb - aabb.halfSize;
		Vector3 aabbMax = pAabb + aabb.halfSize;
		Vector3 closestPoint = Vector3::clamp(pSphere, aabbMin, aabbMax);
		Vector3 delta = closestPoint - pSphere;
		float distSquared = Vector3::squaredMagnitude(delta);
		if (distSquared > sphere.radius * sphere.radius)
		{
			return false;
		}

		float distance = Math::sqrt(distSquared);

		if(distance > Math::epsilon())
		{
			normal = delta / distance;
			depth = sphere.radius - distance;
		}
		else //sphere center is inside the AABB
		{
			Vector3 directionFromCenter = pSphere - pAabb;
			Vector3 absDirectionFromCenter = Vector3::abs(directionFromCenter);

			if(absDirectionFromCenter.x > absDirectionFromCenter.y && absDirectionFromCenter.x > absDirectionFromCenter.z)
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
}