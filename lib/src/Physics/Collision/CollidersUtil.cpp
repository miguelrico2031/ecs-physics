#include <Physics/Collision/CollidersUtil.h>
#include <algorithm>
namespace epl
{
	bool CollidersUtil::isColliding(const SphereCollider& c1, const SphereCollider& c2, Vector3 p1, Vector3 p2)
	{
		p1 += c1.offset;
		p2 += c2.offset;
		float distance = Vector3::distance(p1, p2);
		return distance < (c1.radius + c2.radius);
	}

	bool CollidersUtil::isColliding(const AABBCollider& c1, const AABBCollider& c2, Vector3 p1, Vector3 p2)
	{
		p1 += c1.offset;
		p2 += c2.offset;
		Vector3 min1 = p1 - c1.halfSize;
		Vector3 max1 = p1 + c1.halfSize;
		Vector3 min2 = p2 - c2.halfSize;
		Vector3 max2 = p2 + c2.halfSize;
		return (min1.x <= max2.x && max1.x >= min2.x) &&
			(min1.y <= max2.y && max1.y >= min2.y) &&
			(min1.z <= max2.z && max1.z >= min2.z);
	}

	bool CollidersUtil::isColliding(const SphereCollider& sphere, const AABBCollider& aabb, Vector3 pSphere, Vector3 pAabb)
	{
		pSphere += sphere.offset;
		pAabb += aabb.offset;
		Vector3 aabbMin = pAabb - aabb.halfSize;
		Vector3 aabbMax = pAabb + aabb.halfSize;
		Vector3 closestPoint = Vector3::clamp(pSphere, aabbMin, aabbMax);
		float distSquared = Vector3::squaredMagnitude(closestPoint - pSphere);
		return distSquared <= sphere.radius * sphere.radius;
	}
}