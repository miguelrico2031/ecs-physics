#pragma once
#include <Physics/Raycast/Ray.h>
#include <Physics/Raycast/RayHit.h>

namespace epl::RaycastUtil
{
	bool isIntersecting(const Ray& ray, const SphereCollider& collider, Vector3 position, RayHit& hit);
	bool isIntersecting(const Ray& ray, Vector3 position, Vector3 halfSize, RayHit& hit);
	bool isIntersecting(const Ray& ray, const AABBCollider& collider, Vector3 position, RayHit& hit);

}