#pragma once
#include <Physics/Collision/ColliderComponents.h>

namespace epl::CollidersUtil
{
	bool isColliding(const SphereCollider& c1, const SphereCollider& c2, Vector3 p1, Vector3 p2);
	bool isColliding(const AABBCollider& c1, const AABBCollider& c2, Vector3 p1, Vector3 p2);
	bool isColliding(const SphereCollider& c1, const AABBCollider& c2, Vector3 p1, Vector3 p2);
}