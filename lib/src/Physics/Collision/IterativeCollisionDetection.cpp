#include <Physics/Collision/IterativeCollisionDetection.h>
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>
#include <Physics/Colliders/ColliderFuncs.h>

namespace epl
{
	void IterativeCollisionDetection::detectCollisions(const Registry& reg, std::vector<Collision>& collisions)
	{
		for (const auto [entity1, obb1] : reg.iterate<OBBCollider>())
		{
			for (const auto [entity2, obb2] : reg.iterate<OBBCollider>())
			{
				if (entity1 >= entity2) continue;
				Collision collision;
				if (ColliderFuncs::isCollidingOBBOBB(reg, obb1, obb2, entity1, entity2, collision))
				{
					collisions.push_back(collision);
				}
			}
		}
		for (const auto [entity1, sphereCol1] : reg.iterate<SphereCollider>())
		{
			for (const auto [entity2, obb2] : reg.iterate<OBBCollider>())
			{
				if (entity1 >= entity2) continue;
				Collision collision;
				if (ColliderFuncs::isCollidingSphereOBB(reg, sphereCol1, obb2, entity1, entity2, collision))
				{
					collisions.push_back(collision);
				}
			}
		}
	}
}