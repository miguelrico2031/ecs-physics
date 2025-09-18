#include <Physics/Collision/IterativeCollisionDetection.h>
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>
#include <Physics/Colliders/ColliderFuncs.h>

namespace epl
{
	void IterativeCollisionDetection::detectCollisions(const Registry& reg, std::vector<Collision>& collisions)
	{
		/*
		const auto& allColliderTypes = colliderReg.getAllTypes();

		for (size_t i = 0; i < allColliderTypes.size(); i++)
		{
			auto& colliderType1 = allColliderTypes[i];
			for (size_t j = 0; j < allColliderTypes.size(); j++)
			{
				auto& colliderType2 = allColliderTypes[j];
				const CollisionCheck* collisionCheckPtr = colliderReg.getCollisionCheck(colliderType1.id, colliderType2.id);
				if (!collisionCheckPtr)
				{
					continue;
				}
				const CollisionCheck& collisionCheckFunc = *collisionCheckPtr;

				
				colliderType1.forEachColliderOfThisType(reg, [&](Entity e1, const BaseCollider& col1)
					{
						colliderType2.forEachColliderOfThisType(reg, [&](Entity e2, const BaseCollider& col2)
							{
								if (e1 >= e2) return;
								Collision collision;
								if (collisionCheckFunc(reg, col1, col2, e1, e2, collision))
								{
									collisions.push_back(collision);
								}
							});
					});
				
			}
		}
		*/

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
	}
}