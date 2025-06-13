#include <Physics/Collision/IterativeCollisionDetection.h>
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>
#include <Physics/Colliders/ColliderRegistry.h>

namespace epl
{
	void IterativeCollisionDetection::detectCollisions(const Registry& reg, const ColliderRegistry& colliderReg, std::vector<Collision>& collisions)
	{
		const auto& allColliderTypes = colliderReg.getAllTypes();

		for (size_t i = 0; i < allColliderTypes.size(); i++)
		{
			auto& colliderType1 = allColliderTypes[i];
			for (size_t j = i; j < allColliderTypes.size(); j++)
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
								//Vector3 p1 = reg.getComponent<Position>(e1).value;
								//Vector3 p2 = reg.getComponent<Position>(e2).value;
								//Vector3 normal;
								//float depth;
								Collision collision;
								if (collisionCheckFunc(reg, col1, col2, e1, e2, collision))
								{
									collision.entity1 = e1;
									collision.entity2 = e2;
									collisions.push_back(collision);
								}
							});
					});
			}
		}
	}
}