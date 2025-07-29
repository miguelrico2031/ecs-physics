#include <Physics/Collision/ProjectionSolver.h>
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>

namespace epl
{
	void ProjectionSolver::resolveCollisions(Registry& reg, const std::vector<Collision>& collisions)
	{
		static int i = 0;
		static Vector3 lastPos;
		for (const Collision& collision : collisions)
		{
			bool isDynamic1 = reg.hasComponent<DynamicBody>(collision.entity1);
			bool isDynamic2 = reg.hasComponent<DynamicBody>(collision.entity2);

			if (!isDynamic1 && !isDynamic2) // kinematic - kinematic collisions won't be resolved
			{
				continue;
			}

			Position& pos1 = reg.getComponent<Position>(collision.entity1);
			Position& pos2 = reg.getComponent<Position>(collision.entity2);


			float invMass1 = 0.f;
			float invMass2 = 0.f;

			if (isDynamic1)
			{
				invMass1 = reg.getComponent<Mass>(collision.entity1).inverseMass;
			}
			if (isDynamic2)
			{
				invMass2 = reg.getComponent<Mass>(collision.entity2).inverseMass;
			}

			float totalInvMass = invMass1 + invMass2;

			pos1.value -= collision.normal * (collision.depth * (invMass1 / totalInvMass));
			lastPos = pos2.value;
			pos2.value += collision.normal * (collision.depth * (invMass2 / totalInvMass));
		}
	}
}
