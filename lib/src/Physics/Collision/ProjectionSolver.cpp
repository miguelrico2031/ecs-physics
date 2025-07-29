#include <Physics/Collision/ProjectionSolver.h>
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>

namespace epl
{
	void ProjectionSolver::resolveCollisions(Registry& reg, const std::vector<Collision>& collisions)
	{
		for (const Collision& collision : collisions)
		{
			Position& pos1 = reg.getComponent<Position>(collision.entity1);
			Position& pos2 = reg.getComponent<Position>(collision.entity2);

			float invMass1 = reg.getComponent<Mass>(collision.entity1).inverseMass;
			float invMass2 = reg.getComponent<Mass>(collision.entity2).inverseMass;

			float totalInvMass = invMass1 + invMass2;

			if (Math::equalsZero(totalInvMass))
			{
				continue;
			}

			pos1.value -= collision.normal * (collision.depth * (invMass1 / totalInvMass));
			pos2.value += collision.normal * (collision.depth * (invMass2 / totalInvMass));
		}
	}
}
