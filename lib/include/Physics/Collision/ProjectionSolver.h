#pragma once
#include <Physics/Collision/ICollisionResolutionSystem.h>

namespace epl
{
	//resolves collision by separating the bodies in the collision normal direction using their mass
	class ProjectionSolver : public ICollisionResolutionSystem
	{
	public:
		virtual void resolveCollisions(Registry& reg, const std::vector<Collision>& collisions) override;
	};
}