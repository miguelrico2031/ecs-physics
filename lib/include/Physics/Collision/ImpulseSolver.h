#pragma once
#include <Physics/Collision/ICollisionResolutionSystem.h>


namespace epl
{
	//resolves collision by applying an impulse both linear and angular using the collision normals,
	//the relative velocity and the restitution coefficients
	class ImpulseSolver : public ICollisionResolutionSystem
	{
	public:
		virtual void resolveCollisions(Registry& reg, const std::vector<Collision>& collisions) override;
	};
}