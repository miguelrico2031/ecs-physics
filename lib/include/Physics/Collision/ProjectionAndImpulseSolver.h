#pragma once
#include <Physics/Collision/ICollisionResolutionSystem.h>
#include <Physics/Collision/ProjectionSolver.h>
#include <Physics/Collision/ImpulseSolver.h>

namespace epl
{
	//resolves collision by sepparating the bodies and then applying impulse
	class ProjectionAndImpulseSolver : public ICollisionResolutionSystem
	{
	public:
		virtual void resolveCollisions(Registry& reg, const std::vector<Collision>& collisions) override
		{
			m_projectionSolver.resolveCollisions(reg, collisions);
			m_impulseSolver.resolveCollisions(reg, collisions);
		}
	private:
		ProjectionSolver m_projectionSolver;
		ImpulseSolver m_impulseSolver;
	};
}