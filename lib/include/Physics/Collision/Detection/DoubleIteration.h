#pragma once
#include <Physics/Collision/Detection/ICollisionDetectionSystem.h>

namespace epl
{
	class DoubleIteration : public ICollisionDetectionSystem
	{
	public:
		virtual void detectCollisions(const Registry& reg, const ColliderRegistry& colliderReg, std::vector<Collision>& collisions) override;
	};
}