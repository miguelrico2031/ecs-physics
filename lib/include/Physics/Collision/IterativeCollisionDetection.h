#pragma once
#include <Physics/Collision/ICollisionDetectionSystem.h>

namespace epl
{
	class IterativeCollisionDetection : public ICollisionDetectionSystem
	{
	public:
		virtual void detectCollisions(const Registry& reg, const ColliderRegistry& colliderReg, std::vector<Collision>& collisions) override;
	};
}