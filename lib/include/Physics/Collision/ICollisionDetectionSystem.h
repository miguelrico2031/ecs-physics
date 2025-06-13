#pragma once
#include <vector>
#include <Physics/Collision/Collision.h>

namespace epl
{
	class Registry;
	class ColliderRegistry;
	class ICollisionDetectionSystem
	{
	public:
		virtual ~ICollisionDetectionSystem() = default;
		virtual void detectCollisions(const Registry& reg, const ColliderRegistry& colliderReg, std::vector<Collision>& collisions) = 0;
	};
}