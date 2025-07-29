#pragma once
#include <vector>
#include <Physics/Collision/Collision.h>

namespace epl
{
	class Registry;
	class ICollisionResolutionSystem
	{
	public:
		virtual ~ICollisionResolutionSystem() = default;
		virtual void resolveCollisions(Registry& reg, const std::vector<Collision>& collisions) = 0;
	};
}