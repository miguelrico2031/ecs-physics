#pragma once
#include <ECS/Registry.h>
#include <Physics/Components.h>

namespace epl
{
	class GravitySystem
	{
	public:
		void applyGravity(Registry& registry);
	};
}