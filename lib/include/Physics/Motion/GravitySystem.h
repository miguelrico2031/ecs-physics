#pragma once
#include <ECS/Registry.h>
#include <Physics/Motion/MotionComponents.h>
namespace epl
{
	class GravitySystem
	{
	public:
		void applyGravity(Registry& registry);
	};
}