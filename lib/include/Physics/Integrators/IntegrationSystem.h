#pragma once
#include <Physics/Components.h>

namespace epl
{
	class Registry;
	class IntegrationSystem
	{
		public:
		virtual ~IntegrationSystem() = default;
		virtual void integrate(Registry& registry, float dt) = 0;
	};
}