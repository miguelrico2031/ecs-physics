#pragma once

namespace epl
{
	class Registry;
	class IIntegrationSystem
	{
	public:
		virtual ~IIntegrationSystem() = default;
		virtual void integrate(Registry& registry, float dt, float damping) = 0;
	};
}