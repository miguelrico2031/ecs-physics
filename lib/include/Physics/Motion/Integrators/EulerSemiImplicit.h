#pragma once
#include <Physics/Motion/Integrators/IIntegrationSystem.h>

namespace epl
{
	class EulerSemiImplicit : public IIntegrationSystem
	{
	public:
		EulerSemiImplicit() = default;
		~EulerSemiImplicit() override = default;

		virtual void integrate(Registry& registry, float dt, float damping) override;
	private:
		void integrateLinearAcceleration(Registry& registry, float dt);
		void integrateLinearVelocity(Registry& registry, float dt, float damping);
		void integrateAngularAcceleration(Registry& registry, float dt);
		void integrateAngularVelocity(Registry& registry, float dt, float damping);
	};
}