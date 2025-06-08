#pragma once
#include "IIntegrationSystem.h"

namespace epl
{
	class EulerSemiImplicit : public IIntegrationSystem
	{
	public:
		EulerSemiImplicit() = default;
		~EulerSemiImplicit() override = default;

		virtual void integrate(Registry& registry, float dt) override;
	private:
		void integrateLinearVelocity(Registry& registry, float dt);
		void integratePosition(Registry& registry, float dt);
		void integrateAngularVelocity(Registry& registry, float dt);
		void integrateRotation(Registry& registry, float dt);
	};
}