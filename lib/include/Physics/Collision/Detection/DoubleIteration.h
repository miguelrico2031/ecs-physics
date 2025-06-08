#pragma once
#include "ICollisionDetectionSystem.h"

namespace epl
{
	class DoubleIteration : public ICollisionDetectionSystem
	{
	public:
		virtual void detectCollisions(Registry& reg) override;
	private:
		void resetCollisionFlags(Registry& reg);
		void detectSphereSphereCollisions(Registry& reg);
		void detectAABBAABBCollisions(Registry& reg);
		void detectSphereAABBCollisions(Registry& reg);
	};
}