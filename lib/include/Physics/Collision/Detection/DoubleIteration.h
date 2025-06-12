#pragma once
#include <Physics/Collision/Detection/ICollisionDetectionSystem.h>

namespace epl
{
	class DoubleIteration : public ICollisionDetectionSystem
	{
	public:
		virtual void detectCollisions(Registry& reg, std::vector<Collision>& collisions) override;
	private:
		void resetCollisionFlags(Registry& reg);
		void detectSphereSphereCollisions(Registry& reg, std::vector<Collision>& collisions);
		void detectAABBAABBCollisions(Registry& reg, std::vector<Collision>& collisions);
		void detectSphereAABBCollisions(Registry& reg, std::vector<Collision>& collisions);
	};
}