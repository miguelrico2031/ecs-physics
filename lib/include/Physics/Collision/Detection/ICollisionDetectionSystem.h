#pragma once

namespace epl
{
	class Registry;
	class ICollisionDetectionSystem
	{
	public:
		virtual ~ICollisionDetectionSystem() = default;
		virtual void detectCollisions(Registry& reg) = 0;
	};
}