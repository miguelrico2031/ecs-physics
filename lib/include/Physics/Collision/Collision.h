#pragma once
#include <ECS/Entity.h>
#include <Math/Vector3.h>
#include <cstdint>
namespace epl
{
	struct Collision
	{
		Entity entity1;
		Entity entity2;
		Vector3 normal;
		//Vector3 contactPoint;
		//Vector3 contactPoint2__;
		Vector3 contactPoints[4];
		int contactPointsCount;
		float depth;
	};
}