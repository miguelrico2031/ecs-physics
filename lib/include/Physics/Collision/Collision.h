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
		float depth;
		Collision(Entity entity1_, Entity entity2_, const Vector3& normal_, float depth_)
			: entity1(entity1_), entity2(entity2_), normal(normal_), depth(depth_) {}
	};
}