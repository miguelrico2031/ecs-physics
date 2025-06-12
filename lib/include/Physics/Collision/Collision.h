#pragma once
#include <ECS/Entity.h>
#include <Math/Vector3.h>
#include <cstdint>
namespace epl
{
	struct Collision
	{
		enum class Type : uint8_t
		{
			None = 0,
			SphereSphere,
			AABBAABB,
			SphereAABB
		};

		Entity entity1;
		Entity entity2;
		Vector3 normal;
		float depth;
		Type type = Type::None;
		Collision(Type type_, Entity entity1_, Entity entity2_, const Vector3& normal_, float depth_)
			: type(type_), entity1(entity1_), entity2(entity2_), normal(normal_), depth(depth_) {}
	};
}