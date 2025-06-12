#include <Physics/Collision/Detection/DoubleIteration.h>
#include <ECS/Registry.h>
#include <Physics/Collision/CollidersUtil.h>
#include <Physics/Motion/MotionComponents.h>

namespace epl
{
	void DoubleIteration::detectCollisions(Registry& reg, std::vector<Collision>& collisions)
	{
		resetCollisionFlags(reg);
		detectSphereSphereCollisions(reg, collisions);
		detectAABBAABBCollisions(reg, collisions);
		detectSphereAABBCollisions(reg, collisions);
	}

	void DoubleIteration::resetCollisionFlags(Registry& reg)
	{
		for (auto [e, _] : reg.iterate<IsColliding>())
		{
			reg.removeComponent<IsColliding>(e);
		}
	}

	void DoubleIteration::detectSphereSphereCollisions(Registry& reg, std::vector<Collision>& collisions)
	{
		for (const auto& [e1, col1] : reg.iterate<SphereCollider>())
		{
			for (const auto& [e2, col2] : reg.iterate<SphereCollider>())
			{
				if (e1 >= e2) continue;

				Vector3 p1 = reg.getComponent<Position>(e1).value;
				Vector3 p2 = reg.getComponent<Position>(e2).value;
				Vector3 normal;
				float depth;
				if (CollidersUtil::isColliding(col1, col2, p1, p2, normal, depth))
				{
					collisions.emplace_back(Collision::Type::SphereSphere, e1, e2, normal, depth);
					reg.addOrSetComponent<IsColliding>(e1);
					reg.addOrSetComponent<IsColliding>(e2);
				}
			}
		}
	}

	void DoubleIteration::detectAABBAABBCollisions(Registry& reg, std::vector<Collision>& collisions)
	{
		for (const auto& [e1, col1] : reg.iterate<AABBCollider>())
		{
			for (const auto& [e2, col2] : reg.iterate<AABBCollider>())
			{
				if (e1 >= e2) continue;

				Vector3 p1 = reg.getComponent<Position>(e1).value;
				Vector3 p2 = reg.getComponent<Position>(e2).value;
				Vector3 normal;
				float depth;
				if (CollidersUtil::isColliding(col1, col2, p1, p2, normal, depth))
				{
					collisions.emplace_back(Collision::Type::AABBAABB, e1, e2, normal, depth);
					reg.addOrSetComponent<IsColliding>(e1);
					reg.addOrSetComponent<IsColliding>(e2);
				}
			}
		}
	}

	void DoubleIteration::detectSphereAABBCollisions(Registry& reg, std::vector<Collision>& collisions)
	{
		for (const auto& [e1, col1] : reg.iterate<SphereCollider>())
		{
			for (const auto& [e2, col2] : reg.iterate<AABBCollider>())
			{
				if (e1 >= e2) continue;

				Vector3 p1 = reg.getComponent<Position>(e1).value;
				Vector3 p2 = reg.getComponent<Position>(e2).value;
				Vector3 normal;
				float depth;
				if (CollidersUtil::isColliding(col1, col2, p1, p2, normal, depth))
				{
					collisions.emplace_back(Collision::Type::SphereAABB, e1, e2, normal, depth);
					reg.addOrSetComponent<IsColliding>(e1);
					reg.addOrSetComponent<IsColliding>(e2);
				}
			}
		}
	}
}