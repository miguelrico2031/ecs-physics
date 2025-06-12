#pragma once
#include <Physics/Raycast/Ray.h>
#include <Physics/Raycast/RayHit.h>
#include <Physics/Colliders/ColliderRegistry.h>
#include <ECS/Registry.h>
#include <vector>

namespace epl
{
	class IRaycastSystem
	{
	public:
		~IRaycastSystem() = default;
		virtual bool raycast(const Ray& ray, const Registry& reg, const ColliderRegistry& colliderReg, RayHit& hit) = 0;
		virtual void raycastMultiple(const Ray& ray, const Registry& reg, const ColliderRegistry& colliderReg, 
			std::vector<RayHit>& hits, size_t maxHits) = 0;
	};
}