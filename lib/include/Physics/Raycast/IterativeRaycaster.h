#pragma once
#include<Physics/Raycast/IRaycastSystem.h>

namespace epl
{
	class IterativeRaycaster : public IRaycastSystem
	{
	public:
		virtual bool raycast(const Ray& ray, const Registry& reg, const ColliderRegistry& colliderReg, RayHit& hit) override;
		virtual void raycastMultiple(const Ray& ray, const Registry& reg, const ColliderRegistry& colliderReg,
			std::vector<RayHit>& hits, size_t maxHits) override;
	};
}