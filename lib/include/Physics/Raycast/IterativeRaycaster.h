#pragma once
#include<Physics/Raycast/IRaycastSystem.h>

namespace epl
{
	class IterativeRaycaster : public IRaycastSystem
	{
	public:
		virtual bool raycast(const Ray& ray, const Registry& reg, RayHit& hit) override;
		virtual void raycastMultiple(const Ray& ray, const Registry& reg, std::vector<RayHit>& hits, size_t maxHits) override;
	private:
		bool raycastSphereColliders(const Ray& ray, const Registry& reg, RayHit& hit);
		bool raycastAABBColliders(const Ray& ray, const Registry& reg, RayHit& hit);
		void raycastMultipleSphereColliders(const Ray& ray, const Registry& reg, std::vector<RayHit>& hits, size_t maxHits);
		void raycastMultipleAABBColliders(const Ray& ray, const Registry& reg, std::vector<RayHit>& hits, size_t maxHits);

		RayHit& getClosestHit(RayHit& h1, RayHit& h2);
	};
}