#include <Physics/Raycast/IterativeRaycaster.h>
#include <Physics/Motion/MotionComponents.h>

#include <Physics/Colliders/ColliderFuncs.h>

namespace epl
{
	bool IterativeRaycaster::raycast(const Ray& ray, const Registry& reg, RayHit& hit)
	{
		bool intersected = false;
		hit.distanceFromRayOrigin = Math::infinity();

		RayHit newHit;

		for (const auto [entity, obb] : reg.iterate<OBBCollider>())
		{
			if (ColliderFuncs::isIntersectingOBB(reg, ray, obb, entity, newHit))
			{
				intersected = true;
				if (newHit.distanceFromRayOrigin < hit.distanceFromRayOrigin)
				{
					hit = newHit;
					hit.entity = entity;
				}
			}
		}
		for (const auto [entity, sphereCol] : reg.iterate<SphereCollider>())
		{
			if (ColliderFuncs::isIntersectingSphere(reg, ray, sphereCol, entity, newHit))
			{
				intersected = true;
				if (newHit.distanceFromRayOrigin < hit.distanceFromRayOrigin)
				{
					hit = newHit;
					hit.entity = entity;
				}
			}
		}

		return intersected;
	}

	void IterativeRaycaster::raycastMultiple(const Ray& ray, const Registry& reg, std::vector<RayHit>& hits, size_t maxHits)
	{	

		for (const auto [entity, obb] : reg.iterate<OBBCollider>())
		{
			if (hits.size() >= maxHits) return;
			RayHit newHit;
			if (ColliderFuncs::isIntersectingOBB(reg, ray, obb, entity, newHit))
			{
				newHit.entity = entity;
				hits.push_back(newHit);
			}
		}

		for (const auto [entity, sphereCol] : reg.iterate<SphereCollider>())
		{
			if (hits.size() >= maxHits) return;
			RayHit newHit;
			if (ColliderFuncs::isIntersectingSphere(reg, ray, sphereCol, entity, newHit))
			{
				newHit.entity = entity;
				hits.push_back(newHit);
			}
		}
	}
}