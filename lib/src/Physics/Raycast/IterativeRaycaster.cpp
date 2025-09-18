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

		/*
		const auto& allColliderTypes = colliderReg.getAllTypes();
		for (const auto& colliderType : allColliderTypes)
		{
			const auto* intersectionCheckPtr = colliderReg.getRayIntersectionCheck(colliderType.id);
			if (!intersectionCheckPtr)
			{
				continue;
			}
			const RayIntersectionCheck& intersectionCheckFunc = *intersectionCheckPtr;

			colliderType.forEachColliderOfThisType(reg, [&](Entity entity, const BaseCollider& col)
				{
					//const Position& position = reg.getComponent<Position>(entity);
					
					if (intersectionCheckFunc(reg, ray, col, entity, newHit))
					{
						intersected = true;
						if (newHit.distanceFromRayOrigin < hit.distanceFromRayOrigin)
						{
							hit = newHit;
							hit.entity = entity;
							hit.collider = &col;
						}
					}
				});
		}
		*/

		for (const auto [entity, obb] : reg.iterate<OBBCollider>())
		{
			if (ColliderFuncs::isIntersectingOBB(reg, ray, obb, entity, newHit))
			{
				intersected = true;
				if (newHit.distanceFromRayOrigin < hit.distanceFromRayOrigin)
				{
					hit = newHit;
					hit.entity = entity;
					//hit.collider = &col;
				}
			}
		}

		return intersected;
	}

	void IterativeRaycaster::raycastMultiple(const Ray& ray, const Registry& reg, std::vector<RayHit>& hits, size_t maxHits)
	{	
		/*
		const auto& allColliderTypes = colliderReg.getAllTypes();
		for (const auto& colliderType : allColliderTypes)
		{
			if (hits.size() >= maxHits)
			{
				break;
			}
			const auto* intersectionCheckPtr = colliderReg.getRayIntersectionCheck(colliderType.id);
			if (!intersectionCheckPtr)
			{
				continue;
			}
			const RayIntersectionCheck& intersectionCheckFunc = *intersectionCheckPtr;

			colliderType.forEachColliderOfThisType(reg, [&](Entity entity, const BaseCollider& col)
				{
					if (hits.size() >= maxHits)
					{
						return;
					}
					//const Position& position = reg.getComponent<Position>(entity);
					RayHit newHit;
					if (intersectionCheckFunc(reg, ray, col, entity, newHit))
					{
						newHit.entity = entity;
						newHit.collider = &col;
						hits.push_back(newHit);
					}
				});
		}
		*/

		for (const auto [entity, obb] : reg.iterate<OBBCollider>())
		{
			if (hits.size() >= maxHits) return;
			RayHit newHit;
			if (ColliderFuncs::isIntersectingOBB(reg, ray, obb, entity, newHit))
			{
				newHit.entity = entity;
				//newHit.collider = &col;
				hits.push_back(newHit);
			}
		}
	}
}