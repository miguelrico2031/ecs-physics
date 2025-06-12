#include <Physics/Raycast/IterativeRaycaster.h>
#include <Physics/Motion/MotionComponents.h>

namespace epl
{
	bool IterativeRaycaster::raycast(const Ray& ray, const Registry& reg, const ColliderRegistry& colliderReg, RayHit& hit)
	{
		bool intersected = false;
		hit.distanceFromRayOrigin = Math::Infinity();

		RayHit newHit;

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
					const Position& position = reg.getComponent<Position>(entity);
					
					if (intersectionCheckFunc(ray, col, position.value, newHit))
					{
						intersected = true;
						if (newHit.distanceFromRayOrigin < hit.distanceFromRayOrigin)
						{
							hit = newHit;
							hit.entity = entity;
							hit.collider = col;
						}
					}
				});
		}

		return intersected;
	}

	void IterativeRaycaster::raycastMultiple(const Ray& ray, const Registry& reg, const ColliderRegistry& colliderReg, 
		std::vector<RayHit>& hits, size_t maxHits)
	{		
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
					const Position& position = reg.getComponent<Position>(entity);
					RayHit newHit;
					if (intersectionCheckFunc(ray, col, position.value, newHit))
					{
						newHit.entity = entity;
						newHit.collider = col;
						hits.push_back(newHit);
					}
				});
		}
	}
}