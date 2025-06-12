#include <Physics/Raycast/IterativeRaycaster.h>
#include <Physics/Raycast/RaycastUtil.h>
#include <limits>
#include <Physics/Motion/MotionComponents.h>
#include <Physics/Collision/ColliderComponents.h>

namespace epl
{
	bool IterativeRaycaster::raycast(const Ray& ray, const Registry& reg, RayHit& hit)
	{
		bool intersected = false;
		RayHit newHit;
		hit.distanceFromRayOrigin = std::numeric_limits<float>::max();


		if (raycastSphereColliders(ray, reg, newHit))
		{
			intersected = true;
			hit = getClosestHit(hit, newHit);
		}
		if (raycastAABBColliders(ray, reg, newHit))
		{
			intersected = true;
			hit = getClosestHit(hit, newHit);
		}


		return intersected;
	}

	void IterativeRaycaster::raycastMultiple(const Ray& ray, const Registry& reg, std::vector<RayHit>& hits, size_t maxHits)
	{		
		raycastMultipleSphereColliders(ray, reg, hits, maxHits);
		raycastMultipleAABBColliders(ray, reg, hits, maxHits);
	}




	bool IterativeRaycaster::raycastSphereColliders(const Ray& ray, const Registry& reg, RayHit& hit)
	{
		bool intersected = false;
		hit.distanceFromRayOrigin = std::numeric_limits<float>::max();
		RayHit newHit;
		for (const auto& [entity, sphereCol] : reg.iterate<SphereCollider>())
		{
			const Position& position = reg.getComponent<Position>(entity);
			if (RaycastUtil::isIntersecting(ray, sphereCol, position.value, newHit))
			{
				intersected = true;
				newHit.entity = entity;
				hit = getClosestHit(hit, newHit);
			}
		}
		return intersected;
	}

	bool IterativeRaycaster::raycastAABBColliders(const Ray& ray, const Registry& reg, RayHit& hit)
	{
		bool intersected = false;
		hit.distanceFromRayOrigin = std::numeric_limits<float>::max();
		RayHit newHit;
		for (const auto& [entity, aabbCol] : reg.iterate<AABBCollider>())
		{
			const Position& position = reg.getComponent<Position>(entity);
			if (RaycastUtil::isIntersecting(ray, aabbCol, position.value, newHit))
			{
				intersected = true;
				newHit.entity = entity;
				hit = getClosestHit(hit, newHit);
			}
		}
		return intersected;
	}

	void IterativeRaycaster::raycastMultipleSphereColliders(const Ray& ray, const Registry& reg, std::vector<RayHit>& hits, size_t maxHits)
	{
		for (const auto& [entity, sphereCol] : reg.iterate<SphereCollider>())
		{
			if (hits.size() >= maxHits)
			{
				return;
			}
			const Position& position = reg.getComponent<Position>(entity);
			RayHit newHit;
			if (RaycastUtil::isIntersecting(ray, sphereCol, position.value, newHit))
			{
				hits.push_back(newHit);
			}
		}
	}

	void IterativeRaycaster::raycastMultipleAABBColliders(const Ray& ray, const Registry& reg, std::vector<RayHit>& hits, size_t maxHits)
	{
		for (const auto& [entity, aabbCol] : reg.iterate<AABBCollider>())
		{
			if (hits.size() >= maxHits)
			{
				return;
			}
			const Position& position = reg.getComponent<Position>(entity);
			RayHit newHit;
			if (RaycastUtil::isIntersecting(ray, aabbCol, position.value, newHit))
			{
				hits.push_back(newHit);
			}
		}
	}


	RayHit& IterativeRaycaster::getClosestHit(RayHit& h1, RayHit& h2)
	{
		return h1.distanceFromRayOrigin < h2.distanceFromRayOrigin
			? h1
			: h2;
	}
}