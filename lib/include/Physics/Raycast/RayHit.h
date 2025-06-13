#pragma once
#include <Math/Vector3.h>
#include <ECS/Entity.h>
#include <Physics/Colliders/BaseCollider.h>
namespace epl
{
	struct RayHit
	{
		Entity entity = NULL_ENTITY;
		const BaseCollider* collider;
		Vector3 point;
		float distanceFromRayOrigin;
	};
}