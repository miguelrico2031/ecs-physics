#pragma once
#include <Math/Vector3.h>
#include <ECS/Entity.h>
#include <Physics/Collision/ColliderComponents.h>
namespace epl
{
	struct RayHit
	{
		Entity entity = NULL_ENTITY;
		ColliderType colliderType;
		Vector3 point;
		float distanceFromRayOrigin;
	};
}