#pragma once
#include <Math/Vector3.h>
#include <ECS/Entity.h>
namespace epl
{
	struct RayHit
	{
		Entity entity = NULL_ENTITY;
		Vector3 point;
		float distanceFromRayOrigin;
	};
}