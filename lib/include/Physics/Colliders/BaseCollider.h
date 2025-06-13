#pragma once
#include <Physics/Colliders/ColliderType.h>
#include <Physics/Collision/Collision.h>
#include <Math/Vector3.h>
#include <ECS/Entity.h>

namespace epl
{
	struct BaseCollider
	{
		ColliderTypeID typeID;
		BaseCollider(ColliderTypeID id) : typeID(id) {}
	};
}