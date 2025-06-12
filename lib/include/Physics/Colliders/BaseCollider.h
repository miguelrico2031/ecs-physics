#pragma once
#include <Physics/Colliders/ColliderType.h>


namespace epl
{
	struct BaseCollider
	{
		ColliderTypeID typeID;
		BaseCollider(ColliderTypeID id) : typeID(id) {}
	};
}