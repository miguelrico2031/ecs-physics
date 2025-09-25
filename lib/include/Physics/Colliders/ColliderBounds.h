#pragma once
#include <Math/Vector3.h>


namespace epl
{
	//Contains a tight AABB defined in the entity's local space. This is not updated unless entity's collider size changes
	//Also contains a fat AABB defined in world space. This is updated very time the tight AABB exits the fat AABB.
	struct ColliderBounds
	{
		Vector3 localTightMin;
		Vector3 localTightMax;
		Vector3 worldFatMin;
		Vector3 worldFatMax;
		ColliderBounds() {}
		ColliderBounds(Vector3 localTightMin_, Vector3 localTightMax_, Vector3 worldFatMin_, Vector3 worldFatMax_)
			: localTightMin(localTightMin_), localTightMax(localTightMax_), worldFatMin(worldFatMin_), worldFatMax(worldFatMax_)
		{
		}
	};
}