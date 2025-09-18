#pragma once
#include <Math/Vector3.h>


namespace epl
{
	struct ColliderBounds
	{
		Vector3 localMin;
		Vector3 localMax;
		ColliderBounds(Vector3 min, Vector3 max)
			: localMin(min), localMax(max)
		{
		}
	};
}