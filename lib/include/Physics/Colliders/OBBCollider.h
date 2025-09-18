#pragma once 
#include <Math/Vector3.h>
namespace epl
{
	struct OBBCollider
	{
		Vector3 halfSize;
		OBBCollider(const Vector3& halfSize_)
			: halfSize(halfSize_)
		{
		}
	};

	using BoxCollider = OBBCollider; //default box collider to use in gameplay (AABB is used for broadphase and other internal calculations)
}