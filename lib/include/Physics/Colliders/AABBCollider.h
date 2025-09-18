#pragma once 
#include <Math/Vector3.h>

namespace epl
{

	struct AABBCollider
	{
		Vector3 halfSize;
		AABBCollider(const Vector3& halfSize_)
			:  halfSize(halfSize_)
		{
		}
	};

}