#pragma once
namespace epl
{
	struct SphereCollider
	{
		float radius;
		SphereCollider(float radius_)
			: radius(radius_)
		{
		}
	};
}