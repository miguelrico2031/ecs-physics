#pragma once
#include <Math/Vector3.h>
#include <Math/Quaternion.h>
#include <ECS/Entity.h>
namespace epl
{
	enum class ColliderType
	{
		Sphere = 1,
		AABB = 2
	};

	struct SphereCollider
	{
		float radius;
		Vector3 offset;
		SphereCollider(float radius_, const Vector3& offset_ = Vector3::zero())
			: radius(radius_), offset(offset_) {}
	};

	struct AABBCollider
	{
		Vector3 halfSize;
		Vector3 offset;
		AABBCollider(const Vector3& halfSize_, const Vector3& offset_ = Vector3::zero())
			: halfSize(halfSize_), offset(offset_) {}
	};

	struct IsColliding {};
}