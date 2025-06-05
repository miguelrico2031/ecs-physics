#pragma once
#include <Math/Vector3.h>
#include <Math/Quaternion.h>

namespace epl
{
	using Position = Vector3;
	using LinearVelocity = Vector3;
	using ForceSum = Vector3;

	using Rotation = Quaternion;
	using AngularVelocity = Vector3;
	using TorqueSum = Vector3;

	struct Mass
	{
		float mass;
		float inverseMass;
		Mass(float m) : mass(m), inverseMass(1.f / m) {}
	};
}