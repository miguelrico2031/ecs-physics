#pragma once
#include <Math/Vector3.h>
#include <Math/Quaternion.h>
#include <Math/Matrix3x3.h>
#include <cassert>

namespace epl
{
	struct Position
	{
		Vector3 value;
		Position(const Vector3& v) : value(v) {}
	};
	struct LinearVelocity
	{
		Vector3 value;
		LinearVelocity(const Vector3& v) : value(v) {}
	};
	struct Force
	{
		Vector3 value;
		Force(const Vector3& v) : value(v) {}
	};

	struct Rotation
	{
		Quaternion value;
		Rotation(const Quaternion& q) : value(q) {}
	};
	struct AngularVelocity
	{
		Vector3 value;
		AngularVelocity(const Vector3& v) : value(v) {}
	};
	struct Torque
	{
		Vector3 value;
		Torque(const Vector3& v) : value(v) {}
	};

	struct Gravity
	{
		Vector3 value;
		explicit Gravity(const Vector3& v) : value(v) {}
		static constexpr Vector3 earth() { return { 0.f, -9.81f, 0.f }; }
	};


	struct Mass
	{
		float mass = 0.f;
		float inverseMass = 0.f;
		Mass(float mass_)
		{
			if (!Math::equalsZero(mass_))
			{
				mass = mass_;
				inverseMass = 1.f / mass;
			}
		}
	};

	struct InverseInertia
	{
		Matrix3x3 tensor;
		InverseInertia(const Matrix3x3& tensor_) : tensor(tensor_) {}
	};

	struct LocalInverseInertia
	{
		Matrix3x3 tensor;
		LocalInverseInertia(const Matrix3x3& tensor_) : tensor(tensor_) {}
	};

	struct DynamicBody {};

}