#pragma once
#include <Math/Vector3.h>
#include <Math/Quaternion.h>
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
	struct ForceSum
	{ 
		Vector3 value; 
		ForceSum(const Vector3& v) : value(v) {}
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
	struct TorqueSum
	{ 
		Vector3 value; 
		TorqueSum(const Vector3& v) : value(v) {}
	};

	struct Gravity
	{ 
		Vector3 value; 
		explicit Gravity(const Vector3& v) : value(v) {}
		static constexpr Vector3 Earth() { return { 0.f, -9.81f, 0.f }; }
	};


	struct Mass
	{
		float mass = 0.f;
		float inverseMass = 0.f;
		Mass(float m)
		{
			assert(m > 0.f && "Mass has to be grater than zero.");
			mass = m;
			inverseMass = 1.f / mass;
		}
	};


	struct Kinematic {};
}