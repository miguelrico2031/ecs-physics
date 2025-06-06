#pragma once
#include "Math.h"
namespace epl
{
	struct Quaternion
	{
		float w, x, y, z;

		constexpr Quaternion() noexcept : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

		constexpr Quaternion(float w_, float x_, float y_, float z_) noexcept
			: w(w_), x(x_), y(y_), z(z_) {
		}

		constexpr bool operator==(const Quaternion& other) const noexcept
		{
			return Math::fEquals(w, other.w) && Math::fEquals(x, other.x) && Math::fEquals(y, other.y) && Math::fEquals(z, other.z);
		}

		constexpr bool operator!=(const Quaternion& other) const noexcept
		{
			return !(*this == other);
		}

		constexpr Quaternion operator*(const Quaternion& q) const noexcept
		{
			return
			{
				w * q.w - x * q.x - y * q.y - z * q.z,
				w * q.x + x * q.w + y * q.z - z * q.y,
				w * q.y - x * q.z + y * q.w + z * q.x,
				w * q.z + x * q.y - y * q.x + z * q.w
			};
		}

		constexpr Quaternion operator+(const Quaternion& q) const noexcept
		{
			return { w + q.w, x + q.x, y + q.y, z + q.z };
		}

		constexpr Quaternion operator-(const Quaternion& q) const noexcept
		{
			return { w - q.w, x - q.x, y - q.y, z - q.z };
		}

		constexpr Quaternion& operator+=(const Quaternion& q) noexcept
		{
			w += q.w; x += q.x; y += q.y; z += q.z;
			return *this;
		}

		constexpr Quaternion& operator-=(const Quaternion& q) noexcept
		{
			w -= q.w; x -= q.x; y -= q.y; z -= q.z;
			return *this;
		}

		constexpr Quaternion& operator*=(const Quaternion& q) noexcept
		{
			*this = *this * q;
			return *this;
		}

		constexpr Quaternion& operator*=(float scalar) noexcept
		{
			w *= scalar; x *= scalar; y *= scalar; z *= scalar;
			return *this;
		}

		constexpr Quaternion& operator/=(float scalar) noexcept
		{
			w /= scalar; x /= scalar; y /= scalar; z /= scalar;
			return *this;
		}

		constexpr Quaternion operator*(float scalar) const noexcept
		{
			return { w * scalar, x * scalar, y * scalar, z * scalar };
		}

		constexpr Quaternion operator/(float scalar) const noexcept
		{
			return { w / scalar, x / scalar, y / scalar, z / scalar };
		}

		static constexpr Quaternion conjugate(const Quaternion& q) noexcept
		{
			return { q.w, -q.x, -q.y, -q.z };
		}

		static constexpr float magnitude(const Quaternion& q) noexcept
		{
			return Math::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
		}

		static constexpr float squaredMagnitude(const Quaternion& q) noexcept
		{
			return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
		}

		static constexpr Quaternion normalize(const Quaternion& q) noexcept
		{
			float len = magnitude(q);
			return Math::fEqualsZero(len) ? Quaternion::identity() : Quaternion{ q.w / len, q.x / len, q.y / len, q.z / len };
		}

		static constexpr Quaternion identity() noexcept
		{
			return {};
		}
	};


	constexpr Quaternion operator*(float scalar, const Quaternion& q) noexcept
	{
		return { q.w * scalar, q.x * scalar, q.y * scalar, q.z * scalar };
	}

	constexpr Quaternion operator/(float scalar, const Quaternion& q) noexcept
	{
		return { scalar / q.w, scalar / q.x,  scalar / q.y, scalar / q.z };
	}
}