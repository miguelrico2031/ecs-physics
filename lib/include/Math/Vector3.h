#pragma once
#include "Math.h"
#include <iostream>

namespace epl
{
	struct Vector3
	{
		float x, y, z;

		constexpr Vector3() noexcept : x(0.f), y(0.f), z(0.f) {}
		constexpr Vector3(float x_, float y_, float z_) noexcept : x(x_), y(y_), z(z_) {}

		inline constexpr Vector3 operator-() const noexcept
		{
			return { -x, -y, -z };
		}

		inline constexpr Vector3& operator+=(const Vector3& other) noexcept
		{
			x += other.x; y += other.y; z += other.z;
			return *this;
		}

		inline constexpr Vector3& operator-=(const Vector3& other) noexcept
		{
			x -= other.x; y -= other.y; z -= other.z;
			return *this;
		}

		inline constexpr Vector3& operator*=(float scalar) noexcept
		{
			x *= scalar; y *= scalar; z *= scalar;

			return *this;
		}

		inline constexpr Vector3& operator/=(float scalar) noexcept
		{
			x /= scalar; y /= scalar; z /= scalar;
			return *this;
		}

		inline constexpr Vector3 operator+(const Vector3& other) const noexcept
		{
			return { x + other.x, y + other.y, z + other.z };
		}

		inline constexpr Vector3 operator-(const Vector3& other) const noexcept
		{
			return { x - other.x, y - other.y, z - other.z };
		}

		inline constexpr Vector3 operator*(float scalar) const noexcept
		{
			return { x * scalar, y * scalar, z * scalar };
		}

		inline constexpr Vector3 operator/(float scalar) const noexcept
		{
			return { x / scalar, y / scalar, z / scalar };
		}

		inline constexpr bool operator==(const Vector3& other) const noexcept
		{
			return Math::equals(x, other.x) && Math::equals(y, other.y) && Math::equals(z, other.z);
		}

		inline constexpr bool operator!=(const Vector3& other) const noexcept
		{
			return !(*this == other);
		}



		static inline constexpr float dot(const Vector3& a, const Vector3& b) noexcept
		{
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}

		static inline constexpr Vector3 cross(const Vector3& a, const Vector3& b) noexcept
		{
			return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
		}

		static inline constexpr float magnitude(const Vector3& v) noexcept
		{
			return Math::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
		}

		static inline constexpr float squaredMagnitude(const Vector3& v) noexcept
		{
			return v.x * v.x + v.y * v.y + v.z * v.z;
		}

		static inline constexpr Vector3 normalize(const Vector3& v) noexcept
		{
			float len = magnitude(v);
			return Math::equalsZero(len) ? Vector3::zero() : Vector3{ v.x / len, v.y / len, v.z / len };
		}

		static inline constexpr Vector3 zero() noexcept
		{
			return { 0.f, 0.f, 0.f };
		}

		static inline constexpr float distance(const Vector3& a, const Vector3& b)
		{
			return magnitude(a - b);
		}

		static inline constexpr Vector3 clamp(const Vector3& v, const Vector3& min, const Vector3& max) noexcept
		{
			return { Math::clamp(v.x, min.x, max.x), Math::clamp(v.y, min.y, max.y), Math::clamp(v.z, min.z, max.z) };
		}

		friend std::ostream& operator<<(std::ostream& out, const Vector3& v);

	};


	inline constexpr Vector3 operator*(float scalar, const Vector3& v) noexcept
	{
		return { v.x * scalar, v.y * scalar, v.z * scalar };
	}
	inline constexpr Vector3 operator/(float scalar, const Vector3& v)
	{
		return { scalar / v.x, scalar / v.y, scalar / v.z };
	}

	inline std::ostream& operator<<(std::ostream& out, const Vector3& v)
	{
		out << "Vector3(" << v.x << ", " << v.y << ", " << v.z << ")";
		return out;
	}
}