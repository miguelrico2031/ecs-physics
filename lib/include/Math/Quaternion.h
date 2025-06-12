#pragma once
#include <Math/Math.h>
namespace epl
{
	struct Quaternion
	{
		float w, x, y, z;

		constexpr Quaternion() noexcept : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

		constexpr Quaternion(float w_, float x_, float y_, float z_) noexcept
			: w(w_), x(x_), y(y_), z(z_) {}

		inline constexpr bool operator==(const Quaternion& other) const noexcept
		{
			return Math::equals(w, other.w) && Math::equals(x, other.x) && Math::equals(y, other.y) && Math::equals(z, other.z);
		}

		inline constexpr bool operator!=(const Quaternion& other) const noexcept
		{
			return !(*this == other);
		}

		inline constexpr Quaternion operator*(const Quaternion& q) const noexcept
		{
			return
			{
				w * q.w - x * q.x - y * q.y - z * q.z,
				w * q.x + x * q.w + y * q.z - z * q.y,
				w * q.y - x * q.z + y * q.w + z * q.x,
				w * q.z + x * q.y - y * q.x + z * q.w
			};
		}

		inline constexpr Quaternion operator+(const Quaternion& q) const noexcept
		{
			return { w + q.w, x + q.x, y + q.y, z + q.z };
		}

		inline constexpr Quaternion operator-(const Quaternion& q) const noexcept
		{
			return { w - q.w, x - q.x, y - q.y, z - q.z };
		}

		inline constexpr Quaternion& operator+=(const Quaternion& q) noexcept
		{
			w += q.w; x += q.x; y += q.y; z += q.z;
			return *this;
		}

		inline constexpr Quaternion& operator-=(const Quaternion& q) noexcept
		{
			w -= q.w; x -= q.x; y -= q.y; z -= q.z;
			return *this;
		}

		inline constexpr Quaternion& operator*=(const Quaternion& q) noexcept
		{
			*this = *this * q;
			return *this;
		}

		inline constexpr Quaternion& operator*=(float scalar) noexcept
		{
			w *= scalar; x *= scalar; y *= scalar; z *= scalar;
			return *this;
		}

		inline constexpr Quaternion& operator/=(float scalar) noexcept
		{
			w /= scalar; x /= scalar; y /= scalar; z /= scalar;
			return *this;
		}

		inline constexpr Quaternion operator*(float scalar) const noexcept
		{
			return { w * scalar, x * scalar, y * scalar, z * scalar };
		}

		inline constexpr Quaternion operator/(float scalar) const noexcept
		{
			return { w / scalar, x / scalar, y / scalar, z / scalar };
		}



		static inline constexpr Quaternion conjugate(const Quaternion& q) noexcept
		{
			return { q.w, -q.x, -q.y, -q.z };
		}

		static inline constexpr float magnitude(const Quaternion& q) noexcept
		{
			return Math::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
		}

		static inline constexpr float squaredMagnitude(const Quaternion& q) noexcept
		{
			return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
		}

		static inline constexpr Quaternion normalize(const Quaternion& q) noexcept
		{
			float len = magnitude(q);
			return Math::equalsZero(len) ? Quaternion::identity() : Quaternion{ q.w / len, q.x / len, q.y / len, q.z / len };
		}

		static inline constexpr Quaternion identity() noexcept
		{
			return {};
		}

		friend std::ostream& operator<<(std::ostream& out, const Quaternion& q);
	};


	inline constexpr Quaternion operator*(float scalar, const Quaternion& q) noexcept
	{
		return { q.w * scalar, q.x * scalar, q.y * scalar, q.z * scalar };
	}

	inline constexpr Quaternion operator/(float scalar, const Quaternion& q) noexcept
	{
		return { scalar / q.w, scalar / q.x,  scalar / q.y, scalar / q.z };
	}

	inline std::ostream& operator<<(std::ostream& out, const Quaternion& q)
	{
		out << "Quaternion(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
		return out;
	}
}