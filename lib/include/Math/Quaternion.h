#pragma once
#include <Math/Math.h>
#include <Math/Vector3.h>
#include <utility>
#include <cmath>
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

		static inline Quaternion fromAxisAngle(const Vector3& axis, float angleRadians)
		{
			// axis must be normalized
			float halfAngle = angleRadians * 0.5f;
			float s = sin(halfAngle);
			Vector3 normAxis = Vector3::normalize(axis);
			return {
				cos(halfAngle),
				normAxis.x * s,
				normAxis.y * s,
				normAxis.z * s
			};
		}

		static inline Quaternion fromEulerAngles(const Vector3& eulerAnglesRadians)
		{
			// Assuming eulerAngles are in radians and in XYZ order
			float cx = cos(eulerAnglesRadians.x * 0.5f);
			float sx = sin(eulerAnglesRadians.x * 0.5f);
			float cy = cos(eulerAnglesRadians.y * 0.5f);
			float sy = sin(eulerAnglesRadians.y * 0.5f);
			float cz = cos(eulerAnglesRadians.z * 0.5f);
			float sz = sin(eulerAnglesRadians.z * 0.5f);

			return {
				cx * cy * cz + sx * sy * sz,
				sx * cy * cz - cx * sy * sz,
				cx * sy * cz + sx * cy * sz,
				cx * cy * sz - sx * sy * cz
			};
		}

		static inline std::pair<Vector3, float> toAxisAngle(const Quaternion& q)
		{
			Quaternion nq = Quaternion::normalize(q);
			float angle = 2.0f * acos(nq.w);
			float s = Math::sqrt(1.0f - nq.w * nq.w);
			Vector3 axis;
			if (Math::equalsZero(s))
			{
				// If s is close to zero, direction of axis is not important
				axis = Vector3(1.0f, 0.0f, 0.0f);
			} else {
				axis = Vector3(nq.x / s, nq.y / s, nq.z / s);
			}
			return std::make_pair(axis, angle);
		}

		static inline Vector3 toEulerAngles(const Quaternion& q)
		{
			// Assuming XYZ order
			Quaternion nq = Quaternion::normalize(q);

			// roll (x-axis rotation)
			float sinr_cosp = 2.0f * (nq.w * nq.x + nq.y * nq.z);
			float cosr_cosp = 1.0f - 2.0f * (nq.x * nq.x + nq.y * nq.y);
			float roll = atan2(sinr_cosp, cosr_cosp);

			// pitch (y-axis rotation)
			float sinp = 2.0f * (nq.w * nq.y - nq.z * nq.x);
			float pitch =  Math::abs(sinp) >= 1.0f
				? copysign(Math::pi() * 0.5f, sinp) // use 90 degrees if out of range
				: asin(sinp);

			// yaw (z-axis rotation)
			float siny_cosp = 2.0f * (nq.w * nq.z + nq.x * nq.y);
			float cosy_cosp = 1.0f - 2.0f * (nq.y * nq.y + nq.z * nq.z);
			float yaw = atan2(siny_cosp, cosy_cosp);

			return Vector3(roll, pitch, yaw);
		}

		static inline Vector3 rotate(const Quaternion& q, const Vector3& v)
		{
			Vector3 qvec(q.x, q.y, q.z);
			Vector3 t = Vector3::cross(qvec, v) * 2.0f;
			return v + q.w * t + Vector3::cross(qvec, t);
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