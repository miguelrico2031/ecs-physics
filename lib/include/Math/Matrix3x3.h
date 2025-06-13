#pragma once
#include <Math/Math.h>
#include <Math/Vector3.h>
#include <Math/Quaternion.h>
#include <cstring>
#include <cmath>

namespace epl
{
	struct Matrix3x3
	{
		float matrix[9];

		// Constructors
		Matrix3x3() noexcept = default;

		explicit Matrix3x3(const Vector3& diagonal) noexcept //diagonal vector
		{
			matrix[0] = diagonal.x; matrix[1] = 0.0f;		matrix[2] = 0.0f;
			matrix[3] = 0.0f;		matrix[4] = diagonal.y; matrix[5] = 0.0f;
			matrix[6] = 0.0f;		matrix[7] = 0.0f;		matrix[8] = diagonal.z;
		}

		explicit Matrix3x3(const Quaternion& q) noexcept
		{
			const float x2 = q.x + q.x;
			const float y2 = q.y + q.y;
			const float z2 = q.z + q.z;
			const float xx2 = q.x * x2;
			const float yy2 = q.y * y2;
			const float zz2 = q.z * z2;
			const float xy2 = q.x * y2;
			const float xz2 = q.x * z2;
			const float yz2 = q.y * z2;
			const float wx2 = q.w * x2;
			const float wy2 = q.w * y2;
			const float wz2 = q.w * z2;

			matrix[0] = 1.0f - (yy2 + zz2);
			matrix[1] = xy2 - wz2;
			matrix[2] = xz2 + wy2;
			matrix[3] = xy2 + wz2;
			matrix[4] = 1.0f - (xx2 + zz2);
			matrix[5] = yz2 - wx2;
			matrix[6] = xz2 - wy2;
			matrix[7] = yz2 + wx2;
			matrix[8] = 1.0f - (xx2 + yy2);
		}

		explicit Matrix3x3(const float* values) noexcept
		{
			std::memcpy(matrix, values, 9 * sizeof(float));
		}




		inline constexpr Matrix3x3 Negate() const noexcept
		{
			Matrix3x3 result{};
			for (int i = 0; i < 9; ++i)
				result.matrix[i] = -matrix[i];
			return result;
		}

		inline Matrix3x3& AddAssign(const Matrix3x3& other) noexcept
		{
			for (int i = 0; i < 9; ++i)
				matrix[i] += other.matrix[i];
			return *this;
		}

		inline Matrix3x3& SubtractAssign(const Matrix3x3& other) noexcept
		{
			for (int i = 0; i < 9; ++i)
				matrix[i] -= other.matrix[i];
			return *this;
		}

		inline Matrix3x3& MultiplyAssign(const Matrix3x3& other) noexcept
		{
			*this = Multiply(other);
			return *this;
		}

		//inline Matrix3x3& DivideAssign(const Matrix3x3& other) noexcept
		//{
		//	for (int i = 0; i < 9; ++i)
		//		matrix[i] /= other.matrix[i];
		//	return *this;
		//}

		inline Matrix3x3 Add(const Matrix3x3& other) const noexcept
		{
			Matrix3x3 result;
			for (int i = 0; i < 9; ++i)
				result.matrix[i] = matrix[i] + other.matrix[i];
			return result;
		}

		inline Matrix3x3 Subtract(const Matrix3x3& other) const noexcept
		{
			Matrix3x3 result;
			for (int i = 0; i < 9; ++i)
				result.matrix[i] = matrix[i] - other.matrix[i];
			return result;
		}

		inline Matrix3x3 Multiply(const Matrix3x3& other) const noexcept
		{
			Matrix3x3 result;
			for (int row = 0; row < 3; ++row)
			{
				for (int col = 0; col < 3; ++col)
				{
					result.matrix[row * 3 + col] =
						matrix[row * 3 + 0] * other.matrix[0 * 3 + col] +
						matrix[row * 3 + 1] * other.matrix[1 * 3 + col] +
						matrix[row * 3 + 2] * other.matrix[2 * 3 + col];
				}
			}
			return result;
		}

		//inline Matrix3x3 Divide(const Matrix3x3& other) const noexcept
		//{
		//	Matrix3x3 result;
		//	for (int i = 0; i < 9; ++i)
		//		result.matrix[i] = matrix[i] / other.matrix[i];
		//	return result;
		//}

		inline Vector3 MultiplyVec(const Vector3& v) const noexcept
		{
			return Vector3(
				matrix[0] * v.x + matrix[1] * v.y + matrix[2] * v.z,
				matrix[3] * v.x + matrix[4] * v.y + matrix[5] * v.z,
				matrix[6] * v.x + matrix[7] * v.y + matrix[8] * v.z
			);
		}

		inline Vector3 DivideVec(const Vector3& v) const noexcept
		{
			return Vector3(
				matrix[0] / v.x + matrix[1] / v.y + matrix[2] / v.z,
				matrix[3] / v.x + matrix[4] / v.y + matrix[5] / v.z,
				matrix[6] / v.x + matrix[7] / v.y + matrix[8] / v.z
			);
		}

		// Comparison
		inline bool IsEqual(const Matrix3x3& other) const noexcept
		{
			for (int i = 0; i < 9; ++i)
				if (matrix[i] != other.matrix[i])
					return false;
			return true;
		}

		inline bool IsNotEqual(const Matrix3x3& other) const noexcept
		{
			return !IsEqual(other);
		}

		// Operator overloads for syntactic sugar (optional, but efficient)
		inline constexpr Matrix3x3 operator-() const noexcept { return Negate(); }
		inline Matrix3x3& operator+=(const Matrix3x3& other) noexcept { return AddAssign(other); }
		inline Matrix3x3& operator-=(const Matrix3x3& other) noexcept { return SubtractAssign(other); }
		inline Matrix3x3& operator*=(const Matrix3x3& other) noexcept { return MultiplyAssign(other); }
		//inline Matrix3x3& operator/=(const Matrix3x3& other) noexcept { return DivideAssign(other); }
		inline Matrix3x3 operator+(const Matrix3x3& other) const noexcept { return Add(other); }
		inline Matrix3x3 operator-(const Matrix3x3& other) const noexcept { return Subtract(other); }
		inline Matrix3x3 operator*(const Matrix3x3& other) const noexcept { return Multiply(other); }
		//inline Matrix3x3 operator/(const Matrix3x3& other) const noexcept { return Divide(other); }
		inline Vector3 operator*(const Vector3& v) const noexcept { return MultiplyVec(v); }
		inline Vector3 operator/(const Vector3& v) const noexcept { return DivideVec(v); }
		inline bool operator==(const Matrix3x3& other) const noexcept { return IsEqual(other); }
		inline bool operator!=(const Matrix3x3& other) const noexcept { return IsNotEqual(other); }

		// Row access for [][] operator
		inline float* operator[](size_t row) noexcept
		{
			return &matrix[row * 3];
		}
		inline const float* operator[](size_t row) const noexcept
		{
			return &matrix[row * 3];
		}

		friend std::ostream& operator<<(std::ostream& out, const Matrix3x3& m);

	};

	inline std::ostream& operator<<(std::ostream& out, const Matrix3x3& m)
	{
		out << "Matrix3x3(" << m[0][0] << ",\t" << m[0][1] << ",\t" << m[0][2]<< "," << '\n'
			<< "          " << m[1][0] << ",\t" << m[1][1] << ",\t" << m[1][2]<< "," << '\n'
			<< "          " << m[2][0] << ",\t" << m[2][1] << ",\t" << m[2][2]		<< ")\n";
		return out;
	}
}