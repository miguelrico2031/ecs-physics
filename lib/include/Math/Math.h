#pragma once
#include <cmath>

namespace epl::Math
{
	inline constexpr float fAbs(float value) noexcept
	{
		return value < 0.0f ? -value : value;
	}

	inline constexpr bool fEquals(float a, float b, float epsilon = 0.0005f) noexcept
	{
		return fAbs(a - b) < epsilon;
	}

	inline constexpr bool fEqualsZero(float a, float epsilon = 0.0005f) noexcept
	{
		return fEquals(a, 0.0f, epsilon);
	}

	inline constexpr float sqrtNewtonRaphson(float x, float curr, float prev) noexcept
	{
		return curr == prev ? curr : sqrtNewtonRaphson(x, 0.5f * (curr + x / curr), curr);
	}

	// Public constexpr sqrt function
	inline constexpr float sqrt(float x) noexcept
	{
		return x >= 0.0f
			? (x == 0.0f ? 0.0f : sqrtNewtonRaphson(x, x, 0.0f))
			: throw "sqrt: negative input";
	}
}