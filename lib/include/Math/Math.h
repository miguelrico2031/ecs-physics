#pragma once
#include <cmath>

namespace epl::Math
{
	inline constexpr float abs(float value) noexcept
	{
		return value < 0.0f ? -value : value;
	}

	inline constexpr bool equals(float a, float b, float epsilon = 0.0005f) noexcept
	{
		return abs(a - b) < epsilon;
	}

	inline constexpr bool equalsZero(float a, float epsilon = 0.0005f) noexcept
	{
		return equals(a, 0.0f, epsilon);
	}

	inline constexpr float sqrtNewtonRaphson(float x, float curr, float prev) noexcept
	{
		return curr == prev ? curr : sqrtNewtonRaphson(x, 0.5f * (curr + x / curr), curr);
	}

	// Public constexpr sqrt function
	inline constexpr float sqrt(float x) noexcept
	{
		return x > 0.f ? sqrtNewtonRaphson(x, x, 0.f) : 0.f;
	}
}