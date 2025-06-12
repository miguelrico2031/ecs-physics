#pragma once
//#include <cmath>

namespace epl::Math
{
	inline constexpr float epsilon() noexcept
	{
		return 0.0001f;
	}
	inline constexpr float abs(float value) noexcept
	{
		return value < 0.0f ? -value : value;
	}

	inline constexpr bool equals(float a, float b, float epsilon = 0.0001f) noexcept
	{
		return abs(a - b) < epsilon;
	}

	inline constexpr bool equalsZero(float a, float epsilon = 0.0001f) noexcept
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

	inline constexpr float clamp(float value, float min, float max) noexcept
	{
		return value < min ? min : (value > max ? max : value);
	}

	inline constexpr float max(float a, float b) noexcept
	{
		return a > b ? a : b;
	}

	inline constexpr float min(float a, float b) noexcept
	{
		return a < b ? a : b;
	}
}