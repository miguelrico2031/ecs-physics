#include <Physics/World.h>
#include <iostream>
#include <Math/Matrix3x3.h>

int main()
{
	epl::Matrix3x3 mat1;
	mat1[0][0] = 3;  mat1[0][1] = 1;  mat1[0][2] = 7;
	mat1[1][0] = 3;  mat1[1][1] = 2;  mat1[1][2] = 4;
	mat1[2][0] = -1; mat1[2][1] = -2; mat1[2][2] = 4;

	epl::Matrix3x3 mat2;
	mat2[0][0] = -8; mat2[0][1] = 1;   mat2[0][2] = 0;
	mat2[1][0] = 2;  mat2[1][1] = 4;   mat2[1][2] = 1;
	mat2[2][0] = 4;  mat2[2][1] = -10.1f; mat2[2][2] = 2;

	std::cout << "m1:\n" << mat1;
	std::cout << "m2:\n" << mat2;
	std::cout << "m1 x m2:\n" << (mat1 * mat2);
}