/* 
 * Compile and run with:
 * $ g++ -std=c++0x -Wall -I /usr/local/include/eigen3 cgains.cc test_cg.cc -o test_cg
 * $ ./test_cg
 */
#include <cstdio>
#include <cstdlib>
#include "cgains.h"

namespace {
	bool eq_zero(float f) {
		if (f < 0.0000001f && f > -0.0000001f)
			return true;
		return false;
	}
} /* namespace */

int main(int argc, char** argv) {
	for (int i = 0; i < 9; ++i) {
		auto gains_k = cg::gains[i]; /* gains at a specific speed */
		float theta_r_dot = gains_k->theta_R_dot;
		float A00, A44, B00, B41, C00, C04;
		A00 = gains_k->A[0];
		A44 = gains_k->A[24];
		B00 = gains_k->B[0];
		B41 = gains_k->B[9];
		C00 = gains_k->C[0];
		C04 = gains_k->C[4];

		if (eq_zero(A00) || eq_zero(A44) || eq_zero(B00) ||
			eq_zero(B41) || eq_zero(C00) || eq_zero(C04)) {
			printf("For i = %d, closest speed = %f\n", i, theta_r_dot);
			printf("A(0, 0) = %f\n", A00);
			printf("A(4, 4) = %f\n", A44);
			printf("B(0, 0) = %f\n", B00);
			printf("B(4, 1) = %f\n", B41);
			printf("C(0, 0) = %f\n", C00);
			printf("C(0, 4) = %f\n", C04);
			printf("TEST FAILED\n");
			return EXIT_FAILURE;
		}
	}

	printf("TEST PASSED\n");
	return EXIT_SUCCESS;
}
