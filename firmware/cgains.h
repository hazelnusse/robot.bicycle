#ifndef CGAINS_H
#define CGAINS_H

#include <array>
#include <cstdint>
#include <utility>

namespace cg {
	const uint32_t num_gains = 101;
	const int32_t a_rows = 5;
	const int32_t a_cols = 5;
	const int32_t b_rows = 5;
	const int32_t b_cols = 3;
	const int32_t c_rows = 1;
	const int32_t c_cols = 5;
	struct ControllerGains {
		std::array<float, a_rows * a_cols> A;
		std::array<float, b_rows * b_cols> B;
		std::array<float, c_rows * c_cols> C;
		float theta_R_dot;
	};
	typedef std::array<ControllerGains, num_gains>::iterator it_t;
	typedef std::array<ControllerGains, num_gains>::const_iterator const_it_t;
	class CGArray {
		public:
			std::array<ControllerGains, num_gains> cg_array;
			ControllerGains* operator[](float x) const;
		private:
			it_t upper_bound(const_it_t first,
							 const_it_t last, float value) const;
	};
	extern CGArray gains;
} /* namespace cg */

#endif /* CGAINS_H */