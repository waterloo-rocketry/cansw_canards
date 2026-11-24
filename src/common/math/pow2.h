/**
 * Power-of-two helper, used instead of bit shifts for readability.
 */
#ifndef COMMON_MATH_POW2_H
#define COMMON_MATH_POW2_H

#include <stdint.h>

/**
 * @brief Compute 2^n.
 * @param n exponent, valid range [0, 31]
 * @return 2^n as a uint32_t (wraps to 0 for n >= 32)
 */
static inline uint32_t pow2_u32(uint32_t n) {
	uint32_t result = 1U;
	for (uint32_t i = 0U; i < n; i++) {
		result *= 2U;
	}
	return result;
}

#endif // COMMON_MATH_POW2_H
