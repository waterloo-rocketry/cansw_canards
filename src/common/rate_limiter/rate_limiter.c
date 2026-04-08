
#include <stdbool.h>
#include <stdint.h>

#include "common/rate_limiter/rate_limiter.h"

/**
 * @brief provides control to rate limit any call
 * @param freq the frequency that is being rate limiting to
 * @param current_ms the current time
 * @param p_last_ms pointer to the last operated time (this will be updated automatically)
 * @return if it's time to run the call
 */
bool rate_limiter(const uint16_t freq, const uint32_t current_ms, uint32_t *p_last_ms) {
	// make sure the data is within reason
	if (current_ms > (*p_last_ms)) { // Overflow should never occur, since the clock would overflow
									 // first before uint32
		if (((uint64_t)(current_ms - (*p_last_ms)) * (uint64_t)freq) >=
			1000LL) { // we are casting both numbers to 64 bit to allow larger time gaps with higher
					  // frequency
			*p_last_ms = current_ms;
			return true;
		}
	}
	return false;
}
