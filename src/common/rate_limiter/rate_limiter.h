
#ifndef COMMON_RATE_LIMITER_H
#define COMMON_RATE_LIMITER_H
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief provides control to rate limit any call
 * @param freq the frequency that is being rate limiting to
 * @param current_ms the current time
 * @param p_last_ms pointer to the last operated time (this will be updated automatically)
 * @return if it's time to run the call
 */
bool rate_limiter(const uint16_t freq, const uint32_t current_ms, uint32_t *p_last_ms);

#endif
