#ifndef RATE_LIMITER_H
#define RATE_LIMITER_H
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief provides control to rate limit any call
 * @param freq the frequency that is being rate limiting to
 * @param current_ms the current time
 * @param p_last_ms pointer to the last operated time (this will be updated automatically)
 * @return if it's time to run the call
 */
static inline bool rate_limiter(const uint16_t freq, double current_ms, double * p_last_ms) {
    // make sure the data is within reason
    if ((0 <= current_ms) && (0 <= (*p_last_ms)) && (current_ms > (*p_last_ms))) {
        
        if ((current_ms - (*p_last_ms)) >= (1000 / freq)) {
            *p_last_ms = current_ms;
            return true;
        }
    }
    return false;
}

#endif