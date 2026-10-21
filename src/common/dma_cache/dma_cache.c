#include "common/dma_cache/dma_cache.h"

#include <stdbool.h>

#include "stm32h7xx_hal.h"

static bool dcache_enabled(void) {
	return 0U != (SCB->CCR & SCB_CCR_DC_Msk);
}

void dma_cache_clean(const void *p_buf, uint32_t size) {
	// maintenance on a D-cache that was never enabled can act on garbage lines
	if (dcache_enabled()) {
		SCB_CleanDCache_by_Addr((void *)p_buf, (int32_t)size);
	}
}

void dma_cache_invalidate(void *p_buf, uint32_t size) {
	if (dcache_enabled()) {
		SCB_InvalidateDCache_by_Addr(p_buf, (int32_t)size);
	}
}
