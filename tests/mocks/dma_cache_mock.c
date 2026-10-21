/**
 * @file dma_cache_mock.c
 * @brief No-op D-cache maintenance for unit tests: the host has no D-cache to keep coherent
 */

#include "common/dma_cache/dma_cache.h"

void dma_cache_clean(const void *p_buf, uint32_t size) {
    (void)p_buf;
    (void)size;
}

void dma_cache_invalidate(void *p_buf, uint32_t size) {
    (void)p_buf;
    (void)size;
}
