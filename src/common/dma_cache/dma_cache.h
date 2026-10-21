/**
 * Keeping DMA buffers coherent with the Cortex-M7 D-cache
 *
 * The D-cache is write-back and DMA bypasses it, so:
 * - before DMA reads a buffer (TX), clean it so the CPU's writes reach RAM
 * - after DMA writes a buffer (RX), invalidate it so the CPU stops reading its old cached copy
 *
 * Invalidation drops whole 32-byte cache lines. Every buffer DMA writes into must be declared with
 * DMA_BUFFER_ALIGN and sized with DMA_BUFFER_SIZE so no other variable shares its lines, and the
 * CPU must not write to it.
 */
#ifndef DMA_CACHE_H
#define DMA_CACHE_H

#include <stdint.h>

#define DCACHE_LINE_SIZE 32U

// start a DMA receive buffer on its own cache line
#define DMA_BUFFER_ALIGN __attribute__((aligned(DCACHE_LINE_SIZE)))

// round a DMA receive buffer's size up to whole cache lines
#define DMA_BUFFER_SIZE(bytes)                                                                     \
	((((bytes) + DCACHE_LINE_SIZE - 1U) / DCACHE_LINE_SIZE) * DCACHE_LINE_SIZE)

/**
 * @brief Write the CPU's cached changes to a buffer back to RAM before DMA reads it
 * @param p_buf buffer DMA is about to read, any alignment
 * @param size buffer size in bytes
 */
void dma_cache_clean(const void *p_buf, uint32_t size);

/**
 * @brief Drop cached copies of a buffer DMA has written so the CPU reads the new data from RAM
 * @param p_buf buffer DMA wrote, declared with DMA_BUFFER_ALIGN and DMA_BUFFER_SIZE
 * @param size bytes DMA wrote
 */
void dma_cache_invalidate(void *p_buf, uint32_t size);

#endif
