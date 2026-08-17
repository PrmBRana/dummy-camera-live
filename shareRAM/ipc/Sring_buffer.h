#ifndef __SRING_BUFFER_H
#define __SRING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

/*
 * ============================================================
 * STM32WL55 SHARED SRAM2 RING BUFFER
 * ============================================================
 *
 * CPU1 = Cortex-M4
 * CPU2 = Cortex-M0+
 *
 * SRAM2:
 *
 *     0x20008000 ...
 *
 *
 * IPC message buffer:
 *
 *     0x20008200
 *     size = 256 bytes
 *
 *
 * Ring buffer:
 *
 *     0x20008300
 *
 * ============================================================
 */

#define IPC_RING_BASE       0x20008300UL

#define RING_DEPTH           8U
#define CHUNK_SIZE           80U

#define RING_MAGIC           0x52494E47UL   /* "RING" */


/*
 * ============================================================
 * Shared ring structure
 * ============================================================
 */

struct shared_ring
{
  volatile uint32_t magic;

  volatile uint32_t write_index;

  volatile uint32_t read_index;

  volatile uint32_t count;

  volatile uint8_t packet[RING_DEPTH][CHUNK_SIZE];
};


/*
 * ============================================================
 * Shared ring pointer
 * ============================================================
 *
 * IMPORTANT:
 *
 * This is NOT a normal RAM copy.
 *
 * Both CPUs access the same SRAM2 memory.
 * ============================================================
 */

#define SHARED_RING \
  ((volatile struct shared_ring *)IPC_RING_BASE)


/*
 * ============================================================
 * API
 * ============================================================
 */

void rb_init(void);

bool rb_write(const uint8_t *data);

bool rb_read(uint8_t *data);

bool rb_empty(void);

bool rb_full(void);

uint32_t rb_available(void);

uint32_t rb_free(void);

bool rb_valid(void);

#endif