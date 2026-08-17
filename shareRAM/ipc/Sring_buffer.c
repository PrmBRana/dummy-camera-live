#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Sring_buffer.h"


/*
 * ============================================================
 * Shared ring pointer
 * ============================================================
 */

#define RING_PTR   SHARED_RING


/*
 * ============================================================
 * Memory barrier
 * ============================================================
 *
 * Ensures shared-memory accesses are observed in the
 * correct order between CPU1 and CPU2.
 * ============================================================
 */

static inline void rb_memory_barrier(void)
{
  __asm__ volatile ("dmb sy" ::: "memory");
}


/*
 * ============================================================
 * Initialize ring
 * ============================================================
 *
 * M4/CPU1 should call this before starting the first transfer.
 *
 * M0+ must NOT call rb_init() after the ring is active.
 * ============================================================
 */

void rb_init(void)
{
  uint32_t i;

  /*
   * Invalidate first.
   */

  RING_PTR->magic = 0;

  rb_memory_barrier();

  /*
   * Reset indexes.
   */

  RING_PTR->write_index = 0;
  RING_PTR->read_index  = 0;
  RING_PTR->count       = 0;

  /*
   * Clear packet storage.
   */

  for (i = 0; i < RING_DEPTH; i++)
    {
      memset((void *)RING_PTR->packet[i],
             0,
             CHUNK_SIZE);
    }

  rb_memory_barrier();

  /*
   * Magic written LAST.
   */

  RING_PTR->magic = RING_MAGIC;

  rb_memory_barrier();
}


/*
 * ============================================================
 * Check ring validity
 * ============================================================
 */

bool rb_valid(void)
{
  uint32_t magic;
  uint32_t write_index;
  uint32_t read_index;
  uint32_t count;

  magic       = RING_PTR->magic;
  write_index = RING_PTR->write_index;
  read_index  = RING_PTR->read_index;
  count       = RING_PTR->count;

  if (magic != RING_MAGIC)
    {
      return false;
    }

  if (write_index >= RING_DEPTH)
    {
      return false;
    }

  if (read_index >= RING_DEPTH)
    {
      return false;
    }

  if (count > RING_DEPTH)
    {
      return false;
    }

  return true;
}


/*
 * ============================================================
 * Is empty?
 * ============================================================
 */

bool rb_empty(void)
{
  if (!rb_valid())
    {
      return true;
    }

  return RING_PTR->count == 0U;
}


/*
 * ============================================================
 * Is full?
 * ============================================================
 */

bool rb_full(void)
{
  if (!rb_valid())
    {
      return false;
    }

  return RING_PTR->count >= RING_DEPTH;
}


/*
 * ============================================================
 * Number of packets available
 * ============================================================
 */

uint32_t rb_available(void)
{
  if (!rb_valid())
    {
      return UINT32_MAX;
    }

  return RING_PTR->count;
}


/*
 * ============================================================
 * Number of free packet slots
 * ============================================================
 */

uint32_t rb_free(void)
{
  uint32_t count;

  count = rb_available();

  if (count == UINT32_MAX)
    {
      return 0;
    }

  return RING_DEPTH - count;
}


/*
 * ============================================================
 * Write packet
 *
 * Normally called by M4/CPU1.
 * ============================================================
 */

bool rb_write(const uint8_t *data)
{
  uint32_t index;

  if (data == NULL)
    {
      return false;
    }

  if (!rb_valid())
    {
      return false;
    }

  if (rb_full())
    {
      return false;
    }

  index = RING_PTR->write_index;

  /*
   * Write packet first.
   */

  memcpy((void *)RING_PTR->packet[index],
         data,
         CHUNK_SIZE);

  rb_memory_barrier();

  /*
   * Advance write index.
   */

  index++;

  if (index >= RING_DEPTH)
    {
      index = 0;
    }

  RING_PTR->write_index = index;

  rb_memory_barrier();

  /*
   * Publish packet.
   */

  RING_PTR->count++;

  rb_memory_barrier();

  return true;
}


/*
 * ============================================================
 * Read packet
 *
 * Normally called by M0+/CPU2.
 * ============================================================
 */

bool rb_read(uint8_t *data)
{
  uint32_t index;

  if (data == NULL)
    {
      return false;
    }

  if (!rb_valid())
    {
      return false;
    }

  if (rb_empty())
    {
      return false;
    }

  index = RING_PTR->read_index;

  /*
   * Read packet.
   */

  memcpy(data,
         (const void *)RING_PTR->packet[index],
         CHUNK_SIZE);

  rb_memory_barrier();

  /*
   * Advance read index.
   */

  index++;

  if (index >= RING_DEPTH)
    {
      index = 0;
    }

  RING_PTR->read_index = index;

  rb_memory_barrier();

  /*
   * Packet consumed.
   */

  RING_PTR->count--;

  rb_memory_barrier();

  return true;
}