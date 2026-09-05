#include "ring_buffer.h"
#include <string.h>
#include <nuttx/mutex.h>

/*
 * ring_buffer.c
 *
 * Logic mirrors the Verilog CircularBuffer exactly:
 *
 *   Verilog                     C equivalent
 *   ──────────────────────────  ─────────────────────────────────
 *   wr_en && !full  → write     rb_write()  checks count < RB_DEPTH
 *   rd_en && !empty → read      rb_read()   checks count > 0
 *   wr_ptr <= wr_ptr + 1        g_rb.wr_ptr++ then % RB_DEPTH for slot
 *   rd_ptr <= rd_ptr + 1        g_rb.rd_ptr++ then % RB_DEPTH for slot
 *   count +1 / -1 / hold        count updated after every write/read
 *   full  = (count == DEPTH)    rb_full()
 *   empty = (count == 0)        rb_empty()
 */

static struct ring_buffer_s g_rb;
static mutex_t              g_rb_lock;

/* ------------------------------------------------------------------ */
void rb_init(void)
{
  memset(&g_rb, 0, sizeof(g_rb));
  g_rb.wr_ptr = 0;
  g_rb.rd_ptr = 0;
  g_rb.count  = 0;
  nxmutex_init(&g_rb_lock);
}

/* ------------------------------------------------------------------ */
/*  Write one chunk                                                    */
/*  Verilog: if (wr_en && !full) { mem[wr_ptr] <= wr_data;           */
/*                                  wr_ptr <= wr_ptr + 1; count++ }  */
/* ------------------------------------------------------------------ */
int rb_write(const struct image_chunk_s *c)
{
  if (!c) return -1;

  nxmutex_lock(&g_rb_lock);

  /* full check — mirrors: assign full = (count == DEPTH) */
  if (g_rb.count >= RB_DEPTH)
    {
      nxmutex_unlock(&g_rb_lock);
      return -1;   /* full — caller retries */
    }

  uint32_t slot = g_rb.wr_ptr % RB_DEPTH;           /* slot index      */
  memcpy(&g_rb.slots[slot], c, sizeof(*c));
  g_rb.wr_ptr++;                                     /* advance pointer */
  g_rb.count++;                                      /* update count    */

  nxmutex_unlock(&g_rb_lock);
  return 0;
}

/* ------------------------------------------------------------------ */
/*  Read one chunk                                                     */
/*  Verilog: if (rd_en && !empty) { rd_data = mem[rd_ptr];           */
/*                                   rd_ptr <= rd_ptr + 1; count-- } */
/* ------------------------------------------------------------------ */
int rb_read(struct image_chunk_s *c)
{
  if (!c) return -1;

  nxmutex_lock(&g_rb_lock);

  /* empty check — mirrors: assign empty = (count == 0) */
  if (g_rb.count == 0)
    {
      nxmutex_unlock(&g_rb_lock);
      return -1;   /* empty — caller retries */
    }

  uint32_t slot = g_rb.rd_ptr % RB_DEPTH;           /* slot index      */
  memcpy(c, &g_rb.slots[slot], sizeof(*c));
  g_rb.rd_ptr++;                                     /* advance pointer */
  g_rb.count--;                                      /* update count    */

  nxmutex_unlock(&g_rb_lock);
  return 0;
}

/* ------------------------------------------------------------------ */
int rb_count(void)
{
  nxmutex_lock(&g_rb_lock);
  int n = (int)g_rb.count;
  nxmutex_unlock(&g_rb_lock);
  return n;
}

int rb_full(void)
{
  nxmutex_lock(&g_rb_lock);
  int f = (g_rb.count >= RB_DEPTH) ? 1 : 0;
  nxmutex_unlock(&g_rb_lock);
  return f;
}

int rb_empty(void)
{
  nxmutex_lock(&g_rb_lock);
  int e = (g_rb.count == 0) ? 1 : 0;
  nxmutex_unlock(&g_rb_lock);
  return e;
}

/* ========================================================================= */
/* Telemetry Ring Buffer Implementation (Shared Inter-App)                   */
/* ========================================================================= */

#include <telemetry_rb.h>

struct telemetry_rb_s g_telemetry_rb;
sem_t                 g_telemetry_sem;
static mutex_t        g_telem_lock;
static bool           g_telem_initialized = false;

void telemetry_rb_init(void)
{
  if (!g_telem_initialized)
    {
      nxmutex_init(&g_telem_lock);
      memset(&g_telemetry_rb, 0, sizeof(g_telemetry_rb));
      sem_init(&g_telemetry_sem, 0, 0);
      g_telem_initialized = true;
    }
}

int telemetry_rb_write(const struct telemetry_envelope_s *data)
{
  if (!data)
    {
      return -1;
    }

  nxmutex_lock(&g_telem_lock);

  if (g_telemetry_rb.count >= TELEM_RB_DEPTH)
    {
      /* Buffer full: drop oldest packet */
      g_telemetry_rb.rd_ptr = (g_telemetry_rb.rd_ptr + 1) % TELEM_RB_DEPTH;
      g_telemetry_rb.count--;
    }

  uint32_t slot = g_telemetry_rb.wr_ptr % TELEM_RB_DEPTH;
  g_telemetry_rb.slots[slot] = *data;
  g_telemetry_rb.wr_ptr++;
  g_telemetry_rb.count++;

  nxmutex_unlock(&g_telem_lock);
  return 0;
}

int telemetry_rb_read(struct telemetry_envelope_s *data)
{
  if (!data)
    {
      return -1;
    }

  nxmutex_lock(&g_telem_lock);

  if (g_telemetry_rb.count == 0)
    {
      nxmutex_unlock(&g_telem_lock);
      return -1; /* Empty */
    }

  uint32_t slot = g_telemetry_rb.rd_ptr % TELEM_RB_DEPTH;
  *data = g_telemetry_rb.slots[slot];
  g_telemetry_rb.rd_ptr++;
  g_telemetry_rb.count--;

  nxmutex_unlock(&g_telem_lock);
  return 0;
}

int telemetry_rb_count(void)
{
  nxmutex_lock(&g_telem_lock);
  int c = (int)g_telemetry_rb.count;
  nxmutex_unlock(&g_telem_lock);
  return c;
}