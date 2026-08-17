#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "share_ipc.h"

/*
 * ============================================================
 * Delay
 * ============================================================
 */

static void ipc_delay(void)
{
  volatile uint32_t i;

  for (i = 0; i < 100000; i++)
    {
      __asm__ volatile ("nop");
    }
}

/*
 * ============================================================
 * Antarikchya Pratisthan Nepal text
 * ============================================================
 */

static const char antarikchya_text[] =
"Antarikchya Pratisthan Nepal (Space Foundation Nepal) is a Kathmandu-based "
"non-profit organization founded in 2020 by Dr. Abhas Maskey, the aerospace "
"engineer behind Nepal's historic first satellite, NepaliSat-1. Dedicated to "
"building a sustainable, student-driven space ecosystem, the foundation "
"actively trains the next generation of Nepali space engineers and scientists "
"through hands-on STEM bootcamps, CanSat workshops, and real-world satellite "
"development. The organization spearheads key national aerospace projects, "
"including Project Munal (Nepal's first student-built 1U CubeSat) and the "
"Slippers2Sat initiative, alongside climate monitoring collaborations to "
"build low-cost sensors for the Himalayas. Guided by its ambitious Vision "
"2050, the foundation works closely with government and academic stakeholders "
"to establish Nepal as a regional hub for space technology, with the ultimate "
"long-term goal of placing the first Nepali astronaut in space by the year "
"2050.";

/*
 * ============================================================
 * Print packet
 * ============================================================
 */

static void print_packet(uint32_t slot)
{
  uint32_t i;

  printf("M4: slot     = %lu\r\n",
         (unsigned long)slot);

  printf("M4: sequence = %lu\r\n",
         (unsigned long)
         IPC_SHARED->ring[slot].sequence);

  printf("M4: length   = %lu\r\n",
         (unsigned long)
         IPC_SHARED->ring[slot].length);

  printf("M4: data = ");

  for (i = 0;
       i < IPC_SHARED->ring[slot].length;
       i++)
    {
      printf("%c",
             IPC_SHARED->ring[slot].data[i]);
    }

  printf("\r\n");
}

/*
 * ============================================================
 * Send HELLO
 * ============================================================
 */

static void send_hello(void)
{
  const char hello[] = "HELLO M0";

  /*
   * Use ring slot 0 for handshake.
   */

  IPC_SHARED->ring[0].sequence = 1;

  IPC_SHARED->ring[0].length = sizeof(hello) - 1;

  memcpy((void *)IPC_SHARED->ring[0].data,
         hello,
         sizeof(hello) - 1);

  memory_barrier();

  IPC_SHARED->state = IPC_STATE_M4_HELLO;

  memory_barrier();

  printf("\r\n");
  printf("----------------------------------------\r\n");
  printf(" M4 -> M0+ : HELLO\r\n");
  printf("----------------------------------------\r\n");

  printf("M4: data = HELLO M0\r\n");
  printf("M4: length = %lu\r\n",
         (unsigned long)(sizeof(hello) - 1));

  printf("M4: WAITING FOR ACK...\r\n");

  /*
   * Wait for M0 ACK.
   */

  while (IPC_SHARED->state != IPC_STATE_M0_ACK)
    {
      ipc_delay();
    }

  memory_barrier();

  printf("M0+ -> M4 : ACK RECEIVED\r\n");

  /*
   * Return to idle.
   */

  IPC_SHARED->state = IPC_STATE_IDLE;

  memory_barrier();
}

/*
 * ============================================================
 * Send one text packet
 * ============================================================
 */

static void send_text_packet(uint32_t slot,
                             uint32_t sequence,
                             uint32_t offset,
                             uint32_t length)
{
  /*
   * Fill packet metadata.
   */

  IPC_SHARED->ring[slot].sequence = sequence;

  IPC_SHARED->ring[slot].length = length;

  /*
   * Copy text into shared RAM.
   */

  memcpy((void *)IPC_SHARED->ring[slot].data,
         &antarikchya_text[offset],
         length);

  /*
   * Make sure packet data is visible before state change.
   */

  memory_barrier();

  printf("\r\n");
  printf("========================================\r\n");
  printf(" M4 -> M0+ : TEXT PACKET\r\n");
  printf("========================================\r\n");

  printf("M4: slot     = %lu\r\n",
         (unsigned long)slot);

  printf("M4: sequence = %lu\r\n",
         (unsigned long)sequence);

  printf("M4: offset   = %lu\r\n",
         (unsigned long)offset);

  printf("M4: length   = %lu\r\n",
         (unsigned long)length);

  /*
   * Show packet text locally.
   */

  printf("M4: DATA:\r\n");

  print_packet(slot);

  /*
   * Advance write pointer.
   */

  IPC_SHARED->ring_write =
      (slot + 1) % IPC_RING_SIZE;

  IPC_SHARED->burst_count++;

  memory_barrier();

  /*
   * Notify M0.
   */

  IPC_SHARED->state = IPC_STATE_M4_BURST;

  memory_barrier();

  printf("M4 -> M0+: TEXT PACKET SENT\r\n");

  /*
   * Wait for ACK.
   */

  printf("M4: WAITING FOR ACK...\r\n");

  while (IPC_SHARED->state != IPC_STATE_M0_BURST_ACK)
    {
      ipc_delay();
    }

  memory_barrier();

  printf("M0+ -> M4: TEXT PACKET ACK RECEIVED\r\n");

  /*
   * Return to idle.
   */

  IPC_SHARED->state = IPC_STATE_IDLE;

  memory_barrier();
}

/*
 * ============================================================
 * Send complete text
 * ============================================================
 */

static void send_complete_text(void)
{
  uint32_t total;
  uint32_t offset;
  uint32_t length;
  uint32_t slot;
  uint32_t sequence;

  total = strlen(antarikchya_text);

  offset = 0;

  sequence = 2;

  printf("\r\n");
  printf("########################################\r\n");
  printf(" M4 -> M0+ : ANTARIKCHYA TEXT TRANSFER\r\n");
  printf("########################################\r\n");

  printf("M4: total text length = %lu bytes\r\n",
         (unsigned long)total);

  /*
   * Continue until complete text is sent.
   */

  while (offset < total)
    {
      /*
       * Current ring slot.
       */

      slot = IPC_SHARED->ring_write;

      /*
       * Remaining data.
       */

      length = total - offset;

      /*
       * Limit to one packet.
       */

      if (length > IPC_DATA_SIZE)
        {
          length = IPC_DATA_SIZE;
        }

      /*
       * Send packet.
       */

      send_text_packet(slot,
                       sequence,
                       offset,
                       length);

      /*
       * Move to next chunk.
       */

      offset += length;

      sequence++;
    }

  printf("\r\n");
  printf("########################################\r\n");
  printf(" M4: COMPLETE TEXT SENT\r\n");
  printf("########################################\r\n");

  printf("M4: total bytes = %lu\r\n",
         (unsigned long)total);

  printf("M4: total packets = %lu\r\n",
         (unsigned long)(sequence - 2));
}

/*
 * ============================================================
 * MAIN
 * ============================================================
 */

int main(int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  printf("\r\n");
  printf("========================================\r\n");
  printf(" STM32WL55 M4 IPC RING BUFFER TEST\r\n");
  printf("========================================\r\n");

  printf("M4: START\r\n");

  printf("M4: shared address = 0x%08lX\r\n",
         (unsigned long)IPC_SHARED_ADDR);

  printf("M4: data size = %lu\r\n",
         (unsigned long)IPC_DATA_SIZE);

  printf("M4: ring size = %lu\r\n",
         (unsigned long)IPC_RING_SIZE);

  /*
   * ==========================================================
   * Initialize shared RAM
   * ==========================================================
   */

  IPC_SHARED->magic = IPC_MAGIC;

  IPC_SHARED->state = IPC_STATE_IDLE;

  IPC_SHARED->sequence = 0;

  IPC_SHARED->length = 0;

  IPC_SHARED->burst_count = 0;

  IPC_SHARED->ring_write = 0;

  IPC_SHARED->ring_read = 0;

  memory_barrier();

  printf("M4: SHARED RAM INITIALIZED\r\n");

  /*
   * ==========================================================
   * HANDSHAKE
   * ==========================================================
   */

  send_hello();

  /*
   * ==========================================================
   * SEND ANTARIKCHYA TEXT
   * ==========================================================
   */

  send_complete_text();

  /*
   * ==========================================================
   * DONE
   * ==========================================================
   */

  IPC_SHARED->state = IPC_STATE_DONE;

  memory_barrier();

  printf("\r\n");
  printf("========================================\r\n");
  printf(" M4 IPC TEST SUCCESS\r\n");
  printf("========================================\r\n");

  printf("M4: burst count = %lu\r\n",
         (unsigned long)IPC_SHARED->burst_count);

  while (1)
    {
      ipc_delay();
    }

  return 0;
}