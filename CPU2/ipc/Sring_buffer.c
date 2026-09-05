#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Sring_buffer.h"

/*
 * ============================================================
 * Memory Barrier
 * ============================================================
 * Ensures shared-memory accesses are observed in the correct
 * order across Cortex-M4 and Cortex-M0+.
 */
static inline void rb_memory_barrier(void)
{
  __asm__ volatile ("dmb" ::: "memory");
}

/*
 * ============================================================
 * Standard CCITT-16 CRC Calculation (Polynomial 0x1021)
 * ============================================================
 */
static uint16_t rb_calc_crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < len; i++)
    {
      crc ^= (uint16_t)data[i] << 8;
      for (uint8_t bit = 0; bit < 8; bit++)
        {
          if (crc & 0x8000)
            {
              crc = (crc << 1) ^ 0x1021;
            }
          else
            {
              crc <<= 1;
            }
        }
    }

  return crc;
}

/*
 * ============================================================
 * 1. IPC Subsystem Initialization (Called once by M4 at boot)
 * ============================================================
 */
void rb_ipc_init(void)
{
  /* If already initialized by launcher or previous task, keep intact */
  if (SHARED_RING_TX->magic == RING_TX_MAGIC &&
      SHARED_RING_RX->magic == RING_RX_MAGIC &&
      LIVE_TELEM->magic == LIVE_TELEM_MAGIC &&
      SHARED_RADIO_LOG->magic == RADIO_LOG_MAGIC)
    {
      return;
    }

  /* 1. Reset Live Telemetry Snapshot */
  LIVE_TELEM->magic = 0;
  rb_memory_barrier();

  LIVE_TELEM->batt_voltage    = 3.80f;
  LIVE_TELEM->sat_temperature = 20.0f;
  LIVE_TELEM->batt_volt_mv    = 3800; /* Default 3.80V */
  LIVE_TELEM->sat_temp_c_x10 = 200;  /* Default 20.0 C */
  LIVE_TELEM->timestamp      = 0;
  rb_memory_barrier();

  LIVE_TELEM->magic = LIVE_TELEM_MAGIC;
  rb_memory_barrier();

  /* 2. Reset Ring Buffer 1 (TX: M4 -> M0+) */
  SHARED_RING_TX->magic = 0;
  rb_memory_barrier();

  SHARED_RING_TX->write_index = 0;
  SHARED_RING_TX->read_index  = 0;
  SHARED_RING_TX->count       = 0;
  memset((void *)SHARED_RING_TX->slots, 0, sizeof(SHARED_RING_TX->slots));
  rb_memory_barrier();

  SHARED_RING_TX->magic = RING_TX_MAGIC;
  rb_memory_barrier();

  /* 3. Reset Ring Buffer 2 (RX: M0+ -> M4) */
  SHARED_RING_RX->magic = 0;
  rb_memory_barrier();

  SHARED_RING_RX->write_index = 0;
  SHARED_RING_RX->read_index  = 0;
  SHARED_RING_RX->count       = 0;
  memset((void *)SHARED_RING_RX->slots, 0, sizeof(SHARED_RING_RX->slots));
  rb_memory_barrier();

  SHARED_RING_RX->magic = RING_RX_MAGIC;
  rb_memory_barrier();

  /* 4. Reset Radio Event Log (M0+ -> M4) */
  radio_log_init();
}

/*
 * ============================================================
 * 2. Live Telemetry Snapshot API
 * ============================================================
 */
void live_telem_update_float(float batt_v, float sat_temp)
{
  LIVE_TELEM->batt_voltage    = batt_v;
  LIVE_TELEM->sat_temperature = sat_temp;
  LIVE_TELEM->batt_volt_mv    = (uint16_t)(batt_v * 1000.0f);
  LIVE_TELEM->sat_temp_c_x10  = (int16_t)(sat_temp * 10.0f);
  LIVE_TELEM->timestamp++;
  rb_memory_barrier();
}

bool live_telem_get_float(float *batt_v, float *sat_temp)
{
  if (LIVE_TELEM->magic != LIVE_TELEM_MAGIC)
    {
      return false;
    }

  if (batt_v)
    {
      *batt_v = LIVE_TELEM->batt_voltage;
    }

  if (sat_temp)
    {
      *sat_temp = LIVE_TELEM->sat_temperature;
    }

  return true;
}

void live_telem_update(uint16_t batt_mv, int16_t temp_c_x10)
{
  LIVE_TELEM->batt_voltage    = (float)batt_mv / 1000.0f;
  LIVE_TELEM->sat_temperature = (float)temp_c_x10 / 10.0f;
  LIVE_TELEM->batt_volt_mv    = batt_mv;
  LIVE_TELEM->sat_temp_c_x10  = temp_c_x10;
  LIVE_TELEM->timestamp++;
  rb_memory_barrier();
}

bool live_telem_get(uint16_t *batt_mv, int16_t *temp_c_x10)
{
  if (LIVE_TELEM->magic != LIVE_TELEM_MAGIC)
    {
      return false;
    }

  if (batt_mv)
    {
      *batt_mv = LIVE_TELEM->batt_volt_mv;
    }

  if (temp_c_x10)
    {
      *temp_c_x10 = LIVE_TELEM->sat_temp_c_x10;
    }

  return true;
}

/*
 * ============================================================
 * 3. Ring Buffer 1: TX (M4 -> M0+) API
 * ============================================================
 */
static inline bool rb_tx_valid(void)
{
  return (SHARED_RING_TX->magic == RING_TX_MAGIC) &&
         (SHARED_RING_TX->write_index < RING_TX_DEPTH) &&
         (SHARED_RING_TX->read_index < RING_TX_DEPTH) &&
         (SHARED_RING_TX->count <= RING_TX_DEPTH);
}

bool rb_tx_empty(void)
{
  if (!rb_tx_valid())
    {
      return true;
    }
  return SHARED_RING_TX->count == 0U;
}

bool rb_tx_full(void)
{
  if (!rb_tx_valid())
    {
      return false;
    }
  return SHARED_RING_TX->count >= RING_TX_DEPTH;
}

uint32_t rb_tx_available(void)
{
  if (!rb_tx_valid())
    {
      return 0;
    }
  return SHARED_RING_TX->count;
}

bool rb_tx_write(const struct tx_packet_s *pkt)
{
  if (!pkt || !rb_tx_valid() || rb_tx_full())
    {
      return false;
    }

  uint32_t idx = SHARED_RING_TX->write_index;

  /* Copy packet and compute CRC */
  SHARED_RING_TX->slots[idx].type = pkt->type;
  SHARED_RING_TX->slots[idx].len  = (pkt->len > RING_TX_PAYLOAD_MAX) ? RING_TX_PAYLOAD_MAX : pkt->len;
  memcpy((void *)SHARED_RING_TX->slots[idx].data, pkt->data, SHARED_RING_TX->slots[idx].len);
  SHARED_RING_TX->slots[idx].crc16 = rb_calc_crc16(pkt->data, SHARED_RING_TX->slots[idx].len);

  rb_memory_barrier();

  /* Advance write index atomically */
  SHARED_RING_TX->write_index = (idx + 1) % RING_TX_DEPTH;
  SHARED_RING_TX->count++;

  rb_memory_barrier();
  return true;
}

bool rb_tx_read(struct tx_packet_s *pkt)
{
  if (!pkt || !rb_tx_valid() || rb_tx_empty())
    {
      return false;
    }

  uint32_t idx = SHARED_RING_TX->read_index;

  /* Verify CRC before consuming */
  uint16_t computed_crc = rb_calc_crc16((const uint8_t *)SHARED_RING_TX->slots[idx].data,
                                        SHARED_RING_TX->slots[idx].len);
  if (computed_crc != SHARED_RING_TX->slots[idx].crc16)
    {
      /* CRC corrupted by bit-flip! Drop corrupted packet */
      SHARED_RING_TX->read_index = (idx + 1) % RING_TX_DEPTH;
      SHARED_RING_TX->count--;
      rb_memory_barrier();
      return false;
    }

  /* Copy packet out */
  pkt->type  = SHARED_RING_TX->slots[idx].type;
  pkt->len   = SHARED_RING_TX->slots[idx].len;
  memcpy(pkt->data, (const void *)SHARED_RING_TX->slots[idx].data, pkt->len);
  pkt->crc16 = SHARED_RING_TX->slots[idx].crc16;

  rb_memory_barrier();

  /* Advance read index atomically */
  SHARED_RING_TX->read_index = (idx + 1) % RING_TX_DEPTH;
  SHARED_RING_TX->count--;

  rb_memory_barrier();
  return true;
}

/*
 * ============================================================
 * 4. Ring Buffer 2: RX (M0+ -> M4) API
 * ============================================================
 */
static inline bool rb_rx_valid(void)
{
  return (SHARED_RING_RX->magic == RING_RX_MAGIC) &&
         (SHARED_RING_RX->write_index < RING_RX_DEPTH) &&
         (SHARED_RING_RX->read_index < RING_RX_DEPTH) &&
         (SHARED_RING_RX->count <= RING_RX_DEPTH);
}

bool rb_rx_empty(void)
{
  if (!rb_rx_valid())
    {
      return true;
    }
  return SHARED_RING_RX->count == 0U;
}

bool rb_rx_full(void)
{
  if (!rb_rx_valid())
    {
      return false;
    }
  return SHARED_RING_RX->count >= RING_RX_DEPTH;
}

uint32_t rb_rx_available(void)
{
  if (!rb_rx_valid())
    {
      return 0;
    }
  return SHARED_RING_RX->count;
}

bool rb_rx_write(const struct rx_command_s *cmd)
{
  if (!cmd || !rb_rx_valid() || rb_rx_full())
    {
      return false;
    }

  uint32_t idx = SHARED_RING_RX->write_index;

  /* Copy 13-byte command and calculate CRC */
  SHARED_RING_RX->slots[idx].len = CMD_PAYLOAD_LEN;
  memcpy((void *)SHARED_RING_RX->slots[idx].cmd, cmd->cmd, CMD_PAYLOAD_LEN);
  SHARED_RING_RX->slots[idx].crc16 = rb_calc_crc16(cmd->cmd, CMD_PAYLOAD_LEN);

  rb_memory_barrier();

  /* Advance write index */
  SHARED_RING_RX->write_index = (idx + 1) % RING_RX_DEPTH;
  SHARED_RING_RX->count++;

  rb_memory_barrier();
  return true;
}

bool rb_rx_read(struct rx_command_s *cmd)
{
  if (!cmd || !rb_rx_valid() || rb_rx_empty())
    {
      return false;
    }

  uint32_t idx = SHARED_RING_RX->read_index;

  /* Check CRC for bit-flip protection */
  uint16_t computed_crc = rb_calc_crc16((const uint8_t *)SHARED_RING_RX->slots[idx].cmd,
                                        CMD_PAYLOAD_LEN);
  if (computed_crc != SHARED_RING_RX->slots[idx].crc16)
    {
      /* Corrupt command dropped */
      SHARED_RING_RX->read_index = (idx + 1) % RING_RX_DEPTH;
      SHARED_RING_RX->count--;
      rb_memory_barrier();
      return false;
    }

  cmd->len = CMD_PAYLOAD_LEN;
  memcpy(cmd->cmd, (const void *)SHARED_RING_RX->slots[idx].cmd, CMD_PAYLOAD_LEN);
  cmd->crc16 = SHARED_RING_RX->slots[idx].crc16;

  rb_memory_barrier();

  /* Advance read index */
  SHARED_RING_RX->read_index = (idx + 1) % RING_RX_DEPTH;
  SHARED_RING_RX->count--;

  rb_memory_barrier();
  return true;
}

/*
 * ============================================================
 * 5. Radio Event Log (M0+ -> M4) API
 * ============================================================
 */
void radio_log_init(void)
{
  SHARED_RADIO_LOG->magic = 0;
  rb_memory_barrier();

  SHARED_RADIO_LOG->write_index = 0;
  SHARED_RADIO_LOG->read_index  = 0;
  SHARED_RADIO_LOG->count       = 0;
  memset((void *)SHARED_RADIO_LOG->slots, 0, sizeof(SHARED_RADIO_LOG->slots));
  rb_memory_barrier();

  SHARED_RADIO_LOG->magic = RADIO_LOG_MAGIC;
  rb_memory_barrier();
}

bool radio_log_empty(void)
{
  if (SHARED_RADIO_LOG->magic != RADIO_LOG_MAGIC)
    {
      return true;
    }
  return SHARED_RADIO_LOG->count == 0U;
}

uint32_t radio_log_available(void)
{
  if (SHARED_RADIO_LOG->magic != RADIO_LOG_MAGIC)
    {
      return 0;
    }
  return SHARED_RADIO_LOG->count;
}

bool radio_log_write(uint8_t event_type, const char *msg)
{
  if (!msg || SHARED_RADIO_LOG->magic != RADIO_LOG_MAGIC)
    {
      return false;
    }

  uint32_t idx = SHARED_RADIO_LOG->write_index;

  SHARED_RADIO_LOG->slots[idx].event_type = event_type;
  SHARED_RADIO_LOG->slots[idx].timestamp_ms = 0;
  strncpy((char *)SHARED_RADIO_LOG->slots[idx].text, msg, RADIO_LOG_TEXT_MAX - 1);
  SHARED_RADIO_LOG->slots[idx].text[RADIO_LOG_TEXT_MAX - 1] = '\0';

  rb_memory_barrier();

  SHARED_RADIO_LOG->write_index = (idx + 1) % RADIO_LOG_DEPTH;
  if (SHARED_RADIO_LOG->count < RADIO_LOG_DEPTH)
    {
      SHARED_RADIO_LOG->count++;
    }
  else
    {
      /* Drop oldest slot if buffer is full */
      SHARED_RADIO_LOG->read_index = (SHARED_RADIO_LOG->read_index + 1) % RADIO_LOG_DEPTH;
    }

  rb_memory_barrier();
  return true;
}

bool radio_log_read(struct radio_log_msg_s *msg)
{
  if (!msg || SHARED_RADIO_LOG->magic != RADIO_LOG_MAGIC || SHARED_RADIO_LOG->count == 0U)
    {
      return false;
    }

  uint32_t idx = SHARED_RADIO_LOG->read_index;

  msg->event_type   = SHARED_RADIO_LOG->slots[idx].event_type;
  msg->timestamp_ms = SHARED_RADIO_LOG->slots[idx].timestamp_ms;
  strncpy(msg->text, (const char *)SHARED_RADIO_LOG->slots[idx].text, RADIO_LOG_TEXT_MAX);
  msg->text[RADIO_LOG_TEXT_MAX - 1] = '\0';

  rb_memory_barrier();

  SHARED_RADIO_LOG->read_index = (idx + 1) % RADIO_LOG_DEPTH;
  SHARED_RADIO_LOG->count--;

  rb_memory_barrier();
  return true;
}