#ifndef __SRING_BUFFER_H
#define __SRING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

/*
 * ============================================================
 * STM32WL55 SHARED SRAM2 DUAL RING BUFFER & LIVE TELEMETRY
 * ============================================================
 * CPU1 = Cortex-M4 (NuttX OBC)
 * CPU2 = Cortex-M0+ (Radio Subsystem)
 *
 * Physical Memory Layout in SRAM2 (0x20008000 ...):
 *   0x20008000 - 0x200081FF: Mailbox / Handshake
 *   0x20008200 - 0x200082FF: Live Telemetry Snapshot (Batt V & Temp for CW)
 *   0x20008300 - 0x200085FF: Ring Buffer 1 (TX: M4 -> M0+) [B1, B2, ACK, NACK]
 *   0x20008600 - 0x200087FF: Ring Buffer 2 (RX: M0+ -> M4) [Ground Commands]
 * ============================================================
 */

/* ------------------------------------------------------------
 * 1. Live Telemetry Snapshot (for continuous CW Morse Beacon)
 * ------------------------------------------------------------ */
#define IPC_LIVE_TELEM_BASE   0x20008200UL
#define LIVE_TELEM_MAGIC      0x54454C4DUL  /* "TELM" */

struct live_telemetry_s
{
  volatile uint32_t magic;
  volatile float    batt_voltage;      /* Float Battery Voltage in Volts (e.g. 3.84f) */
  volatile float    sat_temperature;   /* Float Satellite Temperature in °C (e.g. 22.5f) */
  volatile uint16_t batt_volt_mv;      /* Battery Voltage in mV (e.g. 3840) */
  volatile int16_t  sat_temp_c_x10;    /* Satellite Temperature in C * 10 (e.g. 225 = 22.5 C) */
  volatile uint32_t timestamp;         /* System scan counter or uptime tick */
};

#define LIVE_TELEM \
  ((volatile struct live_telemetry_s *)IPC_LIVE_TELEM_BASE)

/* ------------------------------------------------------------
 * 2. Ring Buffer 1: TX (M4 -> M0+) [Telemetry Beacons & Responses]
 * ------------------------------------------------------------ */
#define IPC_RING_TX_BASE      0x20008300UL
#define RING_TX_MAGIC         0x54585242UL  /* "TXRB" */
#define RING_TX_DEPTH         8U
#define RING_TX_PAYLOAD_MAX   64U

#define PKT_TYPE_B1           0x01  /* Beacon 1: HK1 ADC1 (34 Bytes) */
#define PKT_TYPE_B2           0x02  /* Beacon 2: HK2 ADC2 + IMU (38 Bytes) */
#define PKT_TYPE_ACK          0x0A  /* Command Acknowledged (e.g. Camera Data Available) */
#define PKT_TYPE_NACK         0x0E  /* Command Rejected (e.g. Camera Data NOT Available) */

struct tx_packet_s
{
  uint8_t  type;                      /* PKT_TYPE_B1, B2, ACK, NACK */
  uint8_t  len;                       /* Payload length */
  uint8_t  data[RING_TX_PAYLOAD_MAX]; /* Payload bytes */
  uint16_t crc16;                     /* CRC-16 Checksum */
};

struct shared_ring_tx
{
  volatile uint32_t magic;
  volatile uint32_t write_index;
  volatile uint32_t read_index;
  volatile uint32_t count;
  struct tx_packet_s slots[RING_TX_DEPTH];
};

#define SHARED_RING_TX \
  ((volatile struct shared_ring_tx *)IPC_RING_TX_BASE)

/* ------------------------------------------------------------
 * 3. Ring Buffer 2: RX (M0+ -> M4) [Ground Uplink Commands]
 * ------------------------------------------------------------ */
#define IPC_RING_RX_BASE      0x20008600UL
#define RING_RX_MAGIC         0x52585242UL  /* "RXRB" */
#define RING_RX_DEPTH         4U
#define CMD_PAYLOAD_LEN       13U

/* Command Type Opcodes (Byte 1) */
#define CMD_TYPE_HK           0x01  /* Housekeeping Request */
#define CMD_TYPE_ADCS         0x03  /* ADCS Control */
#define CMD_TYPE_CAMERA       0x04  /* Camera Command: 53 04 CC 5E BD 00... */
#define CMD_TYPE_EPDM         0x05  /* EPDM Payload Command */

struct rx_command_s
{
  uint8_t  len;                       /* Always 13 Bytes */
  uint8_t  cmd[CMD_PAYLOAD_LEN];      /* 13-Byte raw command frame */
  uint16_t crc16;                     /* CRC-16 Checksum */
};

struct shared_ring_rx
{
  volatile uint32_t magic;
  volatile uint32_t write_index;
  volatile uint32_t read_index;
  volatile uint32_t count;
  struct rx_command_s slots[RING_RX_DEPTH];
};

#define SHARED_RING_RX \
  ((volatile struct shared_ring_rx *)IPC_RING_RX_BASE)

/* ------------------------------------------------------------
 * 4. Hardware IPCC Register Definitions & Doorbells
 * ------------------------------------------------------------ */
#ifndef IPCC_BASE
#  define IPCC_BASE           0x58000C00UL
#endif
#define IPCC_C1SCR_OFFSET     0x0008UL
#define IPCC_C1TOC2SR_OFFSET  0x000CUL
#define IPCC_C2SCR_OFFSET     0x0018UL
#define IPCC_C2TOC1SR_OFFSET  0x001CUL

#define IPCC_C1SCR \
  (*(volatile uint32_t *)(IPCC_BASE + IPCC_C1SCR_OFFSET))
#define IPCC_C1TOC2SR \
  (*(volatile uint32_t *)(IPCC_BASE + IPCC_C1TOC2SR_OFFSET))
#define IPCC_C2SCR \
  (*(volatile uint32_t *)(IPCC_BASE + IPCC_C2SCR_OFFSET))
#define IPCC_C2TOC1SR \
  (*(volatile uint32_t *)(IPCC_BASE + IPCC_C2TOC1SR_OFFSET))

#define IPCC_CH_BEACON        1U  /* Channel 1: M4 -> M0+ (Beacon/CW/ACK ready) */
#define IPCC_CH_COMMAND       2U  /* Channel 2: M0+ -> M4 (Uplink command received) */
#define IPCC_CH_HANDSHAKE     3U  /* Channel 3: M4 <-> M0+ Initial Handshake */

#define IPCC_RX_BIT(ch)       (1UL << ((ch) - 1U))
#define IPCC_TX_BIT(ch)       (1UL << (((ch) - 1U) + 16U))

/* M4 Doorbell API */
static inline void ipcc_m4_send(uint32_t ch)
{
  IPCC_C1SCR = IPCC_TX_BIT(ch);
  __asm__ volatile ("dmb" ::: "memory");
}

static inline bool ipcc_m4_received(uint32_t ch)
{
  return (IPCC_C2TOC1SR & IPCC_RX_BIT(ch)) != 0U;
}

static inline void ipcc_m4_clear(uint32_t ch)
{
  IPCC_C1SCR = IPCC_RX_BIT(ch);
  __asm__ volatile ("dmb" ::: "memory");
}

/* M0+ Doorbell API */
static inline void ipcc_m0_send(uint32_t ch)
{
  IPCC_C2SCR = IPCC_TX_BIT(ch);
  __asm__ volatile ("dmb" ::: "memory");
}

static inline bool ipcc_m0_received(uint32_t ch)
{
  return (IPCC_C1TOC2SR & IPCC_RX_BIT(ch)) != 0U;
}

static inline void ipcc_m0_clear(uint32_t ch)
{
  IPCC_C2SCR = IPCC_RX_BIT(ch);
  __asm__ volatile ("dmb" ::: "memory");
}

/* ------------------------------------------------------------
 * Public IPC API
 * ------------------------------------------------------------ */

/* Core Initialization (Called by M4 at boot) */
void rb_ipc_init(void);

/* Live Telemetry Snapshot API */
void live_telem_update(uint16_t batt_mv, int16_t temp_c_x10);
bool live_telem_get(uint16_t *batt_mv, int16_t *temp_c_x10);
void live_telem_update_float(float batt_v, float sat_temp);
bool live_telem_get_float(float *batt_v, float *sat_temp);

/* Ring Buffer 1 (TX: M4 -> M0+) */
bool rb_tx_write(const struct tx_packet_s *pkt);
bool rb_tx_read(struct tx_packet_s *pkt);
bool rb_tx_empty(void);
bool rb_tx_full(void);
uint32_t rb_tx_available(void);

/* ------------------------------------------------------------
 * 4. Radio Event Log Ring Buffer (M0+ -> M4)
 * ------------------------------------------------------------ */
#define IPC_RADIO_LOG_BASE    0x20008800UL
#define RADIO_LOG_MAGIC       0x524C4F47UL  /* "RLOG" */
#define RADIO_LOG_DEPTH       8U
#define RADIO_LOG_TEXT_MAX    88U

#define RADIO_EVT_CW          0x01
#define RADIO_EVT_B1          0x02
#define RADIO_EVT_B2          0x03
#define RADIO_EVT_RX_CMD      0x04
#define RADIO_EVT_TX_ACK      0x05
#define RADIO_EVT_TX_NACK     0x06
#define RADIO_EVT_INFO        0x07

struct radio_log_msg_s
{
  uint32_t timestamp_ms;
  uint8_t  event_type;
  char     text[RADIO_LOG_TEXT_MAX];
};

struct shared_radio_log_s
{
  volatile uint32_t magic;
  volatile uint32_t write_index;
  volatile uint32_t read_index;
  volatile uint32_t count;
  struct radio_log_msg_s slots[RADIO_LOG_DEPTH];
};

#define SHARED_RADIO_LOG \
  ((volatile struct shared_radio_log_s *)IPC_RADIO_LOG_BASE)

/* Ring Buffer 2 (RX: M0+ -> M4) */
bool rb_rx_write(const struct rx_command_s *cmd);
bool rb_rx_read(struct rx_command_s *cmd);
bool rb_rx_empty(void);
bool rb_rx_full(void);
uint32_t rb_rx_available(void);

/* Radio Event Log API */
void radio_log_init(void);
bool radio_log_write(uint8_t event_type, const char *msg);
bool radio_log_read(struct radio_log_msg_s *msg);
bool radio_log_empty(void);
uint32_t radio_log_available(void);

#endif /* __SRING_BUFFER_H */