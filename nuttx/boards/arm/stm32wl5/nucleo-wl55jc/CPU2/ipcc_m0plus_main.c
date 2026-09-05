#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32wlxx_hal.h"
#include "radio.h"
#include "subghz.h"
#include "radio_board_if.h"
#include "uart_debug.h"

#include "config.h"
#include "radio_app.h"
#include "protocol.h"
#include "ax25.h"
#include "g3ruh.h"
#include "Sring_buffer.h"

const RadioConfig_t SatelliteProfile = {
    .txFrequency = DEFAULT_TX_FREQUENCY_HZ,
    .rxFrequency = DEFAULT_RX_FREQUENCY_HZ,
    .sourceCallsign = "NEPSAT",
    .sourceSSID = 1,
    .destCallsign = "GROUND",
    .destSSID = 0,
    .isSatelliteMode = true
};

#define BURST_PACKET_COUNT  100
#define BEACON_INTERVAL_MS  90000  /* 90 Seconds between HK Beacons */
#define TX_TIMEOUT_MS       1000

typedef enum {
    STATE_IDLE_RX = 0,
    STATE_PROCESS_COMMAND,
    STATE_TRANSMIT_BURST,
} AppState_t;

volatile AppState_t app_state = STATE_IDLE_RX;
volatile uint8_t tx_busy = 0;
volatile uint8_t rx_frame_buffer[AX25_MAX_FRAME_SIZE];
volatile uint16_t rx_frame_size = 0;
volatile uint8_t rx_done_flag = 0;
volatile uint8_t rx_timeout_flag = 0;
volatile uint8_t rx_error_flag = 0;

static uint8_t last_tx_scrambled[AX25_MAX_FRAME_SIZE];
static uint16_t last_tx_len = 0;
static uint32_t last_beacon_tick_ms = 0;
static uint32_t last_burst_finish_time = 0;

/* Interrupt Status Callbacks */
void OnTxDone(void) { tx_busy = 0; }
void OnTxTimeout(void) { uart2_puts("TX TIMEOUT ERROR\r\n"); tx_busy = 0; }
void OnRxTimeout(void) { rx_timeout_flag = 1; uart2_puts("[SAT RX TIMEOUT]\r\n"); }
void OnRxError(void) { rx_error_flag = 1; uart2_puts("[SAT RX ERROR]\r\n"); }

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    (void)rssi;
    (void)snr;
    uart2_printf("[SAT RX DONE: %u bytes, rssi=%d]\r\n", size, (int)rssi);
    uint16_t copy_len = (size < AX25_MAX_FRAME_SIZE) ? size : AX25_MAX_FRAME_SIZE;
    memcpy((void *)rx_frame_buffer, payload, copy_len);
    rx_frame_size = copy_len;
    rx_done_flag = 1;
}

extern SUBGHZ_HandleTypeDef hsubghz;
void SUBGHZ_Radio_IRQHandler(void) { HAL_SUBGHZ_IRQHandler(&hsubghz); }

static void Print_Hex_Bytes(const uint8_t *buf, uint16_t len) {
    char line[56]; uint16_t pos = 0; uint16_t on_this_line = 0;
    for (uint16_t i = 0; i < len; i++) {
        pos += snprintf(&line[pos], sizeof(line) - pos, "%02X ", buf[i]);
        on_this_line++;
        if (on_this_line == 16 || i == (uint16_t)(len - 1)) {
            uart2_puts(line); uart2_puts("\r\n"); pos = 0; on_this_line = 0;
        }
    }
}

static void Print_Payload_ASCII(const uint8_t *buf, uint16_t start, uint16_t end) {
    if (end <= start) {
        uart2_puts(" Payload (text) : (none)\r\n");
        return;
    }
    char line[96];
    uint16_t n = (uint16_t)(end - start);
    uint16_t max_chars = (uint16_t)(sizeof(line) - 24);
    if (n > max_chars) n = max_chars;
    uint16_t pos = 0;
    pos += (uint16_t)snprintf(&line[pos], sizeof(line) - pos, " Payload (text): \"");
    for (uint16_t i = 0; i < n; i++) {
        uint8_t c = buf[start + i];
        line[pos++] = (c >= 0x20 && c < 0x7F) ? (char)c : '.';
    }
    pos += (uint16_t)snprintf(&line[pos], sizeof(line) - pos, "\"\r\n");
    uart2_puts(line);
}

static bool Looks_Like_Stale_TX_Buffer(const uint8_t *buf, uint16_t len) {
    if (last_tx_len == 0) return false;
    uint16_t compare_len = (len < last_tx_len) ? len : last_tx_len;
    if (compare_len < 8) return false;
    return (memcmp(buf, last_tx_scrambled, compare_len) == 0);
}

static void Print_AX25_Fields(const char *label, const uint8_t *buf, uint16_t len) {
    char line[112];
    snprintf(line, sizeof(line), " --- %s field breakdown (%u bytes) ---\r\n", label, len);
    uart2_puts(line);
    if (len < 20) {
        uart2_puts(" (too short for a full AX.25 header+FCS - skipping field breakdown)\r\n");
        return;
    }
    snprintf(line, sizeof(line), " Start Flag : %02X %s\r\n", buf[0], (buf[0] == AX25_FLAG) ? "(OK)" : "(MISMATCH)");
    uart2_puts(line);
    char dest_cs[7], src_cs[7];
    AX25_DecodeAddress(&buf[1], dest_cs);
    snprintf(line, sizeof(line), " Dest       : %02X %02X %02X %02X %02X %02X %02X -> \"%s\"\r\n", buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7], dest_cs);
    uart2_puts(line);
    AX25_DecodeAddress(&buf[8], src_cs);
    snprintf(line, sizeof(line), " Src        : %02X %02X %02X %02X %02X %02X %02X -> \"%s\"\r\n", buf[8],buf[9],buf[10],buf[11],buf[12],buf[13],buf[14], src_cs);
    uart2_puts(line);
    snprintf(line, sizeof(line), " Control    : %02X\r\n", buf[15]); uart2_puts(line);
    snprintf(line, sizeof(line), " PID        : %02X\r\n", buf[16]); uart2_puts(line);

    uint16_t payload_start = 17; uint16_t payload_end = (uint16_t)(len - 3);
    if (payload_end > payload_start) {
        uint16_t plen = (uint16_t)(payload_end - payload_start);
        snprintf(line, sizeof(line), " Payload    : %u bytes\r\n", plen); uart2_puts(line);
        Print_Hex_Bytes(&buf[payload_start], plen);
        Print_Payload_ASCII(buf, payload_start, payload_end);
    } else {
        uart2_puts(" Payload    : (none)\r\n");
    }
    if (len >= 3) {
        snprintf(line, sizeof(line), " FCS/CRC    : %02X %02X\r\n", buf[len - 3], buf[len - 2]);
        uart2_puts(line);
    }
    snprintf(line, sizeof(line), " End Flag   : %02X %s\r\n", buf[len - 1], (buf[len - 1] == AX25_FLAG) ? "(OK)" : "(MISMATCH)");
    uart2_puts(line);
}

extern void SysTick_Init_CPU2(uint32_t sys_freq_hz);
extern void CPU2_Delay_Ms(uint32_t ms);
extern volatile uint32_t g_system_tick_ms;

static uint32_t Get_Time_Ms(void) {
    return g_system_tick_ms;
}

/* All TX goes through RadioApp_Send() */
static bool Radio_Send_And_Wait(uint8_t *buffer, uint16_t size, uint32_t timeout_ms) {
    tx_busy = 1;
    RadioApp_Send(buffer, size);

    uint32_t start_ms = Get_Time_Ms();
    while (tx_busy != 0) {
        if ((Get_Time_Ms() - start_ms) > timeout_ms) {
            uart2_puts("ERROR: TX timeout\r\n");
            Radio.Standby();
            tx_busy = 0;
            return false;
        }
        CPU2_Delay_Ms(1);
    }
    return true;
}

/* 100x Telemetry Burst Stream Routine */
static void Transmit_Burst_100(void) {
    uint8_t tx_data[90] = "Namaste Everyone! From Antarikchya Nepal. S2S-2 CubeSat Beacon Communication Test";

    uint8_t frame_buffer[AX25_MAX_FRAME_SIZE];
    uint16_t frame_size = Protocol_CreatePacket(frame_buffer, tx_data, (uint16_t)strlen((const char *)tx_data), &SatelliteProfile);
    if (frame_size == 0) return;

    memcpy(last_tx_scrambled, frame_buffer, frame_size);
    last_tx_len = frame_size;

    char size_msg[80];
    snprintf(size_msg, sizeof(size_msg), "BURST START: %d packets, %d bytes each, scrambled\r\n", BURST_PACKET_COUNT, frame_size);
    uart2_puts(size_msg);

    uart2_puts(" TX plaintext payload : \"");
    uart2_puts((const char *)tx_data);
    uart2_puts("\"\r\n");
    Print_AX25_Fields("TX (on-air / scrambled)", frame_buffer, frame_size);

    uint32_t sent_ok = 0, sent_timeout = 0;
    for (int i = 0; i < BURST_PACKET_COUNT; i++) {
        bool ok = Radio_Send_And_Wait(frame_buffer, frame_size, 3000);
        if (ok) sent_ok++; else sent_timeout++;
        char pkt_msg[48];
        snprintf(pkt_msg, sizeof(pkt_msg), " TX packet %3d/%d : %s\r\n", i + 1, BURST_PACKET_COUNT, ok ? "sent OK" : "TIMEOUT");
        uart2_puts(pkt_msg);
        CPU2_Delay_Ms(25);
    }

    char done_msg[96];
    snprintf(done_msg, sizeof(done_msg), "BURST COMPLETE (%lu/%d sent cleanly, %lu timed out)\r\n", (unsigned long)sent_ok, BURST_PACKET_COUNT, (unsigned long)sent_timeout);
    uart2_puts(done_msg);
}

/* ============================================================
 * CW / MORSE CODE BEACON ("S2S2BEA")
 * ============================================================ */
#define MORSE_DOT_MS     80   /* Standard ~15 WPM dot timing */
#define MORSE_DASH_MS   (MORSE_DOT_MS * 3)

static void CW_Tone_Init(void) {
    /* Put radio in STDBY_XOSC: 32 MHz TCXO runs continuously without shutting down */
    SUBGRF_SetStandby(STDBY_XOSC);
    CPU2_Delay_Ms(10); /* Ensure TCXO is 100% stable */
    SUBGRF_SetPacketType(PACKET_TYPE_GFSK);
    SUBGRF_SetRfFrequency(SatelliteProfile.txFrequency);
    SUBGRF_SetPaConfig(0x04, 0x07, 0x00, 0x01);
    uint8_t pa_sw = SUBGRF_SetRfTxPower(RADIO_TX_POWER_DBM);
    SUBGRF_WriteRegister(REG_DRV_CTRL, 0x7 << 1);
    (void)pa_sw;
}

static void CW_Tone_On(void) {
    /* Explicitly arm PA configuration, bias, and output power on every key-on */
    SUBGRF_SetPaConfig(0x04, 0x07, 0x00, 0x01);
    SUBGRF_SetTxParams(RFO_HP, RADIO_TX_POWER_DBM, RADIO_RAMP_40_US);
    SUBGRF_WriteRegister(REG_DRV_CTRL, 0x7 << 1);
    SUBGRF_SetSwitch(RFO_HP, RFSWITCH_TX);
    SUBGRF_SetTxContinuousWave();
}

static void CW_Tone_Off(void) {
    /* Return to STDBY_XOSC: cuts RF carrier immediately while preserving 32MHz TCXO */
    SUBGRF_SetStandby(STDBY_XOSC);
}

/* Full Morse Code Table: Letters A-Z, Numbers 0-9, Decimal Point, Minus */
static const char* Get_Morse_Pattern(char c) {
    if (c >= 'a' && c <= 'z') c -= 32; /* Convert to uppercase */

    switch (c) {
        case 'A': return ".-";
        case 'B': return "-...";
        case 'C': return "-.-.";
        case 'D': return "-..";
        case 'E': return ".";
        case 'F': return "..-.";
        case 'G': return "--.";
        case 'H': return "....";
        case 'I': return "..";
        case 'J': return ".---";
        case 'K': return "-.-";
        case 'L': return ".-..";
        case 'M': return "--";
        case 'N': return "-.";
        case 'O': return "---";
        case 'P': return ".--.";
        case 'Q': return "--.-";
        case 'R': return ".-.";
        case 'S': return "...";
        case 'T': return "-";
        case 'U': return "..-";
        case 'V': return "...-";
        case 'W': return ".--";
        case 'X': return "-..-";
        case 'Y': return "-.--";
        case 'Z': return "--..";
        case '0': return "-----";
        case '1': return ".----";
        case '2': return "..---";
        case '3': return "...--";
        case '4': return "....-";
        case '5': return ".....";
        case '6': return "-....";
        case '7': return "--...";
        case '8': return "---..";
        case '9': return "----.";
        case '.': return ".-.-.-"; /* Decimal point */
        case '-': return "-....-"; /* Minus / dash */
        default:  return "";
    }
}

static void Transmit_Morse_String(const char *str) {
    uart2_printf("CW TX (Morse): \"%s\"\r\n", str);

    CW_Tone_Init();

    /* Brief key to verify hardware RF state */
    CW_Tone_On();
    RadioPhyStatus_t stat = SUBGRF_GetStatus();
    RadioError_t errs = SUBGRF_GetDeviceErrors();
    CW_Tone_Off();

    char diag[96];
    snprintf(diag, sizeof(diag),
             "[M0+ RF] CW RF State: Mode=0x%02X (%s), CmdStat=0x%02X, Errs=0x%04X, 435.000MHz",
             stat.Fields.ChipMode,
             (stat.Fields.ChipMode == 6) ? "TX OK" : (stat.Fields.ChipMode == 3) ? "XOSC" : "OTHER",
             stat.Fields.CmdStatus,
             errs.Value);
    radio_log_write(RADIO_EVT_CW, diag);
    uart2_printf("%s\r\n", diag);

    for (size_t i = 0; i < strlen(str); i++) {
        char c = str[i];
        if (c == ' ') {
            CPU2_Delay_Ms(MORSE_DOT_MS * 4); /* Word space */
            continue;
        }

        const char *pattern = Get_Morse_Pattern(c);
        for (size_t j = 0; j < strlen(pattern); j++) {
            CW_Tone_On();
            if (pattern[j] == '.') {
                CPU2_Delay_Ms(MORSE_DOT_MS);
            } else if (pattern[j] == '-') {
                CPU2_Delay_Ms(MORSE_DASH_MS);
            }
            CW_Tone_Off();
            CPU2_Delay_Ms(MORSE_DOT_MS); /* Space between dots/dashes */
        }
        CPU2_Delay_Ms(MORSE_DOT_MS * 2); /* Space between letters */
    }
    CW_Tone_Off();
    SUBGRF_SetSwitch(RFO_HP, RFSWITCH_RX);
}

#define CADENCE_90S_MS         90000UL /* 90 Seconds Cadence */

static struct tx_packet_s s_cached_b1;
static struct tx_packet_s s_cached_b2;
static bool s_has_b1 = false;
static bool s_has_b2 = false;
static uint8_t s_next_beacon_type = PKT_TYPE_B1;
static uint32_t s_cadence_timer_ms = 0;

/* Helper to drain ring buffer 1 and handle ACK/NACK immediately */
static void Drain_Ring_Buffer_1(void) {
    if (ipcc_m0_received(IPCC_CH_HANDSHAKE)) {
        ipcc_m0_clear(IPCC_CH_HANDSHAKE);
        ipcc_m0_send(IPCC_CH_HANDSHAKE); /* Acknowledge back to M4 */
        radio_log_write(RADIO_EVT_INFO, "[M0+ RF] Handshake Synchronized with M4! Radio Link Ready.");
        uart2_puts("[M0+ IPC] Handshake acknowledged with M4!\r\n");
    }

    struct tx_packet_s pkt;
    while (rb_tx_read(&pkt)) {
        ipcc_m0_clear(IPCC_CH_BEACON);
        if (pkt.type == PKT_TYPE_B1) {
            memcpy(&s_cached_b1, &pkt, sizeof(pkt));
            s_has_b1 = true;
            char lbuf[88];
            snprintf(lbuf, sizeof(lbuf), "[M0+ RF] Cached latest Beacon 1 (HK1 %u Bytes)", pkt.len);
            radio_log_write(RADIO_EVT_INFO, lbuf);
            uart2_printf("[M0+ IPC] Cached latest Beacon 1 (HK1 %u Bytes)\r\n", pkt.len);
        } else if (pkt.type == PKT_TYPE_B2) {
            memcpy(&s_cached_b2, &pkt, sizeof(pkt));
            s_has_b2 = true;
            char lbuf[88];
            snprintf(lbuf, sizeof(lbuf), "[M0+ RF] Cached latest Beacon 2 (HK2 %u Bytes)", pkt.len);
            radio_log_write(RADIO_EVT_INFO, lbuf);
            uart2_printf("[M0+ IPC] Cached latest Beacon 2 (HK2 %u Bytes)\r\n", pkt.len);
        } else if (pkt.type == PKT_TYPE_ACK || pkt.type == PKT_TYPE_NACK) {
            /* Transmit command responses immediately */
            char lbuf[88];
            snprintf(lbuf, sizeof(lbuf), "[M0+ RF] >>> DOWNLINKING %s via AX.25 (435.000MHz) <<<",
                     (pkt.type == PKT_TYPE_ACK) ? "ACK" : "NACK");
            radio_log_write((pkt.type == PKT_TYPE_ACK) ? RADIO_EVT_TX_ACK : RADIO_EVT_TX_NACK, lbuf);
            uart2_printf("\r\n>>> IMMEDIATE COMMAND %s TRANSMIT via AX.25 + G3RUH <<<\r\n",
                         (pkt.type == PKT_TYPE_ACK) ? "ACK" : "NACK");
            uint8_t frame[AX25_MAX_FRAME_SIZE];
            uint16_t flen = Protocol_CreatePacket(frame, pkt.data, pkt.len, &SatelliteProfile);
            if (flen > 0) {
                Radio_Send_And_Wait(frame, flen, 3000);
                rx_done_flag = 0;
                RadioApp_StartRx();
            }
        }
    }
}

void CPU2_Poll_IPC(void) {
    if (ipcc_m0_received(IPCC_CH_HANDSHAKE)) {
        ipcc_m0_clear(IPCC_CH_HANDSHAKE);
        ipcc_m0_send(IPCC_CH_HANDSHAKE); /* Acknowledge back to M4 */
        radio_log_write(RADIO_EVT_INFO, "[M0+ RF] Handshake Synchronized with M4! Radio Link Ready.");
        uart2_puts("[M0+ IPC] Handshake acknowledged with M4!\r\n");
    }
    if (ipcc_m0_received(IPCC_CH_BEACON)) {
        Drain_Ring_Buffer_1();
    }
}

/* Transmits alternating Beacon 1 (HK1: 34B) or Beacon 2 (HK2: 38B) after 90s CW */
static void Transmit_Scheduled_Beacon(void) {
    Drain_Ring_Buffer_1();

    struct tx_packet_s *pkt_to_send = NULL;
    char log_msg[88];

    if (s_next_beacon_type == PKT_TYPE_B1) {
        if (!s_has_b1) {
            s_cached_b1.type = PKT_TYPE_B1;
            s_cached_b1.len  = 34;
            memset(s_cached_b1.data, 0, 34);
            s_cached_b1.data[32] = 0xAA;
            s_cached_b1.data[33] = 0x55;
            s_has_b1 = true;
        }
        pkt_to_send = &s_cached_b1;
        snprintf(log_msg, sizeof(log_msg),
                 "[M0+ RF] >>> TRANSMITTING BEACON 1 (HK1: %uB) via AX.25 G3RUH (435.000MHz) <<<",
                 pkt_to_send->len);
        radio_log_write(RADIO_EVT_B1, log_msg);
        s_next_beacon_type = PKT_TYPE_B2; /* Alternate to B2 next time */
    } else {
        if (!s_has_b2) {
            s_cached_b2.type = PKT_TYPE_B2;
            s_cached_b2.len  = 38;
            memset(s_cached_b2.data, 0, 38);
            s_cached_b2.data[36] = 0xBB;
            s_cached_b2.data[37] = 0x66;
            s_has_b2 = true;
        }
        pkt_to_send = &s_cached_b2;
        snprintf(log_msg, sizeof(log_msg),
                 "[M0+ RF] >>> TRANSMITTING BEACON 2 (HK2: %uB) via AX.25 G3RUH (435.000MHz) <<<",
                 pkt_to_send->len);
        radio_log_write(RADIO_EVT_B2, log_msg);
        s_next_beacon_type = PKT_TYPE_B1; /* Alternate to B1 next time */
    }

    uart2_printf("\r\n=======================================================\r\n");
    uart2_printf("%s\r\n", log_msg);
    uart2_printf("=======================================================\r\n");

    uint8_t frame_buffer[AX25_MAX_FRAME_SIZE];
    uint16_t frame_size = Protocol_CreatePacket(frame_buffer, pkt_to_send->data, pkt_to_send->len, &SatelliteProfile);
    if (frame_size > 0) {
        memcpy(last_tx_scrambled, frame_buffer, frame_size);
        last_tx_len = frame_size;
        Radio_Send_And_Wait(frame_buffer, frame_size, 3000);
        char bdone_msg[88];
        snprintf(bdone_msg, sizeof(bdone_msg),
                 "[M0+ RF] %s Transmitted OK (1x %uB on 435.000MHz)",
                 (pkt_to_send->type == PKT_TYPE_B1) ? "Beacon 1" : "Beacon 2",
                 frame_size);
        radio_log_write(RADIO_EVT_INFO, bdone_msg);
    }

    rx_done_flag = 0;
    rx_timeout_flag = 0;
    rx_error_flag = 0;
    RadioApp_StartRx();
}

static uint8_t s_prev_rx_buf[AX25_MAX_FRAME_SIZE];
static uint16_t s_prev_rx_len = 0;

#define IPCC_C2MR \
  (*(volatile uint32_t *)(0x58000C14UL))

void IPCC_C2_RX_IRQHandler(void) {
    if (ipcc_m0_received(IPCC_CH_HANDSHAKE)) {
        ipcc_m0_clear(IPCC_CH_HANDSHAKE);
        ipcc_m0_send(IPCC_CH_HANDSHAKE);
    }
}

int main(void) {
    SCB->VTOR = 0x08032000;
    SystemInit();
    HAL_Init();
    SysTick_Init_CPU2(HAL_RCC_GetHCLK2Freq());

    /* Enable IPCC, SUBGHZSPI, and SRAM2 peripheral bus clocks in both CPU1 and CPU2 domains */
    (*(volatile uint32_t *)0x58000150UL) |= (1UL << 0) | (1UL << 25);
    (*(volatile uint32_t *)0x58000050UL) |= (1UL << 0) | (1UL << 25);

    /* Enable GPIOA, GPIOB, GPIOC peripheral bus clocks in both domains */
    (*(volatile uint32_t *)0x5800004CUL) |= 0x87;
    (*(volatile uint32_t *)0x5800014CUL) |= 0x87;

    /* Mask IPCC interrupts in CPU2 domain so we use clean polling without IRQ traps */
    IPCC_C2MR = 0xFFFFFFFF;

    uart2_init();
    uart2_puts("HAL_Init done\r\n");
    uart2_puts("\r\n=== STM32WL55 AX.25 + G3RUH [M4/M0+ DUAL-RING DUAL-CORE IPC ACTIVE] ===\r\n");

    /* IPCC Initial Handshake with Cortex-M4 (CPU1) */
    uart2_puts("[M0+] Waiting for M4 Handshake via IPCC Channel 3 (up to 30s)...\r\n");
    uint32_t hs_start = Get_Time_Ms();
    while ((Get_Time_Ms() - hs_start) < 30000) {
        if (ipcc_m0_received(IPCC_CH_HANDSHAKE)) {
            ipcc_m0_clear(IPCC_CH_HANDSHAKE);
            ipcc_m0_send(IPCC_CH_HANDSHAKE); /* Acknowledge back to M4 */
            radio_log_write(RADIO_EVT_INFO, "[M0+ RF] Handshake Synchronized with M4! Radio Link Active.");
            uart2_puts("[M0+] Handshake ACKNOWLEDGED with M4! Dual-core IPC link active.\r\n");
            break;
        }
        CPU2_Delay_Ms(10);
    }

    RBI_Init();
    uart2_puts("RF FRONT END OK\r\n");

    MX_SUBGHZ_Init();
    HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
    __enable_irq();

    RadioEvents_t events = {
        .TxDone = OnTxDone, .TxTimeout = OnTxTimeout,
        .RxDone = OnRxDone, .RxTimeout = OnRxTimeout, .RxError = OnRxError
    };

    RadioApp_Init(&SatelliteProfile, &events);
    RadioApp_StartRx();

    uart2_puts("CADENCE: 90s CW MORSE -> B1 -> 90s CW -> B2...\r\n");

    s_next_beacon_type = PKT_TYPE_B1;
    app_state = STATE_IDLE_RX;
    s_prev_rx_len = 0;

    while (1) {
        /* =================================================================== */
        /* 1. CONTINUOUS CW MORSE TRANSMISSION FOR 90 SECONDS                  */
        /* =================================================================== */
        float batt_volt = 3.80f;
        float sat_temp  = 20.0f;
        live_telem_get_float(&batt_volt, &sat_temp);

        int v_int  = (int)batt_volt;
        int v_frac = (int)((batt_volt - (float)v_int) * 100.0f);
        if (v_frac < 0) v_frac = -v_frac;

        int t_int  = (int)sat_temp;
        int t_frac = (int)((sat_temp - (float)t_int) * 10.0f);
        if (t_frac < 0) t_frac = -t_frac;

        char morse_buf[48];
        snprintf(morse_buf, sizeof(morse_buf), "9NS2S2 V%d.%02d T%d.%d",
                 v_int, v_frac, t_int, t_frac);

        char log_buf[88];
        snprintf(log_buf, sizeof(log_buf),
                 "[M0+ RF] Starting 90s Continuous CW: \"%s\" (435.000MHz +22dBm, Next: %s)",
                 morse_buf, (s_next_beacon_type == PKT_TYPE_B1) ? "B1" : "B2");
        radio_log_write(RADIO_EVT_CW, log_buf);
        uart2_printf("\r\n%s\r\n", log_buf);

        uint32_t cw_start_time = Get_Time_Ms();

        while ((Get_Time_Ms() - cw_start_time) < CADENCE_90S_MS) {
            Drain_Ring_Buffer_1();

            /* Refresh telemetry string if M4 posted new values */
            live_telem_get_float(&batt_volt, &sat_temp);
            v_int  = (int)batt_volt;
            v_frac = (int)((batt_volt - (float)v_int) * 100.0f);
            if (v_frac < 0) v_frac = -v_frac;
            t_int  = (int)sat_temp;
            t_frac = (int)((sat_temp - (float)t_int) * 10.0f);
            if (t_frac < 0) t_frac = -t_frac;
            snprintf(morse_buf, sizeof(morse_buf), "9NS2S2 V%d.%02d T%d.%d",
                     v_int, v_frac, t_int, t_frac);

            uint32_t elapsed_s = (Get_Time_Ms() - cw_start_time) / 1000;
            if (elapsed_s > 90) elapsed_s = 90;
            snprintf(log_buf, sizeof(log_buf),
                     "[M0+ RF] CW Morse Active: \"%s\" (%lus/90s elapsed)",
                     morse_buf, (unsigned long)elapsed_s);
            radio_log_write(RADIO_EVT_CW, log_buf);

            /* Key the CW Morse signal */
            Transmit_Morse_String(morse_buf);

            /* Check remaining time in this 90s window */
            uint32_t now = Get_Time_Ms();
            if ((now - cw_start_time) >= CADENCE_90S_MS) {
                break;
            }

            uint32_t rem_ms = CADENCE_90S_MS - (now - cw_start_time);
            uint32_t listen_ms = (rem_ms > 2000) ? 2000 : rem_ms;

            /* Brief listening window for ground station telecommands */
            rx_done_flag = 0;
            rx_timeout_flag = 0;
            rx_error_flag = 0;
            RadioApp_StartRx();

            uint32_t rx_start = Get_Time_Ms();
            while ((Get_Time_Ms() - rx_start) < listen_ms) {
                if (rx_done_flag) {
                    rx_done_flag = 0;
                    app_state = STATE_PROCESS_COMMAND;
                    break;
                }
                CPU2_Delay_Ms(5);
            }

            /* If telecommand received during listen window, process and respond */
            if (app_state == STATE_PROCESS_COMMAND) {
                uint16_t raw_len = (rx_frame_size > AX25_MAX_FRAME_SIZE) ? AX25_MAX_FRAME_SIZE : rx_frame_size;
                uint8_t raw_copy[AX25_MAX_FRAME_SIZE];
                memcpy(raw_copy, (const void *)rx_frame_buffer, raw_len);

                uint8_t stream_buffer[AX25_MAX_FRAME_SIZE * 2];
                uint16_t stream_len = 0;

                if (s_prev_rx_len > 0) {
                    memcpy(&stream_buffer[0], s_prev_rx_buf, s_prev_rx_len);
                    stream_len += s_prev_rx_len;
                }
                memcpy(&stream_buffer[stream_len], raw_copy, raw_len);
                stream_len += raw_len;
                memcpy(s_prev_rx_buf, raw_copy, raw_len);
                s_prev_rx_len = raw_len;

                uint8_t decoded_frame[AX25_MAX_FRAME_SIZE];
                uint16_t decoded_len = 0;

                if (Protocol_ExtractFrame(stream_buffer, stream_len, decoded_frame, sizeof(decoded_frame), &decoded_len)) {
                    s_prev_rx_len = 0;
                    bool crc1_ok = AX25_VerifyCRC_Method1(decoded_frame, decoded_len);
                    uint16_t crc2_computed = 0, crc2_recv_le = 0, crc2_recv_be = 0;
                    bool crc2_ok = AX25_VerifyCRC_Method2(decoded_frame, decoded_len, &crc2_computed, &crc2_recv_le, &crc2_recv_be);

                    if ((crc1_ok || crc2_ok) && decoded_len >= 17) {
                        char dest_cs[7];
                        AX25_DecodeAddress(&decoded_frame[1], dest_cs);
                        if (strcmp(dest_cs, SatelliteProfile.sourceCallsign) == 0) {
                            uint16_t payload_start = 17;
                            uint16_t payload_end = decoded_len - 3;
                            if (payload_end > payload_start) {
                                uint16_t payload_len = payload_end - payload_start;
                                if (payload_len == 13 || decoded_frame[payload_start] == 0x53) {
                                    struct rx_command_s cmd;
                                    cmd.len = 13;
                                    memcpy(cmd.cmd, &decoded_frame[payload_start], 13);

                                    char rx_log[88];
                                    snprintf(rx_log, sizeof(rx_log),
                                             "[M0+ RF] <<< GROUND TELECOMMAND RECEIVED (13B, 437.375MHz, Opcode: 0x%02X) >>>",
                                             cmd.cmd[1]);
                                    radio_log_write(RADIO_EVT_RX_CMD, rx_log);

                                    rb_rx_write(&cmd);
                                    ipcc_m0_send(IPCC_CH_COMMAND);

                                    /* Wait up to 2 seconds for M4 to check camera and respond */
                                    uint32_t wait_start = Get_Time_Ms();
                                    while ((Get_Time_Ms() - wait_start) < 2000) {
                                        struct tx_packet_s resp;
                                        if (rb_tx_read(&resp)) {
                                            ipcc_m0_clear(IPCC_CH_BEACON);
                                            if (resp.type == PKT_TYPE_ACK) {
                                                snprintf(rx_log, sizeof(rx_log),
                                                         "[M0+ RF] >>> DOWNLINKING COMMAND ACK (0xAA) via AX.25 (435.000MHz) <<<");
                                                radio_log_write(RADIO_EVT_TX_ACK, rx_log);
                                                uint8_t ack_frame[AX25_MAX_FRAME_SIZE];
                                                uint16_t ack_len = Protocol_CreatePacket(ack_frame, resp.data, resp.len, &SatelliteProfile);
                                                Radio_Send_And_Wait(ack_frame, ack_len, 3000);
                                                break;
                                            } else if (resp.type == PKT_TYPE_NACK) {
                                                snprintf(rx_log, sizeof(rx_log),
                                                         "[M0+ RF] >>> DOWNLINKING COMMAND NACK (0x55) via AX.25 (435.000MHz) <<<");
                                                radio_log_write(RADIO_EVT_TX_NACK, rx_log);
                                                uint8_t nack_frame[AX25_MAX_FRAME_SIZE];
                                                uint16_t nack_len = Protocol_CreatePacket(nack_frame, resp.data, resp.len, &SatelliteProfile);
                                                Radio_Send_And_Wait(nack_frame, nack_len, 3000);
                                                break;
                                            }
                                        }
                                        CPU2_Delay_Ms(10);
                                    }
                                }
                            }
                        }
                    }
                }
                app_state = STATE_IDLE_RX;
            }
        }

        /* =================================================================== */
        /* 2. 90s CW COMPLETED -> TRANSMIT SCHEDULED BEACON (B1 OR B2)        */
        /* =================================================================== */
        Transmit_Scheduled_Beacon();
    }

    return 0;
}