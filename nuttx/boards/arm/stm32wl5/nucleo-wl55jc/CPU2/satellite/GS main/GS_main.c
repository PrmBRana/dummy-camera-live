/*
 * STM32WL55 Ground Station
 *
 * AX.25 + G3RUH + GFSK
 *
 * TX:
 * 437.375 MHz
 *
 * RX:
 * 435.000 MHz
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32wlxx_hal.h"

#include "radio.h"
#include "subghz.h"
#include "radio_driver.h"
#include "radio_board_if.h"

#include "uart_debug.h"

#include "config.h"
#include "radio_app.h"
#include "protocol.h"

#include "ax25.h"
#include "g3ruh.h"



/*
=========================================================
GROUND STATION RADIO PROFILE
=========================================================
*/

const RadioConfig_t GroundStationProfile =
{
    /* Ground station transmit frequency, satellite uplink receiver */
    .txFrequency = 437375000UL,

    /* Ground station receive frequency, satellite downlink transmitter */
    .rxFrequency = 435000000UL,

    /* AX25 address */
    .sourceCallsign = "GROUND",
    /* FIX: aligned to 0 to match the SSID the satellite side uses when
       addressing frames to "GROUND" (SatelliteProfile.destSSID = 0).
       AX25_DecodeAddress() doesn't compare SSID today so this wasn't
       breaking the link, but it was inconsistent data - both ends
       should agree on GROUND's SSID. */
    .sourceSSID = 0,

    /* Satellite address */
    .destCallsign = "NEPSAT",
    .destSSID = 0,

    /* Ground station mode */
    .isSatelliteMode = false
};



/*
=========================================================
SYSTEM CONFIGURATION
=========================================================
*/

#define CMD_LINE_MAX        32
#define RX_WATCHDOG_MS      5000
#define TX_TIMEOUT_MS       3000
#define RX_SESSION_MS       60000

#define RX_HEARTBEAT_MS     3000



/*
=========================================================
GLOBAL RADIO FLAGS
=========================================================
*/

volatile uint8_t tx_busy = 0;

volatile uint8_t rx_frame_buffer[AX25_MAX_FRAME_SIZE];
volatile uint16_t rx_frame_size = 0;

volatile uint8_t rx_done_flag = 0;
volatile uint8_t rx_timeout_flag = 0;
volatile uint8_t rx_error_flag = 0;



/*
=========================================================
RX EVENT COUNTERS
=========================================================
*/

static uint32_t stat_rx_timeouts = 0;
static uint32_t stat_rx_errors = 0;
static uint32_t stat_rx_crc_errors = 0;
static uint32_t stat_rx_header_errors = 0;

/* captured inside SUBGHZ_Radio_IRQHandler() BEFORE HAL_SUBGHZ_IRQHandler()
   clears the IRQ register, so OnRxError() can read a real value instead of
   the stale 0x0000 it got from calling SUBGRF_GetIrqStatus() too late. */
volatile uint16_t g_last_irq_status = 0;



/*
=========================================================
TX HISTORY BUFFER
=========================================================
*/

static uint8_t last_tx_frame[AX25_MAX_FRAME_SIZE];
static uint16_t last_tx_len = 0;



/*
=========================================================
RX SESSION STATS
=========================================================
*/

static uint32_t stat_packets_ok = 0;
static uint32_t stat_packets_bad_crc = 0;
static uint32_t stat_packets_misaddressed = 0;
static uint32_t stat_packets_stale_tx = 0;



/*
=========================================================
FORWARD DECLARATIONS
=========================================================
*/

static void Print_Hex_Bytes(const uint8_t *buf, uint16_t len);
static void Print_Payload_ASCII(const uint8_t *buf, uint16_t start, uint16_t end);
static bool Looks_Like_Stale_TX_Buffer(const uint8_t *buf, uint16_t len);
static void Print_AX25_Fields(const char *label, const uint8_t *buf, uint16_t len);
static void Process_Received_Frame(void);



/*
=========================================================
DEBUG PRINT HELPERS
=========================================================
*/

static void Print_Hex_Bytes(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uart2_printf("%02X ", buf[i]);

        if ((i + 1) % 16 == 0)
        {
            uart2_puts("\r\n");
        }
    }

    if (len % 16 != 0)
    {
        uart2_puts("\r\n");
    }
}



static void Print_Payload_ASCII(const uint8_t *buf, uint16_t start, uint16_t end)
{
    if (end <= start)
    {
        uart2_puts("Payload (text) : (none)\r\n");
        return;
    }

    char line[112];
    uint16_t n = (uint16_t)(end - start);
    uint16_t max_chars = (uint16_t)(sizeof(line) - 24);
    if (n > max_chars) n = max_chars;

    uint16_t pos = 0;
    pos += (uint16_t)snprintf(&line[pos], sizeof(line) - pos, "Payload (text): \"");
    for (uint16_t i = 0; i < n; i++)
    {
        uint8_t c = buf[start + i];
        line[pos++] = (c >= 0x20 && c < 0x7F) ? (char)c : '.';
    }
    pos += (uint16_t)snprintf(&line[pos], sizeof(line) - pos, "\"\r\n");
    uart2_puts(line);
}



static bool Looks_Like_Stale_TX_Buffer(const uint8_t *buf, uint16_t len)
{
    if (last_tx_len == 0)
    {
        return false;
    }

    uint16_t compare_len = (len < last_tx_len) ? len : last_tx_len;

    if (compare_len < 8)
    {
        return false;
    }

    return (memcmp(buf, last_tx_frame, compare_len) == 0);
}



static void Print_AX25_Fields(const char *label, const uint8_t *buf, uint16_t len)
{
    uart2_printf("\r\n--- %s FIELD BREAKDOWN (%u bytes) ---\r\n", label, len);

    if (len < 20)
    {
        uart2_puts("(too short for a full AX.25 header+FCS - skipping field breakdown)\r\n");
        return;
    }

    uart2_printf
    (
        "Start Flag : %02X %s\r\n",
        buf[0],
        (buf[0] == AX25_FLAG) ? "(OK, matches 0x7E)" : "(MISMATCH)"
    );

    char dest_cs[7];
    char src_cs[7];

    AX25_DecodeAddress(&buf[1], dest_cs);
    AX25_DecodeAddress(&buf[8], src_cs);

    uart2_printf
    (
        "Dest       : %02X %02X %02X %02X %02X %02X %02X -> \"%s\"\r\n",
        buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
        dest_cs
    );

    uart2_printf
    (
        "Src        : %02X %02X %02X %02X %02X %02X %02X -> \"%s\"\r\n",
        buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14],
        src_cs
    );

    uart2_printf("Control    : %02X\r\n", buf[15]);
    uart2_printf("PID        : %02X\r\n", buf[16]);

    uint16_t payload_start = 17;
    uint16_t payload_end = (uint16_t)(len - 3);

    if (payload_end > payload_start)
    {
        uint16_t plen = (uint16_t)(payload_end - payload_start);

        uart2_printf("Payload    : %u bytes\r\n", plen);

        Print_Hex_Bytes(&buf[payload_start], plen);
        Print_Payload_ASCII(buf, payload_start, payload_end);
    }
    else
    {
        uart2_puts("Payload    : (none)\r\n");
    }

    uart2_printf
    (
        "FCS/CRC    : %02X %02X (as sent: low-byte-first)\r\n",
        buf[len - 3],
        buf[len - 2]
    );

    uart2_printf
    (
        "End Flag   : %02X %s\r\n",
        buf[len - 1],
        (buf[len - 1] == AX25_FLAG) ? "(OK, matches 0x7E)" : "(MISMATCH)"
    );
}



/*
=========================================================
PROCESS A RECEIVED FRAME
=========================================================
*/
static void Process_Received_Frame(void)
{
    uint16_t raw_len = (rx_frame_size > AX25_MAX_FRAME_SIZE)
                        ? AX25_MAX_FRAME_SIZE
                        : rx_frame_size;

    uint8_t raw_copy[AX25_MAX_FRAME_SIZE];
    memcpy(raw_copy, (const void *)rx_frame_buffer, raw_len);

    uart2_printf("\r\nRAW FRAME FROM RADIO (%u bytes):\r\n", raw_len);
    Print_Hex_Bytes(raw_copy, raw_len);

    if (Looks_Like_Stale_TX_Buffer(raw_copy, raw_len))
    {
        stat_packets_stale_tx++;
        uart2_puts("*** WARNING: raw bytes match our own last TX buffer ***\r\n");
    }

    /* Descramble + NRZ-I decode + locate the flag-delimited frame inside
       the fixed-length buffer (see protocol.c Protocol_ExtractFrame). */
    uint8_t decoded[AX25_MAX_FRAME_SIZE];
    uint16_t decoded_len = 0;

    if (!Protocol_ExtractFrame(raw_copy, raw_len, decoded, sizeof(decoded), &decoded_len))
    {
        uart2_puts("*** no valid flag-delimited frame found in this buffer - discarding ***\r\n");
        return;
    }

    Print_AX25_Fields("DESCRAMBLED", decoded, decoded_len);

    bool crc1_ok = AX25_VerifyCRC_Method1(decoded, decoded_len) && (decoded_len >= 17);

    uint16_t crc2_computed = 0;
    uint16_t crc2_recv_le = 0;
    uint16_t crc2_recv_be = 0;

    bool crc2_ok = AX25_VerifyCRC_Method2(
        decoded, decoded_len, &crc2_computed, &crc2_recv_le, &crc2_recv_be);

    uart2_printf(
        "\r\nCRC method 1 (reversed X-25, magic 0xF0B8) : %s\r\n",
        crc1_ok ? "PASS" : "FAIL");

    uart2_printf(
        "CRC method 2 (direct CCITT-FALSE)         : %s (computed=0x%04X LE=0x%04X BE=0x%04X)\r\n",
        crc2_ok ? "PASS" : "FAIL", crc2_computed, crc2_recv_le, crc2_recv_be);

    if (!(crc1_ok || crc2_ok))
    {
        stat_packets_bad_crc++;
        uart2_puts("CRC CHECK FAILED (both methods) - DISCARDING FRAME\r\n");
        return;
    }

    if (decoded_len < 17)
    {
        stat_packets_bad_crc++;
        uart2_puts("FRAME TOO SHORT - DISCARDING\r\n");
        return;
    }

    char dest_cs[7];
    char src_cs[7];

    AX25_DecodeAddress(&decoded[1], dest_cs);
    AX25_DecodeAddress(&decoded[8], src_cs);

    uart2_printf("\r\nFRAME: DEST=%s SRC=%s\r\n", dest_cs, src_cs);

    if (strcmp(dest_cs, GroundStationProfile.sourceCallsign) != 0)
    {
        stat_packets_misaddressed++;
        uart2_puts("NOT ADDRESSED TO US - IGNORING\r\n");
        return;
    }

    stat_packets_ok++;

    uint16_t payload_start = 17;
    uint16_t payload_end = (uint16_t)(decoded_len - 3);

    if (payload_end > payload_start)
    {
        uint16_t plen = (uint16_t)(payload_end - payload_start);
        uart2_printf("\r\nPAYLOAD (%u bytes, ASCII where printable):\r\n", plen);

        for (uint16_t i = 0; i < plen; i++)
        {
            uint8_t c = decoded[payload_start + i];
            uart2_putc((c >= 32 && c <= 126) ? (char)c : '.');
        }
        uart2_puts("\r\n");
    }

    uart2_printf(
        "\r\nSESSION STATS: %lu ok, %lu crc-fail, %lu not-for-us, %lu stale-tx, %lu timeouts, %lu rx-errors (crc=%lu hdr=%lu)\r\n",
        (unsigned long)stat_packets_ok, (unsigned long)stat_packets_bad_crc,
        (unsigned long)stat_packets_misaddressed, (unsigned long)stat_packets_stale_tx,
        (unsigned long)stat_rx_timeouts, (unsigned long)stat_rx_errors,
        (unsigned long)stat_rx_crc_errors, (unsigned long)stat_rx_header_errors);
}

/*
=========================================================
RADIO CALLBACK FUNCTIONS
=========================================================
*/

void OnTxDone(void)
{
    uart2_puts("\r\nTX DONE\r\n");
    tx_busy = 0;
}



void OnTxTimeout(void)
{
    uart2_puts("\r\nTX TIMEOUT\r\n");
    tx_busy = 0;
}



void OnRxTimeout(void)
{
    stat_rx_timeouts++;
    uart2_puts("[RX TIMEOUT] no signal detected this cycle\r\n");
    rx_timeout_flag = 1;
}



void OnRxError(void)
{
    stat_rx_errors++;

    uint16_t irq_status = g_last_irq_status;

    if (irq_status & IRQ_CRC_ERROR)
    {
        stat_rx_crc_errors++;
    }

    if (irq_status & IRQ_HEADER_ERROR)
    {
        stat_rx_header_errors++;
    }

    int16_t inst_rssi = SUBGRF_GetRssiInst();

    uart2_printf
    (
        "[RX ERROR] IRQ=0x%04X RSSI=%d dBm %s%s%s%s\r\n",
        irq_status,
        inst_rssi,
        (irq_status & IRQ_CRC_ERROR)      ? "CRC_ERROR " : "",
        (irq_status & IRQ_HEADER_ERROR)   ? "HEADER_ERROR " : "",
        (irq_status & IRQ_RX_TX_TIMEOUT)  ? "TIMEOUT " : "",
        (irq_status & IRQ_SYNCWORD_VALID) ? "SYNCWORD_OK " : ""
    );

    rx_error_flag = 1;
}



void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    /* FIX: these must be uint8_t to match SUBGRF_GetRxBufferStatus()'s
       actual signature (uint8_t *payloadLength, uint8_t *rxStartBufferPointer).
       Passing int8_t* there was an incompatible-pointer-type warning and,
       strictly speaking, undefined behavior - it happened to work here
       only because both types are 1 byte wide on this target. */
    uint8_t chipPayloadLen = 0, chipStartPtr = 0;
    SUBGRF_GetRxBufferStatus(&chipPayloadLen, &chipStartPtr);
    uart2_printf("CHIP BUFSTATUS: len=%u startPtr=%u\r\n", chipPayloadLen, chipStartPtr);

    RadioPhyStatus_t st = SUBGRF_GetStatus();
    uart2_printf("CHIP STATUS: mode=%d cmd=%d raw=0x%02X | buf[0]=0x%02X buf[1]=0x%02X buf[2]=0x%02X\r\n",
                 st.Fields.ChipMode, st.Fields.CmdStatus, st.Value,
                 payload[0], payload[1], payload[2]);

    uint16_t copy_len = (size > AX25_MAX_FRAME_SIZE) ? AX25_MAX_FRAME_SIZE : size;

    memcpy((void *)rx_frame_buffer, payload, copy_len);

    rx_frame_size = copy_len;

    uart2_printf
    (
        "\r\n[RX EVENT] %u bytes, RSSI=%d dBm, SNR=%d dB\r\n",
        copy_len,
        rssi,
        snr
    );

    rx_done_flag = 1;
}



/*
=========================================================
STM32WL SUBGHZ INTERRUPT HANDLER
=========================================================
*/

extern SUBGHZ_HandleTypeDef hsubghz;

void SUBGHZ_Radio_IRQHandler(void)
{
    g_last_irq_status = SUBGRF_GetIrqStatus();
    HAL_SUBGHZ_IRQHandler(&hsubghz);
}



/*
=========================================================
SEND COMMAND TO SATELLITE

COMMAND FLOW:
UART COMMAND -> AX25 FRAME CREATE -> G3RUH SCRAMBLE (inside
Protocol_CreatePacket) -> GFSK TX

TX goes through RadioApp_Send() instead of manually calling
Radio.SetChannel()+Radio.Send(). RadioApp_Send() resets the
SUBGHZ buffer base address and sets the TX channel internally,
every call - identical to the satellite side's TX path. This
keeps both ends symmetric so neither side can silently drift
out of sync with what RadioApp_Init() originally configured.
=========================================================
*/

static void Send_Command(CommandOpcode_t cmd)
{
    uint8_t payload[1];
    payload[0] = (uint8_t)cmd;

    uint8_t frame[AX25_MAX_FRAME_SIZE];

    uint16_t frame_len = Protocol_CreatePacket
    (
        frame,
        payload,
        sizeof(payload),
        &GroundStationProfile
    );

    if (frame_len == 0)
    {
        uart2_puts("AX25 CREATE ERROR\r\n");
        return;
    }

    uart2_printf("\r\nAX25 FRAME LENGTH : %u bytes\r\n", frame_len);

    /* Save frame for stale-buffer debug check (already scrambled at this point) */
    memcpy(last_tx_frame, frame, frame_len);
    last_tx_len = frame_len;

    uart2_puts("\r\nTX ON AIR DATA (already scrambled by Protocol_CreatePacket):\r\n");

    Print_Hex_Bytes(frame, frame_len);

    Print_AX25_Fields("TX (SCRAMBLED)", frame, frame_len);

    uart2_printf("\r\nTX COMMAND 0x%02X\r\n", cmd);

    tx_busy = 1;

    RadioApp_Send(frame, frame_len);

    uint32_t start = HAL_GetTick();

    while (tx_busy)
    {
        if ((HAL_GetTick() - start) > TX_TIMEOUT_MS)
        {
            uart2_puts("TX TIMEOUT FORCE STOP\r\n");
            Radio.Standby();
            tx_busy = 0;
            break;
        }
    }

    uart2_puts("COMMAND TX COMPLETE\r\n");
}



/*
=========================================================
CASE INSENSITIVE STRING COMPARE
=========================================================
*/

static bool str_ieq(const char *a, const char *b)
{
    while (*a && *b)
    {
        char ca = *a;
        char cb = *b;

        if (ca >= 'a' && ca <= 'z') ca -= 32;
        if (cb >= 'a' && cb <= 'z') cb -= 32;

        if (ca != cb)
        {
            return false;
        }

        a++;
        b++;
    }

    return (*a == 0 && *b == 0);
}



/*
=========================================================
UART COMMAND LINE READER
=========================================================
*/

static bool USART_TryReadLine(char *out, uint8_t max)
{
    static char buffer[CMD_LINE_MAX];
    static uint8_t index = 0;
    uint8_t c;

    if (!uart2_try_getc(&c))
    {
        return false;
    }

    uart2_putc(c);

    if (c == '\r' || c == '\n')
    {
        if (index == 0)
        {
            return false;
        }

        buffer[index] = 0;

        strncpy(out, buffer, max - 1);
        out[max - 1] = 0;

        index = 0;

        uart2_puts("\r\n");

        return true;
    }

    if (index < CMD_LINE_MAX - 1)
    {
        buffer[index++] = c;
    }

    return false;
}



/*
=========================================================
MAIN FUNCTION
=========================================================
*/

int main(void)
{
    /* MCU INITIALIZATION */

    SystemInit();
    HAL_Init();
    uart2_init();

    uart2_puts("\r\n");
    uart2_puts("================================\r\n");
    uart2_puts(" STM32WL55 GROUND STATION\r\n");
    uart2_puts(" AX25 + G3RUH + GFSK\r\n");
    uart2_puts(" [rev3: SSID aligned + buf status type fix]\r\n");
    uart2_puts(" UPLINK   : 437.375 MHz\r\n");
    uart2_puts(" DOWNLINK : 435.000 MHz\r\n");
    uart2_puts("================================\r\n");

    /* RADIO HARDWARE INIT */

    RBI_Init();

    MX_SUBGHZ_Init();

    /* ENABLE SUBGHZ INTERRUPT */

    HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);

    __enable_irq();

    /* RADIO CALLBACK TABLE */

    RadioEvents_t events =
    {
        .TxDone = OnTxDone,
        .TxTimeout = OnTxTimeout,
        .RxDone = OnRxDone,
        .RxTimeout = OnRxTimeout,
        .RxError = OnRxError
    };

    /* RADIO APPLICATION INIT */

    RadioApp_Init(&GroundStationProfile, &events);

    uart2_puts("\r\nRADIO READY\r\n");
    uart2_puts("Type COMMAND to request satellite burst\r\n");
    uart2_puts("> ");

    char command[CMD_LINE_MAX];

    /* MAIN LOOP */

    while (1)
    {
        if (USART_TryReadLine(command, sizeof(command)))
        {
            if (str_ieq(command, "COMMAND"))
            {
                uart2_puts("\r\nCOMMAND ACCEPTED\r\n");

                stat_packets_ok = 0;
                stat_packets_bad_crc = 0;
                stat_packets_misaddressed = 0;
                stat_packets_stale_tx = 0;
                stat_rx_timeouts = 0;
                stat_rx_errors = 0;
                stat_rx_crc_errors = 0;
                stat_rx_header_errors = 0;

                Send_Command(CMD_REQUEST_BURST);

                uart2_puts("\r\nLISTENING 435 MHz...\r\n");

                rx_done_flag = 0;
                rx_timeout_flag = 0;
                rx_error_flag = 0;

                /* Enable RX interrupt BEFORE RX */

                SUBGRF_SetDioIrqParams
                (
                    IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                    IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                    IRQ_RADIO_NONE,
                    IRQ_RADIO_NONE
                );

                /* RadioApp_StartRx() resets the SUBGHZ buffer base
                   address, sets the RX channel, and arms Rx(0) - all
                   in one place, identical to every later re-arm below.
                   This is the fix for the ground station reading stale
                   bytes left over from a previous TX/RX cycle. */
                RadioApp_StartRx();

                memset((void*)rx_frame_buffer, 0xAA, sizeof(rx_frame_buffer));
                RadioApp_StartRx();

                uint32_t start = HAL_GetTick();
                uint32_t last_event = start;
                uint32_t last_heartbeat = start;

                while ((HAL_GetTick() - start) < RX_SESSION_MS)
                {
                    if (rx_done_flag)
                    {
                        rx_done_flag = 0;
                        last_event = HAL_GetTick();
                        last_heartbeat = last_event;

                        Process_Received_Frame();

                        /* resets buffer base address every re-arm too */
                        RadioApp_ResumeRx();
                    }

                    if (rx_timeout_flag || rx_error_flag)
                    {
                        rx_timeout_flag = 0;
                        rx_error_flag = 0;
                        last_event = HAL_GetTick();
                        last_heartbeat = last_event;

                        RadioApp_ResumeRx();
                    }

                    if ((HAL_GetTick() - last_heartbeat) > RX_HEARTBEAT_MS)
                    {
                        last_heartbeat = HAL_GetTick();

                        uart2_printf
                        (
                            "... still listening (%lus elapsed, %lu ok / %lu crc-fail / %lu rx-errors / %lu timeouts so far)\r\n",
                            (unsigned long)((HAL_GetTick() - start) / 1000UL),
                            (unsigned long)stat_packets_ok,
                            (unsigned long)stat_packets_bad_crc,
                            (unsigned long)stat_rx_errors,
                            (unsigned long)stat_rx_timeouts
                        );
                    }

                    if ((HAL_GetTick() - last_event) > RX_WATCHDOG_MS)
                    {
                        uart2_puts("\r\nRX watchdog: no events for a while - re-arming radio\r\n");

                        Radio.Standby();

                        /* resets buffer base address too, same as every
                           other re-arm point */
                        RadioApp_ResumeRx();

                        SUBGRF_SetDioIrqParams
                        (
                            IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                            IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                            IRQ_RADIO_NONE,
                            IRQ_RADIO_NONE
                        );

                        last_event = HAL_GetTick();
                    }
                }

                Radio.Standby();

                uart2_printf
                (
                    "\r\nRX SESSION COMPLETE: %lu ok, %lu crc-fail, %lu not-for-us, %lu stale-tx, %lu timeouts, %lu rx-errors (crc=%lu hdr=%lu)\r\n",
                    (unsigned long)stat_packets_ok,
                    (unsigned long)stat_packets_bad_crc,
                    (unsigned long)stat_packets_misaddressed,
                    (unsigned long)stat_packets_stale_tx,
                    (unsigned long)stat_rx_timeouts,
                    (unsigned long)stat_rx_errors,
                    (unsigned long)stat_rx_crc_errors,
                    (unsigned long)stat_rx_header_errors
                );
            }
            else
            {
                uart2_printf("UNKNOWN COMMAND : %s\r\n", command);
            }

            uart2_puts("\r\n> ");
        }
    }

    return 0;
}