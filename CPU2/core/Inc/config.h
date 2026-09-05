#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================
 * FREQUENCIES
 * ============================================================
 *
 * Satellite:
 *
 *     TX = 435.000 MHz
 *     RX = 437.375 MHz
 *
 * Ground station must use:
 *
 *     TX = 437.375 MHz
 *     RX = 435.000 MHz
 * ============================================================
 */

#define DEFAULT_TX_FREQUENCY_HZ        435000000UL
#define DEFAULT_RX_FREQUENCY_HZ        437375000UL

/*
 * ============================================================
 * DEFAULT AX.25 ADDRESS
 * ============================================================
 */

#define DEFAULT_SOURCE_CALLSIGN       "NEPSAT"
#define DEFAULT_SOURCE_SSID            1

#define DEFAULT_DEST_CALLSIGN         "GROUND"
#define DEFAULT_DEST_SSID              0

/*
 * ============================================================
 * GFSK / G3RUH
 * ============================================================
 */

#define RADIO_TX_POWER_DBM             22

/*
 * Standard G3RUH data rate.
 */
#define RADIO_BIT_RATE_BPS             4800

/*
 * Standard G3RUH 4800 baud frequency deviation (+/- 2.4 kHz).
 */
#define RADIO_FDEV_HZ                  1200

/*
 * SX126x GMSK bandwidth.
 */
#define RADIO_RX_BANDWIDTH_HZ         23400

/*
 * 128 bits = 16 bytes.
 */
#define RADIO_PREAMBLE_LENGTH_BITS     128

#define RADIO_TX_TIMEOUT_MS            3000

/*
 * ============================================================
 * PHYSICAL PACKET LENGTH
 *
 * The SX126x packet is deliberately fixed length.
 *
 * This is NOT an AX.25 requirement.
 * It is our radio transport configuration.
 * ============================================================
 */

#define RADIO_FIXED_PACKET_LEN         200

/*
 * Number of HDLC flags before actual frame.
 * 64 flags = 512 bits = 53 ms audio lock time at 4800 baud.
 */
#define PROTOCOL_LEADING_FLAG_COUNT    19
/*  
*Mission COMMAND CONFIGURATION 13 BYTES
*/
#define CMD_CAMERA_COMMAND_LEN 13
#define CMD_ADCD_COMMAND_LEN 13
#define CMD_EPDM_COMMAND_LEN 13

/*13 bytes command of mission run*/
#define CMD_ADCD_COMMAND {0x53, 0x03, 0xA0, 0x53, 0xCF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define CMD_CAMERA_COMMAND {0x53, 0x04, 0xCC, 0x5E, 0xBD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define CMD_EPDM_COMMAND {0x53, 0x05, 0xEC, 0xCF, 0xCF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}


/*
 * ============================================================
 * RADIO CONFIG
 * ============================================================
 */

typedef struct
{
    uint32_t txFrequency;
    uint32_t rxFrequency;

    const char *sourceCallsign;
    uint8_t sourceSSID;

    const char *destCallsign;
    uint8_t destSSID;

    bool isSatelliteMode;

} RadioConfig_t;

/*
 * ============================================================
 * COMMANDS
 * ============================================================
 */

typedef enum
{
    CMD_NONE          = 0x00,

    CMD_REQUEST_BURST = 0x01,

    CMD_ACK           = 0x02,

    CMD_NACK          = 0x03,

    CMD_HEARTBEAT     = 0x04

} CommandOpcode_t;

#ifdef __cplusplus
}
#endif

#endif