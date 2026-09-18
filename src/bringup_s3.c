/* Bring-up firmware for the new ESP32-S3 N16R8 board -- NOT the real
 * application yet (see HARDWARE_MIGRATION.md). Confirmed so far: GPIO48
 * drives the onboard WS2812 RGB LED (matches a vendor pinout diagram,
 * verified against the physical board), and the working flash/monitor
 * procedure for this board's finicky USB-Serial/JTAG port (documented in
 * HARDWARE_MIGRATION.md's "Open questions -- resolved" section).
 *
 * This step adds just enough Wi-Fi/BLE/HTTP for the *already-installed,
 * unmodified* CarTheftGuard phone app to connect to this board and control
 * the LED's blink rate, to prove native Wi-Fi+BLE (no ESP-Hosted, no C6
 * co-processor) actually works on this chip before porting anything else.
 * Deliberately reuses the JC-ESP32P4-M3 board's exact BLE advertised name
 * ("JC-P4-C6") and AP SSID/password ("CarTheftGuard-P4" / "&Car1310") --
 * that's the ONLY way the phone app (which only knows to auto-join a board
 * advertising that specific name, see BoardLink.java's scanCallback) will
 * recognize this board at all. This is a deliberate, temporary bring-up
 * shortcut, not a real identity for this new hardware -- expect to pick
 * real distinct values once this becomes its own product line rather than
 * a stand-in for the P4 board during testing.
 *
 * POST /api/frequency ("freq <ms>" body) and its response format
 * ("OK freq=<ms> ms\r\n" / "ERR ...\r\n") are copied verbatim from
 * JC-ESP32P4-M3's apply_freq_command/frequency_http_handler so the app's
 * existing BoardLink.sendFrequency() needs zero changes to work here.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_coexist.h"
#include "esp_bt.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "led_strip.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "bringup";

/* Forward declarations -- can_mode_http_handler (defined well before
 * can_set_mode) and start_can (which starts obd_query_task before that
 * task's own definition later in the file) both need these. */
static void can_set_mode(bool passive);
static void obd_query_task(void *arg);

#define LED_GPIO_CANDIDATE 48
/* First real test of the native TWAI (CAN) controller -- the whole point
 * of this hardware migration (see HARDWARE_MIGRATION.md's "Why change
 * hardware"): no SPI, no external MCP2515 controller chip, just these two
 * GPIOs straight into an SN65HVD230 transceiver's TXD/RXD pins. 500 kbps
 * to match ArdunioUsbBridgeToCan's simulator (see project memory). Chosen
 * as safe general-purpose pins per the vendor pinout diagram: not
 * strapping (0/3/45/46), not USB (19/20), not PSRAM (35/36/37), not the
 * confirmed LED pin (48). */
#define CAN_TX_GPIO 4
#define CAN_RX_GPIO 5
/* ArdunioUsbBridgeToCan's simulated GPS goes out over SoftwareSerial on
 * the Arduino's pin 3 (GPS_TX_PIN in its main.cpp) at 9600 baud, 5V logic
 * -- routed through a voltage divider down to 3.3V before reaching here,
 * since S3 GPIOs aren't 5V-tolerant. TX_GPIO is wired but unused (nothing
 * needs to transmit back to the simulator's GPS input). */
#define GPS_TX_GPIO 6
#define GPS_RX_GPIO 7
#define GPS_BAUD 9600
#define GPS_UART_PORT UART_NUM_1
#define WIFI_AP_SSID "CarTheftGuard-P4"
#define WIFI_AP_PASSWORD "&Car1310"
#define WIFI_AP_IP "192.168.4.1"
#define BLE_DEVICE_NAME "JC-P4-C6"
#define BLINK_HALF_PERIOD_MIN_MS 10
#define BLINK_HALF_PERIOD_MAX_MS 60000

static led_strip_handle_t s_led;
static volatile uint32_t s_blink_half_period_ms = 500;
static uint8_t s_ble_addr_type = 0;

/* GET /api/gps state + JSON contract copied verbatim from
 * JC-ESP32P4-M3's gps_state_t/gps_http_handler so BoardLink.java's
 * existing GPS polling needs zero changes. */
typedef struct {
    bool fix_valid;
    double lat;
    double lon;
    float speed_kmh;
    float heading_deg;
    char utc_time[8];
    char utc_date[8];
    uint8_t satellites;
} gps_state_t;

static gps_state_t s_gps_state;
static portMUX_TYPE s_gps_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_last_gps_log_us;

/* GET /api/obd -- same JSON contract as JC-ESP32P4-M3's obd_http_handler.
 * Populated two ways now: passively, from the simulator's unsolicited
 * engine/vehicle broadcasts on 0x120/0x180 (decode_engine_broadcast/
 * decode_vehicle_broadcast below -- works with zero CAN TX, so it's always
 * live even in Passive mode or against a quiet bus), and actively, from
 * real Mode 01 PID polling once obd_query_task is running (see below --
 * needed against a real car, which won't broadcast unsolicited data the
 * way the simulator does). supported_pids/supported_pids_2 only ever come
 * from the active path (PID 0x00/0x20). */
typedef struct {
    uint32_t supported_pids;
    uint32_t supported_pids_2;
    int coolant_c;
    uint16_t rpm;
    uint8_t speed_kmh;
    uint8_t throttle_pct;
} obd_state_t;

#define CAN_ID_ENGINE_STATE 0x120
#define CAN_ID_VEHICLE_STATE 0x180

static obd_state_t s_obd_state;
static portMUX_TYPE s_obd_lock = portMUX_INITIALIZER_UNLOCKED;

/* --- Active OBD-II / UDS engine, ported from JC-ESP32P4-M3 (see that
 * file's equivalent structs/functions for the original doc comments this
 * was adapted from). Only the pieces needed for real-car testing: dual-
 * scheme (11-bit/29-bit) Mode 01 polling, simulator/real-car partner
 * detection, and a configurable Deep Scan (UDS session + DID identify
 * sweep). Left out as not essential right now: DTC read/clear, VIN read,
 * the plain address-discovery sweep, and VWTP -- all real JC-ESP32P4-M3
 * features, just not ported here yet. */

#define OBD_MODE_CURRENT_DATA 0x01
#define OBD_REQUEST_ID 0x7DF
#define OBD_RESPONSE_ID_MIN 0x7E8
#define OBD_RESPONSE_ID_MAX 0x7EF
#define OBD_REQUEST_ID_EXT 0x18DB33F1UL
#define OBD_RESPONSE_ID_EXT_MIN 0x18DAF100UL
#define OBD_RESPONSE_ID_EXT_MAX 0x18DAF1FFUL
#define OBD_RESPONSE_TIMEOUT_MS 500
#define OBD_QUERY_INTERVAL_MS 200
#define OBD_ADDR_RESET_AFTER_TIMEOUTS 5

/* Bench simulator's private identify ping -- see obd_identify_partner().
 * A real car never answers this ID, so silence after a few attempts means
 * "real car" (or "nothing on the bus yet"), not an error. */
#define SIM_IDENTIFY_CAN_ID 0x702
#define SIM_IDENTIFY_MAGIC 0xA5
#define SIM_IDENTIFY_TIMEOUT_MS 250
#define SIM_IDENTIFY_ATTEMPTS 3

#define ISOTP_PCI_SF 0x0
#define ISOTP_PCI_FF 0x1
#define ISOTP_PCI_CF 0x2
#define ISOTP_MAX_PAYLOAD 32

#define DEEP_SCAN_ID_START 0xF180
#define DEEP_SCAN_ID_END 0xF1A0
#define DEEP_SCAN_MAX_HITS 8
#define DEEP_SCAN_MAX_DIDS_PER_HIT 16
#define DEEP_SCAN_NVS_NAMESPACE "deepscan"
#define UDS_ADDR_SCAN_MAX_RANGE 256

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} obd_frame_t;

typedef enum {
    OBD_ADDR_UNKNOWN = 0,
    OBD_ADDR_STANDARD,
    OBD_ADDR_EXTENDED,
} obd_addressing_t;
static volatile obd_addressing_t s_obd_addressing = OBD_ADDR_UNKNOWN;
static portMUX_TYPE s_obd_addressing_lock = portMUX_INITIALIZER_UNLOCKED;

typedef enum {
    OBD_PARTNER_UNKNOWN = 0,
    OBD_PARTNER_SIM_11,
    OBD_PARTNER_SIM_29,
    OBD_PARTNER_CAR_11,
    OBD_PARTNER_CAR_29,
} obd_partner_t;
static volatile obd_partner_t s_obd_partner = OBD_PARTNER_UNKNOWN;
static portMUX_TYPE s_obd_partner_lock = portMUX_INITIALIZER_UNLOCKED;

static volatile bool s_identify_pending;
static volatile uint8_t s_identify_response[2];
static SemaphoreHandle_t s_identify_semaphore;

/* True only for the brief window between sending a request and getting/
 * timing out its reply -- see JC-ESP32P4-M3's s_obd_request_pending doc
 * comment for why this matters (the simulator's own unsolicited broadcasts
 * would otherwise desync the request/response pairing). */
static volatile bool s_obd_request_pending;
static QueueHandle_t s_obd_response_queue;

static volatile bool s_uds_request_pending;
static volatile uint32_t s_uds_expected_response_id;
static QueueHandle_t s_uds_response_queue;

static volatile bool s_can_passive;

typedef enum {
    UDS_SCAN_IDLE = 0,
    UDS_SCAN_RUNNING,
    UDS_SCAN_DONE,
    UDS_SCAN_ERROR,
} uds_scan_state_t;

typedef enum {
    UDS_SESSION_NOT_ATTEMPTED = 0,
    UDS_SESSION_POSITIVE,
    UDS_SESSION_NEGATIVE,
    UDS_SESSION_TIMEOUT,
} uds_session_state_t;

typedef struct {
    uint16_t did;
    uint8_t dlc;
    uint8_t data[8];
} uds_scan_result_t;

typedef struct {
    uint32_t addr;
    uds_session_state_t session_state;
    uint8_t did_result_count;
    uds_scan_result_t did_results[DEEP_SCAN_MAX_DIDS_PER_HIT];
} deep_scan_hit_t;

typedef struct {
    uint32_t req_start;
    uint32_t req_end;
    uint32_t response_offset;
    bool extended;
    uds_scan_state_t state;
    uint32_t current_req;
    uint8_t hit_count;
    deep_scan_hit_t hits[DEEP_SCAN_MAX_HITS];
} deep_scan_t;

static deep_scan_t s_deep_scan;
static portMUX_TYPE s_deep_scan_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_deep_scan_requested;

static void decode_engine_broadcast(const uint8_t *data, uint8_t dlc)
{
    if (dlc < 4) {
        return;
    }
    uint16_t rpm_raw = ((uint16_t)data[0] << 8) | data[1];
    int coolant_c = (int)data[2] - 40;
    uint8_t throttle_pct = (uint8_t)((uint16_t)data[3] * 100u / 255u);

    portENTER_CRITICAL(&s_obd_lock);
    s_obd_state.rpm = rpm_raw / 4;
    s_obd_state.coolant_c = coolant_c;
    s_obd_state.throttle_pct = throttle_pct;
    portEXIT_CRITICAL(&s_obd_lock);
}

static void decode_vehicle_broadcast(const uint8_t *data, uint8_t dlc)
{
    if (dlc < 2) {
        return;
    }
    uint16_t speed_centi_kmh = ((uint16_t)data[0] << 8) | data[1];

    portENTER_CRITICAL(&s_obd_lock);
    s_obd_state.speed_kmh = (uint8_t)(speed_centi_kmh / 100);
    portEXIT_CRITICAL(&s_obd_lock);
}

/* Decodes one active Mode 01 PID response (buf[0]=len, buf[1]=mode+0x40,
 * buf[2]=pid, buf[3..]=data) into s_obd_state -- ported verbatim from
 * JC-ESP32P4-M3's obd_print_response (SAE J1979 formulas per PID), minus
 * the printf-style logging (ESP_LOGI is enough here). */
static void obd_print_response(uint8_t pid, const uint8_t *buf, uint8_t len)
{
    switch (pid) {
        case 0x00:
            if (len >= 7) {
                uint32_t bitmask = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) |
                                    ((uint32_t)buf[5] << 8) | buf[6];
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.supported_pids = bitmask;
                portEXIT_CRITICAL(&s_obd_lock);
            }
            break;
        case 0x05:
            if (len >= 4) {
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.coolant_c = buf[3] - 40;
                portEXIT_CRITICAL(&s_obd_lock);
            }
            break;
        case 0x0C:
            if (len >= 5) {
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.rpm = ((unsigned)buf[3] * 256 + buf[4]) / 4;
                portEXIT_CRITICAL(&s_obd_lock);
            }
            break;
        case 0x0D:
            if (len >= 4) {
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.speed_kmh = buf[3];
                portEXIT_CRITICAL(&s_obd_lock);
            }
            break;
        case 0x11:
            if (len >= 4) {
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.throttle_pct = (buf[3] * 100u) / 255u;
                portEXIT_CRITICAL(&s_obd_lock);
            }
            break;
        case 0x20:
            if (len >= 7) {
                uint32_t bitmask = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) |
                                    ((uint32_t)buf[5] << 8) | buf[6];
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.supported_pids_2 = bitmask;
                portEXIT_CRITICAL(&s_obd_lock);
            }
            break;
        default:
            ESP_LOGD(TAG, "OBD PID 0x%02x -> unrecognized response", pid);
            break;
    }
}

/* GET /api/can ring buffer + JSON contract, same idea -- see
 * JC-ESP32P4-M3's can_capture_frame_t/can_capture_http_handler. Unlike
 * that board, extended/rtr here reflect the *real* flags from
 * twai_message_t (the P4 board's MCP2515 path never tracked them and
 * always reported false; the native TWAI driver already gives us the
 * real values for free). "passive"/"isr_count"/"timeout_count" are kept
 * as fields for JSON-shape compatibility but aren't meaningful for the
 * native driver yet (no listen-only mode wired up in this bring-up) --
 * always report false/0/0. */
typedef struct {
    uint64_t sequence;
    int64_t timestamp_us;
    uint32_t id;
    bool extended;
    bool rtr;
    uint8_t dlc;
    uint8_t data[8];
} can_capture_frame_t;

#define CAN_CAPTURE_CAPACITY 4096
#define CAN_CAPTURE_HTTP_BATCH 128
#define CAN_CAPTURE_RESPONSE_SIZE 24576

/* Allocated in PSRAM (see start_can()), not a static internal-RAM array --
 * at CAN_CAPTURE_CAPACITY=4096 frames (~130KB) this was the single largest
 * consumer of internal RAM in the whole firmware, and WiFi/lwIP can only
 * use internal RAM (CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP is off). Measured
 * root cause of the CAN+GPS/WiFi-BLE conflict: internal heap after
 * CAN+GPS+WiFi+BLE init was only 14KB free / 7.5KB largest contiguous
 * block (vs. 23KB/15KB with CAN alone) -- tight enough that WiFi's own TX
 * buffer allocation for the DHCP OFFER intermittently failed. This buffer
 * doesn't need DMA/ISR access (only touched from can_receive_task, a
 * normal task context), so PSRAM is a safe, effectively free fix. */
static can_capture_frame_t *s_can_capture;
static uint64_t s_can_capture_sequence;
static portMUX_TYPE s_can_capture_lock = portMUX_INITIALIZER_UNLOCKED;

static void capture_can_frame(const twai_message_t *message)
{
    int64_t timestamp_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_can_capture_lock);
    uint64_t sequence = ++s_can_capture_sequence;
    can_capture_frame_t *frame = &s_can_capture[(sequence - 1) % CAN_CAPTURE_CAPACITY];
    frame->sequence = sequence;
    frame->timestamp_us = timestamp_us;
    frame->id = message->identifier;
    frame->extended = message->extd;
    frame->rtr = message->rtr;
    frame->dlc = message->data_length_code > 8 ? 8 : message->data_length_code;
    memcpy(frame->data, message->data, frame->dlc);
    portEXIT_CRITICAL(&s_can_capture_lock);
}

/* Transmits one frame -- the native TWAI driver takes standard and extended
 * (29-bit) IDs through the same call (just the .extd flag), unlike
 * JC-ESP32P4-M3's MCP2515 path (mcp2515_send), which needed separate
 * register-level handling for each. Silently drops the send if the bus is
 * in BUS_OFF or the queue is full rather than blocking the caller -- every
 * caller here already has its own timeout waiting for a response, so a
 * failed transmit just surfaces as that timeout instead of a distinct
 * error path. */
static void can_send(uint32_t id, bool extended, const uint8_t *data, uint8_t len)
{
    twai_message_t msg = {0};
    msg.identifier = id;
    msg.extd = extended;
    msg.data_length_code = len > 8 ? 8 : len;
    memcpy(msg.data, data, msg.data_length_code);
    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "can_send id=0x%lx failed: %s", (unsigned long)id, esp_err_to_name(err));
    }
}

/* Reassembles one ISO-TP (ISO 15765-2) message starting from an already-
 * dequeued frame that begins it -- ported verbatim from JC-ESP32P4-M3's
 * isotp_reassemble (see that file for the full doc comment). Note: like
 * the original, Consecutive Frames are pulled from s_obd_response_queue
 * specifically, even when called from the UDS path (uds_send_and_receive
 * below) -- harmless in practice since every UDS response seen so far
 * (session control, DID reads) has fit in a Single Frame, but a genuinely
 * multi-frame UDS response would need its own queue to reassemble
 * correctly. Carried over as-is rather than "fixed" without being able to
 * test a real multi-frame UDS response. */
static bool isotp_reassemble(uint32_t response_id, const obd_frame_t *first,
                              uint8_t *out_payload, uint8_t *out_len, int64_t deadline_us)
{
    if (first->dlc < 1) {
        return false;
    }
    uint8_t pci = (first->data[0] >> 4) & 0x0F;

    if (pci == ISOTP_PCI_SF) {
        uint8_t sf_len = first->data[0] & 0x0F;
        if (sf_len > 7) {
            sf_len = 7;
        }
        if (sf_len > ISOTP_MAX_PAYLOAD) {
            sf_len = ISOTP_MAX_PAYLOAD;
        }
        memcpy(out_payload, &first->data[1], sf_len);
        *out_len = sf_len;
        return true;
    }

    if (pci != ISOTP_PCI_FF) {
        return false;
    }

    uint16_t total_len = ((uint16_t)(first->data[0] & 0x0F) << 8) | first->data[1];
    if (total_len > ISOTP_MAX_PAYLOAD) {
        total_len = ISOTP_MAX_PAYLOAD;
    }
    uint8_t received = (total_len < 6) ? total_len : 6;
    memcpy(out_payload, &first->data[2], received);

    bool response_is_extended = response_id > 0x7FF;
    uint32_t request_id;
    if (response_is_extended) {
        uint8_t byte1 = (uint8_t)(response_id >> 8);
        uint8_t byte0 = (uint8_t)response_id;
        request_id = (response_id & 0xFFFF0000UL) | ((uint32_t)byte0 << 8) | byte1;
    } else {
        request_id = response_id - 8;
    }
    uint8_t fc[8] = { 0x30, 0x00, 0x00, 0, 0, 0, 0, 0 };
    can_send(request_id, response_is_extended, fc, sizeof(fc));

    uint8_t expected_seq = 1;
    while (received < total_len) {
        int64_t remaining_ms = (deadline_us - esp_timer_get_time()) / 1000;
        if (remaining_ms <= 0) {
            break;
        }
        obd_frame_t frame;
        if (!xQueueReceive(s_obd_response_queue, &frame, pdMS_TO_TICKS(remaining_ms))) {
            break;
        }
        if (frame.id != response_id || frame.dlc < 1) {
            continue;
        }
        uint8_t frame_pci = (frame.data[0] >> 4) & 0x0F;
        if (frame_pci != ISOTP_PCI_CF || (frame.data[0] & 0x0F) != (expected_seq & 0x0F)) {
            continue;
        }
        uint8_t chunk = total_len - received;
        if (chunk > 7) {
            chunk = 7;
        }
        memcpy(out_payload + received, &frame.data[1], chunk);
        received += chunk;
        expected_seq++;
    }

    *out_len = received;
    return received > 0;
}

/* Sends one OBD request via both addressing schemes until one is locked in
 * -- see JC-ESP32P4-M3's obd_send_request doc comment (same dual-scheme
 * probe-then-commit behavior). */
static void obd_send_request(const uint8_t *request, uint8_t len)
{
    portENTER_CRITICAL(&s_obd_addressing_lock);
    obd_addressing_t addressing = s_obd_addressing;
    portEXIT_CRITICAL(&s_obd_addressing_lock);

    if (addressing != OBD_ADDR_EXTENDED) {
        can_send(OBD_REQUEST_ID, false, request, len);
    }
    if (addressing != OBD_ADDR_STANDARD) {
        can_send(OBD_REQUEST_ID_EXT, true, request, len);
    }
}

/* Pings the bench simulator's private identify ID -- see JC-ESP32P4-M3's
 * obd_identify_partner doc comment. A response means the simulator is on
 * the bus and tells us its addressing mode directly; no response after
 * SIM_IDENTIFY_ATTEMPTS means either a real vehicle (infer from whatever
 * s_obd_addressing has locked onto from real OBD traffic) or nothing
 * conclusive yet. */
static void obd_identify_partner(void)
{
    if (s_can_passive) {
        portENTER_CRITICAL(&s_obd_partner_lock);
        s_obd_partner = OBD_PARTNER_UNKNOWN;
        portEXIT_CRITICAL(&s_obd_partner_lock);
        return;
    }

    uint8_t ping[1] = { 0x01 };
    for (int attempt = 0; attempt < SIM_IDENTIFY_ATTEMPTS; attempt++) {
        s_identify_response[0] = 0;
        s_identify_response[1] = 0;
        s_identify_pending = true;
        can_send(SIM_IDENTIFY_CAN_ID, false, ping, sizeof(ping));
        bool got = xSemaphoreTake(s_identify_semaphore, pdMS_TO_TICKS(SIM_IDENTIFY_TIMEOUT_MS)) == pdTRUE;
        s_identify_pending = false;

        if (got && s_identify_response[0] == SIM_IDENTIFY_MAGIC) {
            bool ext = s_identify_response[1] != 0;
            portENTER_CRITICAL(&s_obd_addressing_lock);
            s_obd_addressing = ext ? OBD_ADDR_EXTENDED : OBD_ADDR_STANDARD;
            portEXIT_CRITICAL(&s_obd_addressing_lock);
            portENTER_CRITICAL(&s_obd_partner_lock);
            s_obd_partner = ext ? OBD_PARTNER_SIM_29 : OBD_PARTNER_SIM_11;
            portEXIT_CRITICAL(&s_obd_partner_lock);
            return;
        }
    }

    portENTER_CRITICAL(&s_obd_addressing_lock);
    obd_addressing_t addressing = s_obd_addressing;
    portEXIT_CRITICAL(&s_obd_addressing_lock);
    portENTER_CRITICAL(&s_obd_partner_lock);
    if (addressing == OBD_ADDR_STANDARD) {
        s_obd_partner = OBD_PARTNER_CAR_11;
    } else if (addressing == OBD_ADDR_EXTENDED) {
        s_obd_partner = OBD_PARTNER_CAR_29;
    } else {
        s_obd_partner = OBD_PARTNER_UNKNOWN;
    }
    portEXIT_CRITICAL(&s_obd_partner_lock);
}

static const char *obd_partner_to_string(obd_partner_t partner)
{
    switch (partner) {
        case OBD_PARTNER_SIM_11: return "SIM_11";
        case OBD_PARTNER_SIM_29: return "SIM_29";
        case OBD_PARTNER_CAR_11: return "CAR_11";
        case OBD_PARTNER_CAR_29: return "CAR_29";
        default: return "UNKNOWN";
    }
}

/* Point-to-point UDS send/wait -- exact request/response CAN ID pair
 * supplied by the caller, unlike obd_send_request's dual-scheme functional
 * broadcast. Shared by uds_read_did (service 0x22) and uds_start_session
 * (service 0x10) below. */
static bool uds_send_and_receive(uint32_t request_id, uint32_t response_id, bool extended,
                                  const uint8_t *request, uint8_t request_len,
                                  uint8_t *out_payload, uint8_t *out_len)
{
    s_uds_expected_response_id = response_id;
    s_uds_request_pending = true;
    can_send(request_id, extended, request, request_len);

    int64_t deadline_us = esp_timer_get_time() + ((int64_t)OBD_RESPONSE_TIMEOUT_MS * 1000);
    obd_frame_t response;
    bool got = xQueueReceive(s_uds_response_queue, &response, pdMS_TO_TICKS(OBD_RESPONSE_TIMEOUT_MS));
    s_uds_request_pending = false;
    if (!got) {
        return false;
    }
    return isotp_reassemble(response.id, &response, out_payload, out_len, deadline_us);
}

static bool uds_read_did(uint32_t request_id, uint32_t response_id, bool extended,
                          uint16_t did, uint8_t *out_payload, uint8_t *out_len)
{
    uint8_t request[8] = { 0x03, 0x22, (uint8_t)(did >> 8), (uint8_t)did, 0, 0, 0, 0 };
    return uds_send_and_receive(request_id, response_id, extended, request, sizeof(request),
                                 out_payload, out_len);
}

static uds_session_state_t uds_start_session(uint32_t request_id, uint32_t response_id, bool extended)
{
    uint8_t request[8] = { 0x02, 0x10, 0x03, 0, 0, 0, 0, 0 };
    uint8_t payload[ISOTP_MAX_PAYLOAD];
    uint8_t payload_len = 0;
    bool got = uds_send_and_receive(request_id, response_id, extended, request, sizeof(request),
                                     payload, &payload_len);
    if (!got) {
        return UDS_SESSION_TIMEOUT;
    }
    if (payload_len >= 2 && payload[0] == 0x50 && payload[1] == 0x03) {
        return UDS_SESSION_POSITIVE;
    }
    return UDS_SESSION_NEGATIVE;
}

/* --- Deep Scan NVS persistence -- ported verbatim from JC-ESP32P4-M3 (see
 * deep_scan_t's doc comment there for why: surviving brownout resets during
 * an unattended sweep by persisting progress and resuming automatically at
 * boot instead of losing a long run to one crash partway through). */
static void deep_scan_nvs_save_progress(uint32_t current_req)
{
    nvs_handle_t handle;
    if (nvs_open(DEEP_SCAN_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_set_u32(handle, "cur_req", current_req);
    nvs_commit(handle);
    nvs_close(handle);
}

static void deep_scan_nvs_save_hits(void)
{
    nvs_handle_t handle;
    if (nvs_open(DEEP_SCAN_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    portENTER_CRITICAL(&s_deep_scan_lock);
    uint8_t hit_count = s_deep_scan.hit_count;
    deep_scan_hit_t hits_copy[DEEP_SCAN_MAX_HITS];
    memcpy(hits_copy, s_deep_scan.hits, sizeof(hits_copy));
    portEXIT_CRITICAL(&s_deep_scan_lock);
    nvs_set_u8(handle, "hit_count", hit_count);
    nvs_set_blob(handle, "hits", hits_copy, sizeof(hits_copy));
    nvs_commit(handle);
    nvs_close(handle);
}

static void deep_scan_nvs_set_active(bool active)
{
    nvs_handle_t handle;
    if (nvs_open(DEEP_SCAN_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_set_u8(handle, "active", active ? 1 : 0);
    nvs_commit(handle);
    nvs_close(handle);
}

static void deep_scan_nvs_save_params(uint32_t req_start, uint32_t req_end, uint32_t response_offset, bool extended)
{
    nvs_handle_t handle;
    if (nvs_open(DEEP_SCAN_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_set_u32(handle, "req_start", req_start);
    nvs_set_u32(handle, "req_end", req_end);
    nvs_set_u32(handle, "resp_off", response_offset);
    nvs_set_u8(handle, "extended", extended ? 1 : 0);
    nvs_set_u32(handle, "cur_req", req_start);
    nvs_set_u8(handle, "hit_count", 0);
    deep_scan_hit_t empty_hits[DEEP_SCAN_MAX_HITS] = {0};
    nvs_set_blob(handle, "hits", empty_hits, sizeof(empty_hits));
    nvs_set_u8(handle, "active", 1);
    nvs_commit(handle);
    nvs_close(handle);
}

static void deep_scan_resume_from_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open(DEEP_SCAN_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return;
    }
    uint8_t active = 0;
    if (nvs_get_u8(handle, "active", &active) != ESP_OK || !active) {
        nvs_close(handle);
        return;
    }
    uint32_t req_start = 0, req_end = 0, response_offset = 0, current_req = 0;
    uint8_t extended = 0, hit_count = 0;
    nvs_get_u32(handle, "req_start", &req_start);
    nvs_get_u32(handle, "req_end", &req_end);
    nvs_get_u32(handle, "resp_off", &response_offset);
    nvs_get_u8(handle, "extended", &extended);
    nvs_get_u32(handle, "cur_req", &current_req);
    nvs_get_u8(handle, "hit_count", &hit_count);
    deep_scan_hit_t hits[DEEP_SCAN_MAX_HITS] = {0};
    size_t hits_size = sizeof(hits);
    nvs_get_blob(handle, "hits", hits, &hits_size);
    nvs_close(handle);

    if (req_end < req_start || current_req > req_end) {
        return;
    }

    portENTER_CRITICAL(&s_deep_scan_lock);
    s_deep_scan.req_start = req_start;
    s_deep_scan.req_end = req_end;
    s_deep_scan.response_offset = response_offset;
    s_deep_scan.extended = extended != 0;
    s_deep_scan.current_req = current_req;
    s_deep_scan.hit_count = hit_count > DEEP_SCAN_MAX_HITS ? DEEP_SCAN_MAX_HITS : hit_count;
    memcpy(s_deep_scan.hits, hits, sizeof(hits));
    s_deep_scan.state = UDS_SCAN_RUNNING;
    portEXIT_CRITICAL(&s_deep_scan_lock);
    s_deep_scan_requested = true;
    ESP_LOGI(TAG, "Deep scan: resuming after reset at 0x%lx (range 0x%lx-0x%lx, %u hit(s) so far)",
             (unsigned long)current_req, (unsigned long)req_start, (unsigned long)req_end, (unsigned)hit_count);
}

/* Services a deep-scan request: sweeps request IDs, and for every address
 * that gets a real response (not a timeout), immediately runs the
 * identification-DID sweep against it before moving to the next candidate
 * -- see JC-ESP32P4-M3's obd_query_deep_scan_if_requested doc comment. */
static void obd_query_deep_scan_if_requested(void)
{
    if (!s_deep_scan_requested) {
        return;
    }
    s_deep_scan_requested = false;

    uint32_t req_start, req_end, response_offset, resume_req;
    bool extended;
    portENTER_CRITICAL(&s_deep_scan_lock);
    req_start = s_deep_scan.req_start;
    req_end = s_deep_scan.req_end;
    response_offset = s_deep_scan.response_offset;
    extended = s_deep_scan.extended;
    resume_req = s_deep_scan.current_req;
    portEXIT_CRITICAL(&s_deep_scan_lock);

    if (s_can_passive) {
        ESP_LOGW(TAG, "Deep scan requested while CAN bridge is passive; ignored");
        portENTER_CRITICAL(&s_deep_scan_lock);
        s_deep_scan.state = UDS_SCAN_ERROR;
        portEXIT_CRITICAL(&s_deep_scan_lock);
        deep_scan_nvs_set_active(false);
        return;
    }

    ESP_LOGI(TAG, "Deep scan starting at 0x%lx (range 0x%lx-0x%lx)",
             (unsigned long)resume_req, (unsigned long)req_start, (unsigned long)req_end);

    for (uint32_t req_id = resume_req; req_id <= req_end; req_id++) {
        portENTER_CRITICAL(&s_deep_scan_lock);
        s_deep_scan.current_req = req_id;
        portEXIT_CRITICAL(&s_deep_scan_lock);
        deep_scan_nvs_save_progress(req_id);

        uint32_t resp_id = req_id + response_offset;
        uds_session_state_t session_state = uds_start_session(req_id, resp_id, extended);

        if (session_state != UDS_SESSION_TIMEOUT) {
            ESP_LOGI(TAG, "Deep scan: 0x%lx -> %s, identifying...", (unsigned long)req_id,
                     session_state == UDS_SESSION_POSITIVE ? "positive" : "negative");

            deep_scan_hit_t hit;
            memset(&hit, 0, sizeof(hit));
            hit.addr = req_id;
            hit.session_state = session_state;
            for (uint32_t did = DEEP_SCAN_ID_START; did <= DEEP_SCAN_ID_END; did++) {
                uint8_t payload[ISOTP_MAX_PAYLOAD];
                uint8_t payload_len = 0;
                bool ok = uds_read_did(req_id, resp_id, extended, (uint16_t)did, payload, &payload_len);
                if (ok && payload_len >= 3 && payload[0] == 0x62 &&
                    (uint16_t)(((uint16_t)payload[1] << 8) | payload[2]) == (uint16_t)did &&
                    hit.did_result_count < DEEP_SCAN_MAX_DIDS_PER_HIT) {
                    uds_scan_result_t *result = &hit.did_results[hit.did_result_count++];
                    result->did = (uint16_t)did;
                    result->dlc = payload_len > 8 ? 8 : payload_len;
                    memcpy(result->data, payload, result->dlc);
                }
                vTaskDelay(pdMS_TO_TICKS(OBD_QUERY_INTERVAL_MS));
            }

            portENTER_CRITICAL(&s_deep_scan_lock);
            if (s_deep_scan.hit_count < DEEP_SCAN_MAX_HITS) {
                s_deep_scan.hits[s_deep_scan.hit_count++] = hit;
            }
            portEXIT_CRITICAL(&s_deep_scan_lock);
            deep_scan_nvs_save_hits();
            ESP_LOGI(TAG, "Deep scan: 0x%lx identified, %u DID(s) responded",
                     (unsigned long)req_id, (unsigned)hit.did_result_count);
        } else {
            ESP_LOGD(TAG, "Deep scan: 0x%lx -> timeout", (unsigned long)req_id);
            vTaskDelay(pdMS_TO_TICKS(OBD_QUERY_INTERVAL_MS));
        }
    }

    portENTER_CRITICAL(&s_deep_scan_lock);
    s_deep_scan.state = UDS_SCAN_DONE;
    portEXIT_CRITICAL(&s_deep_scan_lock);
    deep_scan_nvs_set_active(false);
    ESP_LOGI(TAG, "Deep scan complete: %u module(s) identified", (unsigned)s_deep_scan.hit_count);
}

/* Routes one received frame to whichever consumer (if any) is waiting for
 * it -- identify-ping reply, UDS point-to-point response, OBD Mode 01
 * response (locks in the addressing scheme on the first real reply, like a
 * real scan tool committing after its initial probe), or the simulator's
 * unsolicited engine/vehicle broadcasts. Called from can_receive_task right
 * after capture_can_frame(); capture always happens regardless of routing. */
static void route_can_frame(const twai_message_t *message)
{
    uint32_t id = message->identifier;
    uint8_t dlc = message->data_length_code > 8 ? 8 : message->data_length_code;
    const uint8_t *data = message->data;

    if (s_uds_request_pending && id == s_uds_expected_response_id) {
        obd_frame_t frame = { .id = id, .dlc = dlc };
        memcpy(frame.data, data, sizeof(frame.data));
        if (xQueueSend(s_uds_response_queue, &frame, 0) != pdPASS) {
            ESP_LOGW(TAG, "UDS response queue full; dropping response");
        }
        return;
    }
    if (id == SIM_IDENTIFY_CAN_ID) {
        if (s_identify_pending && dlc >= 2) {
            s_identify_response[0] = data[0];
            s_identify_response[1] = data[1];
            xSemaphoreGive(s_identify_semaphore);
        }
        return;
    }
    bool response_is_extended = (id >= OBD_RESPONSE_ID_EXT_MIN && id <= OBD_RESPONSE_ID_EXT_MAX);
    bool is_obd_response = (id >= OBD_RESPONSE_ID_MIN && id <= OBD_RESPONSE_ID_MAX) || response_is_extended;
    if (is_obd_response) {
        portENTER_CRITICAL(&s_obd_addressing_lock);
        if (s_obd_addressing == OBD_ADDR_UNKNOWN) {
            s_obd_addressing = response_is_extended ? OBD_ADDR_EXTENDED : OBD_ADDR_STANDARD;
        }
        portEXIT_CRITICAL(&s_obd_addressing_lock);
        if (s_can_passive) {
            if (dlc >= 3 && data[1] == (uint8_t)(0x40 | OBD_MODE_CURRENT_DATA)) {
                obd_print_response(data[2], data, dlc);
            }
        } else if (s_obd_request_pending) {
            obd_frame_t frame = { .id = id, .dlc = dlc };
            memcpy(frame.data, data, sizeof(frame.data));
            if (xQueueSend(s_obd_response_queue, &frame, 0) != pdPASS) {
                ESP_LOGW(TAG, "OBD response queue full; dropping response");
            }
        } else if (dlc >= 3 && data[1] == (uint8_t)(0x40 | OBD_MODE_CURRENT_DATA)) {
            obd_print_response(data[2], data, dlc);
        }
        return;
    }
    if (id == CAN_ID_ENGINE_STATE) {
        decode_engine_broadcast(data, dlc);
        return;
    }
    if (id == CAN_ID_VEHICLE_STATE) {
        decode_vehicle_broadcast(data, dlc);
        return;
    }
}

/* Same parsing/response contract as JC-ESP32P4-M3's apply_freq_command --
 * see this file's top doc comment. */
static bool apply_freq_command(const char *line, char *out_msg, size_t out_msg_len)
{
    if (strncmp(line, "freq ", 5) != 0) {
        snprintf(out_msg, out_msg_len, "ERR unknown command '%s'. Type 'help'.\r\n", line);
        return false;
    }
    char *endptr = NULL;
    long value = strtol(line + 5, &endptr, 10);
    if (endptr == line + 5 || *endptr != '\0' ||
        value < BLINK_HALF_PERIOD_MIN_MS || value > BLINK_HALF_PERIOD_MAX_MS) {
        snprintf(out_msg, out_msg_len, "ERR invalid value. Enter a number of ms (%d-%d).\r\n",
                 BLINK_HALF_PERIOD_MIN_MS, BLINK_HALF_PERIOD_MAX_MS);
        return false;
    }
    s_blink_half_period_ms = (uint32_t)value;
    ESP_LOGI(TAG, "Blink half-period set to %ld ms", value);
    snprintf(out_msg, out_msg_len, "OK freq=%ld ms\r\n", value);
    return true;
}

static esp_err_t frequency_http_handler(httpd_req_t *request)
{
    char command[64] = {0};
    int received = httpd_req_recv(request, command, sizeof(command) - 1);
    char response[96];
    if (received <= 0) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Expected freq <ms>");
        return ESP_FAIL;
    }
    command[received] = '\0';
    bool accepted = apply_freq_command(command, response, sizeof(response));
    httpd_resp_set_type(request, "text/plain");
    httpd_resp_sendstr(request, response);
    return accepted ? ESP_OK : ESP_FAIL;
}

/* JSON contract copied verbatim from JC-ESP32P4-M3's gps_http_handler. */
/* JSON contract copied verbatim from JC-ESP32P4-M3's obd_http_handler
 * (see the obd_state_t doc comment for what's populated vs. left at 0). */
static esp_err_t obd_http_handler(httpd_req_t *request)
{
    obd_state_t state;
    portENTER_CRITICAL(&s_obd_lock);
    state = s_obd_state;
    portEXIT_CRITICAL(&s_obd_lock);
    char response[224];
    snprintf(response, sizeof(response),
             "{\"supported_pids\":\"%08lx\",\"supported_pids_2\":\"%08lx\",\"coolant_c\":%d,\"rpm\":%u,\"speed_kmh\":%u,\"throttle_pct\":%u}",
             (unsigned long)state.supported_pids, (unsigned long)state.supported_pids_2,
             state.coolant_c, state.rpm, state.speed_kmh, state.throttle_pct);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

static esp_err_t gps_http_handler(httpd_req_t *request)
{
    gps_state_t state;
    portENTER_CRITICAL(&s_gps_lock);
    state = s_gps_state;
    portEXIT_CRITICAL(&s_gps_lock);

    char response[256];
    snprintf(response, sizeof(response),
             "{\"fix_valid\":%s,\"lat\":%.6f,\"lon\":%.6f,\"speed_kmh\":%.1f,"
             "\"heading_deg\":%.1f,\"utc_time\":\"%s\",\"utc_date\":\"%s\",\"satellites\":%u}",
             state.fix_valid ? "true" : "false", state.lat, state.lon,
             (double)state.speed_kmh, (double)state.heading_deg,
             state.utc_time, state.utc_date, state.satellites);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

/* JSON contract copied verbatim from JC-ESP32P4-M3's
 * can_capture_http_handler -- see capture_can_frame's doc comment for the
 * one real difference (extended/rtr are the real flags here, not always
 * false). */
static esp_err_t can_capture_http_handler(httpd_req_t *request)
{
    uint64_t after = 0;
    char query[48];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK) {
        char value[24];
        if (httpd_query_key_value(query, "after", value, sizeof(value)) == ESP_OK) {
            after = strtoull(value, NULL, 10);
        }
    }

    can_capture_frame_t batch[CAN_CAPTURE_HTTP_BATCH];
    size_t count = 0;
    uint64_t latest;
    uint64_t dropped = 0;

    portENTER_CRITICAL(&s_can_capture_lock);
    latest = s_can_capture_sequence;
    uint64_t oldest = latest >= CAN_CAPTURE_CAPACITY ? latest - CAN_CAPTURE_CAPACITY + 1 : 1;
    uint64_t first = after + 1;
    if (first < oldest) {
        dropped = oldest - first;
        first = oldest;
    }
    for (uint64_t sequence = first; sequence <= latest && count < CAN_CAPTURE_HTTP_BATCH; sequence++) {
        can_capture_frame_t frame = s_can_capture[(sequence - 1) % CAN_CAPTURE_CAPACITY];
        if (frame.sequence == sequence) {
            batch[count++] = frame;
        }
    }
    portEXIT_CRITICAL(&s_can_capture_lock);

    twai_status_info_t status = {0};
    twai_get_status_info(&status);

    char *response = malloc(CAN_CAPTURE_RESPONSE_SIZE);
    if (response == NULL) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_ERR_NO_MEM;
    }
    size_t used = (size_t)snprintf(response, CAN_CAPTURE_RESPONSE_SIZE,
                                   "{\"latest\":%llu,\"dropped\":%llu,\"hardware_overflow\":%lu,"
                                   "\"passive\":false,\"isr_count\":0,\"timeout_count\":0,\"frames\":[",
                                   (unsigned long long)latest, (unsigned long long)dropped,
                                   (unsigned long)status.rx_overrun_count);
    for (size_t i = 0; i < count; i++) {
        can_capture_frame_t *frame = &batch[i];
        used += (size_t)snprintf(response + used, CAN_CAPTURE_RESPONSE_SIZE - used,
                                 "%s{\"seq\":%llu,\"time_us\":%lld,\"bus\":0,\"id\":%lu,"
                                 "\"extended\":%s,\"rtr\":%s,\"dlc\":%u,\"data\":\"",
                                 i == 0 ? "" : ",", (unsigned long long)frame->sequence,
                                 (long long)frame->timestamp_us, (unsigned long)frame->id,
                                 frame->extended ? "true" : "false", frame->rtr ? "true" : "false",
                                 frame->dlc);
        for (uint8_t j = 0; j < frame->dlc && used + 2 < CAN_CAPTURE_RESPONSE_SIZE; j++) {
            used += (size_t)snprintf(response + used, CAN_CAPTURE_RESPONSE_SIZE - used,
                                     "%02X", frame->data[j]);
        }
        used += (size_t)snprintf(response + used, CAN_CAPTURE_RESPONSE_SIZE - used, "\"}");
    }
    snprintf(response + used, CAN_CAPTURE_RESPONSE_SIZE - used, "]}");

    httpd_resp_set_type(request, "application/json");
    esp_err_t result = httpd_resp_sendstr(request, response);
    free(response);
    return result;
}

/* Re-probes (obd_identify_partner) on every request rather than just
 * reporting a cached value, so a fresh probe happens whenever the app
 * checks this -- matches JC-ESP32P4-M3's can_partner_http_handler. */
static esp_err_t can_partner_http_handler(httpd_req_t *request)
{
    obd_identify_partner();
    portENTER_CRITICAL(&s_obd_partner_lock);
    obd_partner_t partner = s_obd_partner;
    portEXIT_CRITICAL(&s_obd_partner_lock);

    char response[48];
    snprintf(response, sizeof(response), "{\"partner\":\"%s\"}", obd_partner_to_string(partner));
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

/* Body: "active" or "passive". Reconfigures the TWAI controller's actual
 * hardware mode -- see can_set_mode's doc comment. */
static esp_err_t can_mode_http_handler(httpd_req_t *request)
{
    char mode[8] = {0};
    int received = httpd_req_recv(request, mode, sizeof(mode) - 1);
    if (received <= 0) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Expected active or passive");
        return ESP_FAIL;
    }
    mode[received] = '\0';

    bool passive;
    if (strcmp(mode, "passive") == 0) {
        passive = true;
    } else if (strcmp(mode, "active") == 0) {
        passive = false;
    } else {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Expected active or passive");
        return ESP_FAIL;
    }

    can_set_mode(passive);
    httpd_resp_set_type(request, "text/plain");
    httpd_resp_sendstr(request, passive ? "OK passive" : "OK active");
    return ESP_OK;
}

/* Body (plain text): "req_start,req_end,offset,extended" (hex,hex,hex,0|1),
 * e.g. "710,710,6A,0" -- see JC-ESP32P4-M3's deep_scan_start_http_handler
 * doc comment (same contract, including the max-256-address range limit).
 * Always starts a *fresh* scan from req_start. */
static esp_err_t deep_scan_start_http_handler(httpd_req_t *request)
{
    char body[48] = {0};
    int received = httpd_req_recv(request, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Expected req_start,req_end,offset,extended");
        return ESP_FAIL;
    }
    body[received] = '\0';

    unsigned long req_start = 0, req_end = 0, offset = 0, extended = 0;
    if (sscanf(body, "%lx,%lx,%lx,%lu", &req_start, &req_end, &offset, &extended) != 4) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                             "Expected req_start,req_end,offset,extended (hex,hex,hex,0|1)");
        return ESP_FAIL;
    }
    if (req_end < req_start || (req_end - req_start + 1) > UDS_ADDR_SCAN_MAX_RANGE) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Address range invalid or too large (max 256)");
        return ESP_FAIL;
    }

    portENTER_CRITICAL(&s_deep_scan_lock);
    s_deep_scan.req_start = (uint32_t)req_start;
    s_deep_scan.req_end = (uint32_t)req_end;
    s_deep_scan.response_offset = (uint32_t)offset;
    s_deep_scan.extended = extended != 0;
    s_deep_scan.current_req = (uint32_t)req_start;
    s_deep_scan.hit_count = 0;
    memset(s_deep_scan.hits, 0, sizeof(s_deep_scan.hits));
    s_deep_scan.state = UDS_SCAN_RUNNING;
    portEXIT_CRITICAL(&s_deep_scan_lock);
    deep_scan_nvs_save_params((uint32_t)req_start, (uint32_t)req_end, (uint32_t)offset, extended != 0);
    s_deep_scan_requested = true;

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, "{\"status\":\"deep scan requested\"}");
    return ESP_OK;
}

/* Same JSON contract as JC-ESP32P4-M3's deep_scan_status_http_handler. */
static esp_err_t deep_scan_status_http_handler(httpd_req_t *request)
{
    deep_scan_t snapshot;
    portENTER_CRITICAL(&s_deep_scan_lock);
    snapshot = s_deep_scan;
    portEXIT_CRITICAL(&s_deep_scan_lock);

    const char *state_str;
    switch (snapshot.state) {
        case UDS_SCAN_RUNNING: state_str = "running"; break;
        case UDS_SCAN_DONE: state_str = "done"; break;
        case UDS_SCAN_ERROR: state_str = "error"; break;
        default: state_str = "idle"; break;
    }

    static char response[DEEP_SCAN_MAX_HITS * (DEEP_SCAN_MAX_DIDS_PER_HIT * 48 + 96) + 128];
    int written = snprintf(response, sizeof(response),
            "{\"state\":\"%s\",\"current_req\":\"0x%03lx\",\"hit_count\":%u,\"hits\":[",
            state_str, (unsigned long)snapshot.current_req, snapshot.hit_count);
    size_t used = (written > 0 && (size_t)written < sizeof(response)) ? (size_t)written : sizeof(response);
    for (uint8_t i = 0; i < snapshot.hit_count && i < DEEP_SCAN_MAX_HITS && used < sizeof(response); i++) {
        deep_scan_hit_t *hit = &snapshot.hits[i];
        written = snprintf(response + used, sizeof(response) - used,
                "%s{\"addr\":\"0x%03lx\",\"session\":\"%s\",\"did_results\":[",
                i == 0 ? "" : ",", (unsigned long)hit->addr,
                hit->session_state == UDS_SESSION_POSITIVE ? "positive" : "negative");
        used += (written > 0 && (size_t)written < sizeof(response) - used) ? (size_t)written : sizeof(response) - used;

        for (uint8_t d = 0; d < hit->did_result_count && d < DEEP_SCAN_MAX_DIDS_PER_HIT && used < sizeof(response); d++) {
            char hex[17] = {0};
            uint8_t hex_bytes = hit->did_results[d].dlc > 8 ? 8 : hit->did_results[d].dlc;
            for (uint8_t b = 0; b < hex_bytes; b++) {
                snprintf(hex + b * 2, 3, "%02X", hit->did_results[d].data[b]);
            }
            written = snprintf(response + used, sizeof(response) - used,
                    "%s{\"did\":\"0x%04x\",\"data\":\"%s\"}",
                    d == 0 ? "" : ",", hit->did_results[d].did, hex);
            used += (written > 0 && (size_t)written < sizeof(response) - used) ? (size_t)written : sizeof(response) - used;
        }
        if (used < sizeof(response)) {
            written = snprintf(response + used, sizeof(response) - used, "]}");
            used += (written > 0 && (size_t)written < sizeof(response) - used) ? (size_t)written : sizeof(response) - used;
        }
    }
    if (used < sizeof(response)) {
        snprintf(response + used, sizeof(response) - used, "]}");
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

static void start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    /* Stock default is exactly 8 -- we now register exactly 8 handlers,
     * right at the edge. Give headroom for whatever's added next. */
    config.max_uri_handlers = 16;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        return;
    }
    httpd_uri_t frequency_uri = {
        .uri = "/api/frequency",
        .method = HTTP_POST,
        .handler = frequency_http_handler,
    };
    httpd_uri_t gps_uri = {
        .uri = "/api/gps",
        .method = HTTP_GET,
        .handler = gps_http_handler,
    };
    httpd_uri_t can_uri = {
        .uri = "/api/can",
        .method = HTTP_GET,
        .handler = can_capture_http_handler,
    };
    httpd_uri_t obd_uri = {
        .uri = "/api/obd",
        .method = HTTP_GET,
        .handler = obd_http_handler,
    };
    httpd_uri_t can_partner_uri = {
        .uri = "/api/can/partner",
        .method = HTTP_GET,
        .handler = can_partner_http_handler,
    };
    httpd_uri_t can_mode_uri = {
        .uri = "/api/can/mode",
        .method = HTTP_POST,
        .handler = can_mode_http_handler,
    };
    httpd_uri_t deep_scan_start_uri = {
        .uri = "/api/deepscan",
        .method = HTTP_POST,
        .handler = deep_scan_start_http_handler,
    };
    httpd_uri_t deep_scan_status_uri = {
        .uri = "/api/deepscan",
        .method = HTTP_GET,
        .handler = deep_scan_status_http_handler,
    };
    httpd_register_uri_handler(server, &frequency_uri);
    httpd_register_uri_handler(server, &gps_uri);
    httpd_register_uri_handler(server, &can_uri);
    httpd_register_uri_handler(server, &obd_uri);
    httpd_register_uri_handler(server, &can_partner_uri);
    httpd_register_uri_handler(server, &can_mode_uri);
    httpd_register_uri_handler(server, &deep_scan_start_uri);
    httpd_register_uri_handler(server, &deep_scan_status_uri);
    ESP_LOGI(TAG, "HTTP API ready: POST /api/frequency, GET /api/gps, GET /api/can?after=<seq>, GET /api/obd, "
             "GET /api/can/partner, POST /api/can/mode, GET|POST /api/deepscan");
}

/* BLE+WiFi concurrency was tested exhaustively and abandoned -- see
 * HARDWARE_MIGRATION.md's DHCP section for the full record. Nothing short
 * of BLE never being initialized at all (not paused, not merely disabled)
 * gave reliable DHCP: esp_coex_preference_set(ESP_COEX_PREFER_WIFI), a
 * much slower advertising interval, pausing just the GAP advertisement,
 * and even a full esp_bt_controller_disable() all still lost the DHCP
 * OFFER transmit on every retry once a WiFi station associated. Real
 * porting work should treat BLE and this board's WiFi AP as mutually
 * exclusive by design (BLE for discovery only, fully off during WiFi use)
 * rather than trying to run both concurrently on this chip's single
 * shared 2.4GHz radio. */
static void wifi_ap_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *evt = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "WIFI_EVENT_AP_STACONNECTED: mac=%02x:%02x:%02x:%02x:%02x:%02x aid=%d",
                 evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5], evt->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *evt = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "WIFI_EVENT_AP_STADISCONNECTED: mac=%02x:%02x:%02x:%02x:%02x:%02x aid=%d",
                 evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5], evt->aid);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED) {
        ip_event_ap_staipassigned_t *evt = (ip_event_ap_staipassigned_t *)event_data;
        ESP_LOGI(TAG, "IP_EVENT_AP_STAIPASSIGNED: DHCP leased " IPSTR " to mac=%02x:%02x:%02x:%02x:%02x:%02x",
                 IP2STR(&evt->ip), evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5]);
    }
}

static void start_wifi_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "WiFi AP netif create failed");
        return;
    }
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ap_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &wifi_ap_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_country_t country = { .cc = "01", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL };
    esp_wifi_set_country(&country);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    wifi_config_t ap_config = {0};
    snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), "%s", WIFI_AP_SSID);
    ap_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    snprintf((char *)ap_config.ap.password, sizeof(ap_config.ap.password), "%s", WIFI_AP_PASSWORD);
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_config.ap.max_connection = 4;
    ap_config.ap.channel = 6;
    /* Same PMF reasoning as JC-ESP32P4-M3: Android's WifiNetworkSpecifier
     * needs PMF-capable or it can silently fail to auto-join. */
    ap_config.ap.pmf_cfg.capable = true;
    ap_config.ap.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* TX power cap tried during the coex investigation and reverted --
     * never proven to help (DHCP still failed with it on), and it's a
     * real deviation from the exact e9ad195 config that was 100% reliable
     * on the first bring-up test. Don't carry unproven changes forward. */

    ESP_LOGI(TAG, "SoftAP up: ssid=%s ip=%s", WIFI_AP_SSID, WIFI_AP_IP);
    start_http_server();
}

static void ble_start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.name = (const uint8_t *)BLE_DEVICE_NAME;
    fields.name_len = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE adv fields failed: rc=%d", rc);
        return;
    }
    struct ble_gap_adv_params adv = {0};
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    /* Reverted back to NimBLE's plain default interval (itvl_min/max left
     * at 0) -- matches the exact config from the first bring-up commit
     * (e9ad195) that was 100% reliable. A slowed-down interval (800/1600)
     * was tried during the coex investigation and never definitively
     * proven to help (it didn't fix DHCP with BLE active either), so
     * don't carry a deviation from the known-good config without reason. */
    rc = ble_gap_adv_start(s_ble_addr_type, NULL, BLE_HS_FOREVER, &adv, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE adv start failed: rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE advertising started as %s", BLE_DEVICE_NAME);
}

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_ble_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE addr infer failed: rc=%d", rc);
        return;
    }
    ble_start_advertising();
}

static void ble_host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void start_ble(void)
{
    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "NimBLE init failed: rc=%d", rc);
        return;
    }
    ble_svc_gap_init();
    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "NimBLE host started (native controller).");
}

/* Just logs every frame it receives -- proves the physical CAN wiring
 * (S3 -> SN65HVD230 -> CANH/CANL -> simulator) works before any real
 * protocol logic gets built on top. No termination resistor on this end
 * yet (see HARDWARE_MIGRATION.md) -- if frames show up reliably anyway,
 * that's still worth knowing, but don't conclude the bus is healthy
 * long-term from that alone; a missing terminator causing reflections
 * can look fine at low traffic/short wire runs and get worse under load. */
static void can_receive_task(void *arg)
{
    (void)arg;
    while (1) {
        twai_message_t message;
        esp_err_t err = twai_receive(&message, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) {
            capture_can_frame(&message);
            route_can_frame(&message);
        } else if (err == ESP_ERR_TIMEOUT) {
            twai_status_info_t diag = {0};
            twai_get_status_info(&diag);
            const char *state_str = diag.state == TWAI_STATE_RUNNING ? "RUNNING"
                    : diag.state == TWAI_STATE_BUS_OFF ? "BUS_OFF"
                    : diag.state == TWAI_STATE_STOPPED ? "STOPPED" : "RECOVERING";
            ESP_LOGI(TAG, "CAN RX: no frames -- state=%s tx_err=%lu rx_err=%lu "
                     "bus_err_count=%lu arb_lost=%lu tx_failed=%lu rx_missed=%lu rx_overrun=%lu",
                     state_str, (unsigned long)diag.tx_error_counter, (unsigned long)diag.rx_error_counter,
                     (unsigned long)diag.bus_error_count, (unsigned long)diag.arb_lost_count,
                     (unsigned long)diag.tx_failed_count, (unsigned long)diag.rx_missed_count,
                     (unsigned long)diag.rx_overrun_count);
        } else {
            ESP_LOGW(TAG, "twai_receive error: %s", esp_err_to_name(err));
        }

        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_BUS_OFF) {
            ESP_LOGE(TAG, "CAN controller is BUS-OFF (too many errors) -- recovering...");
            twai_initiate_recovery();
            vTaskDelay(pdMS_TO_TICKS(500));
            twai_start();
        }
    }
}

/* NMEA parsing below is ported verbatim from JC-ESP32P4-M3's
 * nmea_coord_to_decimal/nmea_split_fields/nmea_parse_rmc/nmea_parse_gga/
 * nmea_parse_line -- see that file for the field-index doc comments this
 * was copied from. Confirmed against the real simulator earlier tonight
 * (raw $GPRMC sentences with correct checksums received cleanly on
 * GPIO7); this adds the parsing on top of that already-proven wiring. */
static double nmea_coord_to_decimal(const char *value, char hemisphere)
{
    if (value == NULL || value[0] == '\0') {
        return NAN;
    }
    char *dot = strchr(value, '.');
    if (dot == NULL || (dot - value) < 2) {
        return NAN;
    }
    int degree_digits = (int)(dot - value) - 2;
    char degree_buf[4] = {0};
    if (degree_digits < 0 || degree_digits >= (int)sizeof(degree_buf)) {
        return NAN;
    }
    memcpy(degree_buf, value, degree_digits);
    double degrees_part = atof(degree_buf);
    double minutes_part = atof(value + degree_digits);
    double decimal = degrees_part + minutes_part / 60.0;
    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

static size_t nmea_split_fields(char *body, char **fields, size_t max_fields)
{
    size_t count = 0;
    char *field = body;
    while (count < max_fields) {
        fields[count++] = field;
        char *comma = strchr(field, ',');
        if (comma == NULL) {
            break;
        }
        *comma = '\0';
        field = comma + 1;
    }
    return count;
}

static void nmea_parse_rmc(char **fields, size_t count)
{
    if (count < 10) {
        return;
    }
    bool fix_valid = (fields[2][0] == 'A');
    double lat = NAN;
    double lon = NAN;
    float speed_kmh = 0;
    float heading_deg = 0;
    if (fix_valid) {
        lat = nmea_coord_to_decimal(fields[3], fields[4][0]);
        lon = nmea_coord_to_decimal(fields[5], fields[6][0]);
        speed_kmh = (float)(atof(fields[7]) * 1.852);
        heading_deg = (float)atof(fields[8]);
    }
    portENTER_CRITICAL(&s_gps_lock);
    s_gps_state.fix_valid = fix_valid;
    if (fix_valid) {
        s_gps_state.lat = lat;
        s_gps_state.lon = lon;
        s_gps_state.speed_kmh = speed_kmh;
        s_gps_state.heading_deg = heading_deg;
    }
    snprintf(s_gps_state.utc_time, sizeof(s_gps_state.utc_time), "%s", fields[1]);
    snprintf(s_gps_state.utc_date, sizeof(s_gps_state.utc_date), "%s", fields[9]);
    portEXIT_CRITICAL(&s_gps_lock);

    int64_t now_us = esp_timer_get_time();
    if (now_us - s_last_gps_log_us >= 1000000) {
        s_last_gps_log_us = now_us;
        if (fix_valid) {
            ESP_LOGI(TAG, "GPS fix -> lat=%.6f lon=%.6f speed=%.1f km/h heading=%.1f deg",
                     lat, lon, (double)speed_kmh, (double)heading_deg);
        } else {
            ESP_LOGI(TAG, "GPS -> waiting for valid fix");
        }
    }
}

static void nmea_parse_gga(char **fields, size_t count)
{
    if (count < 8) {
        return;
    }
    portENTER_CRITICAL(&s_gps_lock);
    s_gps_state.satellites = (uint8_t)atoi(fields[7]);
    portEXIT_CRITICAL(&s_gps_lock);
}

static void nmea_parse_line(char *line)
{
    if (line[0] != '$') {
        return;
    }
    char *star = strchr(line, '*');
    if (star == NULL || strlen(star) < 3) {
        return;
    }
    uint8_t checksum = 0;
    for (char *p = line + 1; p < star; p++) {
        checksum ^= (uint8_t)*p;
    }
    uint8_t expected = (uint8_t)strtol(star + 1, NULL, 16);
    if (checksum != expected) {
        ESP_LOGW(TAG, "GPS: checksum mismatch");
        return;
    }
    *star = '\0';

    char *fields[12];
    size_t count = nmea_split_fields(line + 1, fields, 12);
    if (count == 0) {
        return;
    }
    if (strcmp(fields[0], "GPRMC") == 0) {
        nmea_parse_rmc(fields, count);
    } else if (strcmp(fields[0], "GPGGA") == 0) {
        nmea_parse_gga(fields, count);
    }
}

/* Reassembles raw UART bytes into '\n'-terminated NMEA lines before
 * parsing -- sentences are short so a single-line buffer is enough. */
static void gps_receive_task(void *arg)
{
    (void)arg;
    static char line_buf[96];
    size_t line_len = 0;
    uint8_t rx_buf[64];

    while (1) {
        int read = uart_read_bytes(GPS_UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(1000));
        for (int i = 0; i < read; i++) {
            char c = (char)rx_buf[i];
            if (c == '\r') {
                continue;
            }
            if (c == '\n') {
                if (line_len > 0) {
                    line_buf[line_len] = '\0';
                    nmea_parse_line(line_buf);
                    line_len = 0;
                }
                continue;
            }
            if (line_len < sizeof(line_buf) - 1) {
                line_buf[line_len++] = c;
            }
        }
    }
}

static void start_gps(void)
{
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, GPS_TX_GPIO, GPS_RX_GPIO,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "GPS UART ready: TX=GPIO%d RX=GPIO%d %d baud", GPS_TX_GPIO, GPS_RX_GPIO, GPS_BAUD);
    xTaskCreatePinnedToCore(gps_receive_task, "gps_rx", 4096, NULL, 5, NULL, 1);
}

/* Reconfigures the TWAI controller's actual hardware mode -- true listen-
 * only (not just a software gate on our own transmit calls), so on a real
 * car "Passive" genuinely can't ACK or disturb bus traffic at the
 * controller level. Requires a full stop/uninstall/reinstall since
 * ESP-IDF's TWAI driver has no live mode-change call -- ported behavior
 * from JC-ESP32P4-M3's can_mode_http_handler (mcp2515_set_listen_only
 * there does the equivalent for that board's controller). */
static void can_set_mode(bool passive)
{
    twai_stop();
    twai_driver_uninstall();
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
            CAN_TX_GPIO, CAN_RX_GPIO, passive ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "can_set_mode: twai_driver_install failed: %s", esp_err_to_name(err));
        return;
    }
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "can_set_mode: twai_start failed: %s", esp_err_to_name(err));
        return;
    }
    s_can_passive = passive;
    if (!passive) {
        xQueueReset(s_obd_response_queue);
    }
    ESP_LOGI(TAG, "CAN mode changed to %s", passive ? "passive" : "active");
}

static void start_can(void)
{
    s_can_capture = heap_caps_calloc(CAN_CAPTURE_CAPACITY, sizeof(can_capture_frame_t), MALLOC_CAP_SPIRAM);
    if (s_can_capture == NULL) {
        ESP_LOGE(TAG, "CAN: failed to allocate %d-frame capture buffer in PSRAM", CAN_CAPTURE_CAPACITY);
        return;
    }
    s_obd_response_queue = xQueueCreate(16, sizeof(obd_frame_t));
    s_uds_response_queue = xQueueCreate(4, sizeof(obd_frame_t));
    s_identify_semaphore = xSemaphoreCreateBinary();
    if (s_obd_response_queue == NULL || s_uds_response_queue == NULL || s_identify_semaphore == NULL) {
        ESP_LOGE(TAG, "CAN: failed to allocate OBD/UDS queues or semaphore");
        return;
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
            CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(err));
        return;
    }
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "CAN ready: TX=GPIO%d RX=GPIO%d 500kbps, free heap=%lu (internal=%lu)",
             CAN_TX_GPIO, CAN_RX_GPIO, (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    /* 2026-09-18 morning: after adding GPS/CAN-capture/HTTP, one boot showed
     * "CAN ready" print but then NEVER logged a single can_receive_task
     * line again (not even a "no frames" timeout, which normally fires
     * every ~1s) -- looks like this xTaskCreate silently failed (returned
     * pdFAIL, task never actually runs) rather than a wiring problem, since
     * a genuinely running receive task logs *something* every second
     * regardless of what's on the wire. Checking the return value and
     * free heap here to catch that precisely instead of guessing. */
    BaseType_t created = xTaskCreatePinnedToCore(can_receive_task, "can_rx", 4096, NULL, 5, NULL, 1);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "CAN: xTaskCreate FAILED (result=%d) -- can_receive_task never started. "
                 "Free heap=%lu internal=%lu", (int)created,
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    }
    xTaskCreatePinnedToCore(obd_query_task, "obd_query", 4096, NULL, 5, NULL, 1);
    /* Picks back up an interrupted deep scan (most likely a brownout reset
     * mid-sweep) -- see deep_scan_resume_from_nvs's doc comment. Safe to
     * call even if none was ever started. */
    deep_scan_resume_from_nvs();
}

/* Active Mode 01 PID polling -- ported from JC-ESP32P4-M3's obd_query_task,
 * trimmed to just what's essential here: the deep-scan trigger and the PID
 * loop with its multi-ECU responder-lock disambiguation (a real car,
 * unlike the simulator's single ECU, can have more than one module answer
 * a functional broadcast -- see the doc comment below for why that
 * matters). DTC/VIN/address-scan/VWTP triggers from the original aren't
 * ported (not needed for this board yet). Passive mode still gets live
 * data for free via route_can_frame's unsolicited-broadcast decode path
 * (decode_engine_broadcast/decode_vehicle_broadcast, or obd_print_response
 * on the simulator's own unsolicited PID broadcasts) -- this task simply
 * skips actually transmitting while passive. */
static void obd_query_task(void *arg)
{
    (void)arg;
    static const uint8_t pids[] = { 0x00, 0x05, 0x0C, 0x0D, 0x11, 0x20 };
    uint8_t consecutive_timeouts = 0;
    /* Which physical ECU's responses to trust, once one has answered -- see
     * JC-ESP32P4-M3's obd_query_task doc comment: a real car can have more
     * than one module willing to answer a functional-broadcast Mode 01
     * request, and only the first one to answer should keep being trusted
     * for the rest of this PID sequence. 0 means "not locked yet". */
    uint32_t responder_id = 0;

    while (1) {
        obd_query_deep_scan_if_requested();

        for (size_t i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
            if (s_can_passive) {
                vTaskDelay(pdMS_TO_TICKS(OBD_QUERY_INTERVAL_MS));
                continue;
            }

            uint8_t request[8] = { 0x02, OBD_MODE_CURRENT_DATA, pids[i], 0, 0, 0, 0, 0 };
            s_obd_request_pending = true;
            obd_send_request(request, sizeof(request));

            obd_frame_t response;
            bool got = xQueueReceive(s_obd_response_queue, &response, pdMS_TO_TICKS(OBD_RESPONSE_TIMEOUT_MS));
            s_obd_request_pending = false;
            bool id_ok = (responder_id == 0) || (response.id == responder_id);
            if (got && response.dlc >= 3 && response.data[2] == pids[i] && id_ok) {
                if (responder_id == 0) {
                    responder_id = response.id;
                    ESP_LOGI(TAG, "OBD responder locked to id 0x%lx", (unsigned long)responder_id);
                }
                obd_print_response(pids[i], response.data, response.dlc);
                consecutive_timeouts = 0;
            } else if (got && response.dlc >= 3 && response.data[2] == pids[i]) {
                ESP_LOGW(TAG, "OBD PID 0x%02x -> answered by id 0x%lx, not the locked responder 0x%lx; discarding",
                         pids[i], (unsigned long)response.id, (unsigned long)responder_id);
                consecutive_timeouts = 0;
            } else if (got) {
                ESP_LOGW(TAG, "OBD PID 0x%02x -> mismatched response (got pid 0x%02x), discarding",
                         pids[i], response.data[2]);
                consecutive_timeouts = 0;
            } else {
                if (++consecutive_timeouts >= OBD_ADDR_RESET_AFTER_TIMEOUTS) {
                    consecutive_timeouts = 0;
                    responder_id = 0;
                    portENTER_CRITICAL(&s_obd_addressing_lock);
                    s_obd_addressing = OBD_ADDR_UNKNOWN;
                    portEXIT_CRITICAL(&s_obd_addressing_lock);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(OBD_QUERY_INTERVAL_MS));
        }
    }
}

/* CAN+GPS together used to break Wi-Fi/BLE reliability -- root-caused to
 * internal-RAM exhaustion (see s_can_capture's doc comment), not CPU
 * affinity or interrupt allocation (both tested and ruled out first).
 * Fixed by moving the CAN capture buffer to PSRAM; both drivers run fine
 * together now. */
static void sensors_init_task(void *arg)
{
    SemaphoreHandle_t done = (SemaphoreHandle_t)arg;
    start_can();
    start_gps();
    xSemaphoreGive(done);
    vTaskDelete(NULL);
}

static void sensors_init_on_core1(void)
{
    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(sensors_init_task, "sensors_init", 4096, done, 5, NULL, 1);
    xSemaphoreTake(done, portMAX_DELAY);
    vSemaphoreDelete(done);
}

void app_main(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "JC-ESP32S3-CAN bring-up: cores=%d rev=v%d.%d flash=%luMB psram=%s",
             chip_info.cores, chip_info.revision / 100, chip_info.revision % 100,
             (unsigned long)(flash_size / (1024 * 1024)),
#if CONFIG_SPIRAM
             "enabled"
#else
             "NOT enabled (check sdkconfig)"
#endif
    );

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO_CANDIDATE,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip init on GPIO%d failed: %s", LED_GPIO_CANDIDATE, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "led_strip init OK on GPIO%d", LED_GPIO_CANDIDATE);
    }

    /* Runs start_can()/start_gps() from a task pinned to CPU1 rather than
     * directly here (app_main() runs pinned to CPU0, same as WiFi/BLE --
     * CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0/CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0/
     * CONFIG_BT_NIMBLE_PINNED_TO_CORE_0). This turned out NOT to be why
     * CAN+GPS together broke Wi-Fi/BLE (that was internal-RAM exhaustion,
     * see s_can_capture's doc comment) -- kept anyway since freeing CPU0
     * for WiFi/BLE is sound practice regardless and costs nothing. */
    sensors_init_on_core1();
    start_wifi_ap();
    start_ble();

    ESP_LOGI(TAG, "=== post-init heap: free=%lu internal=%lu internal_8bit=%lu largest_internal_block=%lu ===",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    ESP_LOGI(TAG, "Blinking LED at the app-controlled rate (default %lu ms half-period) -- "
             "connect with the CarTheftGuard app and adjust it from the Control tab.",
             (unsigned long)s_blink_half_period_ms);

    bool on = false;
    while (1) {
        if (s_led != NULL) {
            if (on) {
                led_strip_set_pixel(s_led, 0, 24, 24, 24);
            } else {
                led_strip_clear(s_led);
            }
            led_strip_refresh(s_led);
        }
        on = !on;
        vTaskDelay(pdMS_TO_TICKS(s_blink_half_period_ms));
    }
}
