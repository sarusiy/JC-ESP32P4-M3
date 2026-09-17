#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_hosted.h"
#include "esp_freertos_hooks.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED)
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#endif
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_default_config.h"
#include "mcp2515.h"

#ifndef LED_GPIO
#define LED_GPIO 28
#endif

/* MCP2515 CAN module wiring on header JP1 (see Doc/electrical drawing.png). */
#ifndef CAN_CS_GPIO
#define CAN_CS_GPIO 49
#endif
#ifndef CAN_SCK_GPIO
#define CAN_SCK_GPIO 50
#endif
#ifndef CAN_MOSI_GPIO
#define CAN_MOSI_GPIO 51
#endif
#ifndef CAN_MISO_GPIO
#define CAN_MISO_GPIO 52
#endif
#ifndef CAN_INT_GPIO
#define CAN_INT_GPIO 29
#endif
#define CAN_TEST_ID 0x100

/* Simulator-only IDs modeling a vehicle's internal (non-diagnostic) CAN bus:
 * unsolicited, always-broadcasting, proprietary encodings -- unlike the
 * standardized OBD-II request/response pair above. Real vehicles broadcast
 * this kind of data continuously too, but on manufacturer-specific IDs with
 * manufacturer-specific encodings that differ per make/model; these exact
 * IDs and byte layouts will NOT mean anything on a real car's bus. */
#define CAN_ID_ENGINE_STATE 0x120
#define CAN_ID_VEHICLE_STATE 0x180

/* Simulated GPS UART wiring on JP1 (see Doc/GPS_SIMULATION_AND_INTEGRATION_REQUIREMENTS.md).
 * GPIO34/35 are free on the same header as the MCP2515 wiring above. The
 * Arduino simulator's SoftwareSerial TX is 5V logic and is stepped down to
 * GPIO34 with a 10k/15k divider, matching the CAN SO/INT dividers. */
#ifndef GPS_UART_RX_GPIO
#define GPS_UART_RX_GPIO 34
#endif
#ifndef GPS_UART_TX_GPIO
#define GPS_UART_TX_GPIO 35
#endif
#define GPS_UART_NUM UART_NUM_1
#define GPS_UART_BAUD 9600

/* Quarter-dBm units: 40 = 10 dBm. Reduces Wi-Fi current spikes on USB-powered bench setups. */
#define WIFI_MAX_TX_POWER_QDBM 40

/* OBD-II (SAE J1979) scan-tool request/response IDs, matching the simulated
 * ECU in the ArdunioUsbBridgeToCan repo. OBD_RESPONSE_ID is kept as the
 * primary/engine ECU's ID (used for Mode 01 PID matching, which only that
 * ECU answers here); OBD_RESPONSE_ID_MIN/MAX cover the full ISO 15765-4
 * physical response range (0x7E8-0x7EF, ECUs 1-8) for Mode 03/04, since a
 * real car's functional broadcast request can draw replies from multiple
 * modules (engine, transmission, ABS, ...), not just one. */
#define OBD_REQUEST_ID 0x7DF
#define OBD_RESPONSE_ID 0x7E8
#define OBD_RESPONSE_ID_MIN 0x7E8
#define OBD_RESPONSE_ID_MAX 0x7EF

/* Some vehicles (confirmed: Fiat 500, 2007-2015 "Type 312" platform) use
 * 29-bit extended CAN IDs for OBD-II instead of the 11-bit scheme above --
 * still classic CAN 2.0B (NOT CAN FD), just a longer identifier field. Every
 * query is sent on BOTH schemes; whichever one the vehicle doesn't use
 * simply times out unanswered, same as any other non-response. */
#define OBD_REQUEST_ID_EXT 0x18DB33F1UL
#define OBD_RESPONSE_ID_EXT_MIN 0x18DAF100UL
#define OBD_RESPONSE_ID_EXT_MAX 0x18DAF1FFUL

/* Private CAN IDs (outside any real OBD-II range, mirrors FAULT_INJECT_CAN_ID
 * below) understood only by the ArdunioUsbBridgeToCan bench simulator: lets
 * this firmware tell whether it's talking to that simulator or a real
 * vehicle, and lets the phone app remote-control which OBD-II addressing
 * scheme the simulator currently answers on, to exercise real-tool-style
 * detection logic on the bench against both schemes without needing two
 * different real cars. */
#define SIM_IDENTIFY_CAN_ID    0x702
#define SIM_MODE_SWITCH_CAN_ID 0x703
#define SIM_IDENTIFY_MAGIC     0xA5
#define SIM_IDENTIFY_TIMEOUT_MS 250
/* The identify ping is a one-off, low-priority message competing with a lot
 * of continuous traffic (OBD queries, the periodic broadcasts) -- on a busy
 * bus it, or its reply, can occasionally get lost to contention alone, with
 * no simulator/real-car ambiguity involved. A single failed attempt used to
 * be read as "no simulator -> must be a real car", which was wrong; retrying
 * a few times before concluding that fixes it, since re-sending compensates
 * for transient loss in a way a longer single wait would not. */
#define SIM_IDENTIFY_ATTEMPTS 3
#define OBD_MODE_CURRENT_DATA 0x01
#define OBD_MODE_REQUEST_DTC 0x03
#define OBD_MODE_CLEAR_DTC 0x04
#define OBD_QUERY_INTERVAL_MS 200
#define OBD_RESPONSE_TIMEOUT_MS 500
/* Window to collect DTC/clear responses from potentially several ECUs
 * answering one functional broadcast request, not just the first reply. */
#define OBD_MULTI_ECU_WINDOW_MS 800

/* Private/test CAN ID (outside the standard OBD range) used to tell the
 * ArdunioUsbBridgeToCan simulator to set a fault, normally in response to
 * the "Simulate Fault" button in the phone app. Real OBD-II has no "set a
 * DTC" mode, so this is a vendor-private extension, not part of SAE J1979. */
#define FAULT_INJECT_CAN_ID 0x701
#define FAULT_CODE_COUNT 5
/* Aggregate cap across all responding ECUs and all frames of a multi-frame
 * (ISO-TP) response each -- not "per CAN frame" like before. */
#define DTC_MAX_ACTIVE 16

/* ISO 15765-2 (ISO-TP) protocol control information (PCI) nibble values,
 * used to reassemble a Mode 03 DTC-list response spanning more than one CAN
 * frame (needed once a single ECU reports more than ~3 DTCs). */
#define ISOTP_PCI_SF 0x0 /* Single Frame: whole message fits in one CAN frame */
#define ISOTP_PCI_FF 0x1 /* First Frame: starts a multi-frame message */
#define ISOTP_PCI_CF 0x2 /* Consecutive Frame: continuation of a First Frame */
#define ISOTP_PCI_FC 0x3 /* Flow Control: receiver telling sender to continue */
#define ISOTP_MAX_PAYLOAD 32 /* mode byte + up to 15 DTCs -- generous for one ECU */

#define BLINK_HALF_PERIOD_MIN_MS 10
#define BLINK_HALF_PERIOD_MAX_MS 60000

/* Half-period of the blink, in ms; changed live from the control channel. */
static volatile uint32_t blink_half_period_ms = 500;
static httpd_handle_t http_server;

/* Always-on SoftAP, AP-only (no STA/home-Wi-Fi support at all) -- the phone
 * always reaches the board the same way, anywhere, with zero setup: join
 * this network. Fixed IP from ESP-IDF's default AP netif is 192.168.4.1. */
#define WIFI_AP_SSID "CarTheftGuard-P4"
#define WIFI_AP_PASSWORD "&Car1310"
#define WIFI_AP_IP "192.168.4.1"

static const char *TAG = "main";
/* Runtime log verbosity, changed live via the "ll" control-channel command. */
static esp_log_level_t s_log_level = ESP_LOG_INFO;

typedef struct {
    uint32_t supported_pids;
    /* PIDs 0x21-0x40's own support bitmask (Mode 01 PID 0x20's response) --
     * PID 0x00's bitmask has its top bit-set-at-0x20 convention meaning
     * "there's a further page", which this car's does (see
     * PID_SUPPORT_ANALYSIS.md) -- queried but not yet decoded into specific
     * named fields since which of 0x21-0x40 this car actually supports
     * hasn't been confirmed yet; raw value exposed via the API so it can be
     * inspected/decoded next time a real car is connected. */
    uint32_t supported_pids_2;
    int coolant_c;
    uint16_t rpm;
    uint8_t speed_kmh;
    uint8_t throttle_pct;
} obd_state_t;

static obd_state_t s_obd_state;
/* Written from two tasks now: can_echo_task (passive decode of engine/vehicle/
 * OBD broadcasts, and now also unsolicited internal-bus broadcasts) and
 * obd_query_task (active-mode Mode 01 request/response), read from the HTTP
 * handler task -- needs a lock like health_state_t/gps_state_t below. */
static portMUX_TYPE s_obd_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    uint8_t count;
    uint16_t codes[DTC_MAX_ACTIVE]; /* each packed as (byte_high << 8) | byte_low, SAE J2012 */
} dtc_state_t;

static dtc_state_t s_dtc_state;
static portMUX_TYPE s_dtc_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_dtc_clear_requested;

/* Mode 09 PID 0x02 (VIN) is a one-off read, not something worth polling --
 * unlike engine PIDs the VIN never changes, so it's requested on demand via
 * POST /api/obd/vin and cached here for GET /api/obd/vin to return. */
typedef struct {
    char vin[18]; /* 17 VIN characters + null terminator */
    const char *status; /* "none" | "ok" | "timeout" | "error" (passive mode) */
} vin_state_t;

static vin_state_t s_vin_state = { .vin = "", .status = "none" };
static portMUX_TYPE s_vin_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_vin_requested;

/* Firmware OTA: the phone app pushes a new image over the board's own
 * Wi-Fi link (POST /api/ota) instead of needing a USB/PC visit for every
 * fix -- see partitions.csv for the two-app-slot layout this needs.
 * ota_http_handler runs on the httpd server's single worker task, so this
 * plain (non-atomic) state is safe without a lock the same way s_ble_response
 * and other handler-local state elsewhere in this file are. */
typedef enum {
    OTA_STATE_IDLE,
    OTA_STATE_RECEIVING,
    OTA_STATE_SUCCESS,
    OTA_STATE_FAILED,
} ota_state_t;

static ota_state_t s_ota_state = OTA_STATE_IDLE;
static size_t s_ota_bytes_written;
/* Generous headroom under partitions.csv's 0x180000-byte OTA slots -- rejects
 * an oversized upload up front instead of discovering the overflow mid-write. */
#define OTA_MAX_IMAGE_SIZE (0x180000 - 0x1000)

typedef struct {
    uint16_t raw;
    const char *description;
} dtc_description_t;

/* Generic (SAE J2012-standardized) powertrain DTCs -- these P0xxx codes mean
 * the same thing on every manufacturer's vehicle, unlike P1xxx+/manufacturer-
 * specific codes, which aren't publicly standardized and can't be looked up
 * without that maker's own documentation (those still decode to a correct
 * "P1xxx"-style string via dtc_code_to_string, just with no description
 * here -- see dtc_lookup_description's "Unknown fault" fallback). Includes
 * the 5 codes ArdunioUsbBridgeToCan's simulator can inject. */
static const dtc_description_t DTC_DESCRIPTIONS[] = {
    { 0x0100, "Mass or Volume Air Flow Circuit Malfunction" },
    { 0x0101, "Mass or Volume Air Flow Circuit Range/Performance Problem" },
    { 0x0102, "Mass or Volume Air Flow Circuit Low Input" },
    { 0x0103, "Mass or Volume Air Flow Circuit High Input" },
    { 0x0106, "Manifold Absolute Pressure/Barometric Pressure Circuit Range/Performance" },
    { 0x0107, "Manifold Absolute Pressure/Barometric Pressure Circuit Low Input" },
    { 0x0108, "Manifold Absolute Pressure/Barometric Pressure Circuit High Input" },
    { 0x0110, "Intake Air Temperature Circuit Malfunction" },
    { 0x0111, "Intake Air Temperature Circuit Range/Performance Problem" },
    { 0x0112, "Intake Air Temperature Circuit Low Input" },
    { 0x0113, "Intake Air Temperature Circuit High Input" },
    { 0x0115, "Engine Coolant Temperature Circuit Malfunction" },
    { 0x0116, "Engine Coolant Temperature Circuit Range/Performance Problem" },
    { 0x0117, "Engine Coolant Temperature Circuit Low Input" },
    { 0x0118, "Engine Coolant Temperature Circuit High Input" },
    { 0x0120, "Throttle/Pedal Position Sensor A Circuit Malfunction" },
    { 0x0121, "Throttle/Pedal Position Sensor A Circuit Range/Performance Problem" },
    { 0x0122, "Throttle/Pedal Position Sensor A Circuit Low Input" },
    { 0x0123, "Throttle/Pedal Position Sensor A Circuit High Input" },
    { 0x0125, "Insufficient Coolant Temperature for Closed Loop Fuel Control" },
    { 0x0128, "Coolant Thermostat (Below Regulating Temperature)" },
    { 0x0130, "O2 Sensor Circuit Malfunction (Bank 1 Sensor 1)" },
    { 0x0131, "O2 Sensor Circuit Low Voltage (Bank 1 Sensor 1)" },
    { 0x0132, "O2 Sensor Circuit High Voltage (Bank 1 Sensor 1)" },
    { 0x0133, "O2 Sensor Circuit Slow Response (Bank 1 Sensor 1)" },
    { 0x0134, "O2 Sensor Circuit No Activity Detected (Bank 1 Sensor 1)" },
    { 0x0135, "O2 Sensor Heater Circuit Malfunction (Bank 1 Sensor 1)" },
    { 0x0136, "O2 Sensor Circuit Malfunction (Bank 1 Sensor 2)" },
    { 0x0141, "O2 Sensor Heater Circuit Malfunction (Bank 1 Sensor 2)" },
    { 0x0170, "Fuel Trim Malfunction (Bank 1)" },
    { 0x0171, "System Too Lean (Bank 1)" },
    { 0x0172, "System Too Rich (Bank 1)" },
    { 0x0173, "Fuel Trim Malfunction (Bank 2)" },
    { 0x0174, "System Too Lean (Bank 2)" },
    { 0x0175, "System Too Rich (Bank 2)" },
    { 0x0190, "Fuel Rail Pressure Sensor Circuit Malfunction" },
    { 0x0201, "Injector Circuit Malfunction - Cylinder 1" },
    { 0x0202, "Injector Circuit Malfunction - Cylinder 2" },
    { 0x0203, "Injector Circuit Malfunction - Cylinder 3" },
    { 0x0204, "Injector Circuit Malfunction - Cylinder 4" },
    { 0x0217, "Engine Overtemperature Condition" },
    { 0x0219, "Engine Overspeed Condition" },
    { 0x0230, "Fuel Pump Primary Circuit Malfunction" },
    { 0x0234, "Engine Overboost Condition" },
    { 0x0300, "Random/Multiple Cylinder Misfire Detected" },
    { 0x0301, "Cylinder 1 Misfire Detected" },
    { 0x0302, "Cylinder 2 Misfire Detected" },
    { 0x0303, "Cylinder 3 Misfire Detected" },
    { 0x0304, "Cylinder 4 Misfire Detected" },
    { 0x0305, "Cylinder 5 Misfire Detected" },
    { 0x0306, "Cylinder 6 Misfire Detected" },
    { 0x0325, "Knock Sensor 1 Circuit Malfunction" },
    { 0x0335, "Crankshaft Position Sensor A Circuit Malfunction" },
    { 0x0336, "Crankshaft Position Sensor A Circuit Range/Performance" },
    { 0x0340, "Camshaft Position Sensor A Circuit Malfunction" },
    { 0x0341, "Camshaft Position Sensor A Circuit Range/Performance" },
    { 0x0351, "Ignition Coil A Primary/Secondary Circuit Malfunction" },
    { 0x0352, "Ignition Coil B Primary/Secondary Circuit Malfunction" },
    { 0x0401, "Exhaust Gas Recirculation Flow Insufficient Detected" },
    { 0x0402, "Exhaust Gas Recirculation Flow Excessive Detected" },
    { 0x0403, "Exhaust Gas Recirculation Circuit Malfunction" },
    { 0x0410, "Secondary Air Injection System Malfunction" },
    { 0x0420, "Catalyst System Efficiency Below Threshold (Bank 1)" },
    { 0x0430, "Catalyst System Efficiency Below Threshold (Bank 2)" },
    { 0x0440, "Evaporative Emission Control System Malfunction" },
    { 0x0441, "Evaporative Emission Control System Incorrect Purge Flow" },
    { 0x0442, "EVAP Emission Control System Leak Detected (small leak)" },
    { 0x0443, "Evaporative Emission Control System Purge Control Valve Circuit Malfunction" },
    { 0x0446, "Evaporative Emission Control System Vent Control Circuit Malfunction" },
    { 0x0455, "Evaporative Emission Control System Leak Detected (Large Leak)" },
    { 0x0456, "Evaporative Emission Control System Leak Detected (Very Small Leak)" },
    { 0x0460, "Fuel Level Sensor Circuit Malfunction" },
    { 0x0480, "Cooling Fan 1 Control Circuit Malfunction" },
    { 0x0500, "Vehicle Speed Sensor Malfunction" },
    { 0x0505, "Idle Control System Malfunction" },
    { 0x0506, "Idle Control System RPM Lower Than Expected" },
    { 0x0507, "Idle Control System RPM Higher Than Expected" },
    { 0x0520, "Engine Oil Pressure Sensor/Switch Circuit Malfunction" },
    { 0x0562, "System Voltage Low" },
    { 0x0563, "System Voltage High" },
    { 0x0600, "Serial Communication Link Malfunction" },
    { 0x0601, "Internal Control Module Memory Check Sum Error" },
    { 0x0605, "Internal Control Module Read Only Memory (ROM) Error" },
    { 0x0620, "Generator Control Circuit Malfunction" },
    { 0x0630, "VIN Not Programmed or Incompatible - ECM" },
    { 0x0700, "Transmission Control System Malfunction (MIL Request)" },
    { 0x0701, "Transmission Control System Range/Performance" },
    { 0x0705, "Transmission Range Sensor Circuit Malfunction" },
    { 0x0706, "Transmission Range Sensor Circuit Range/Performance" },
    { 0x0710, "Transmission Fluid Temperature Sensor Circuit Malfunction" },
    { 0x0715, "Input/Turbine Speed Sensor Circuit Malfunction" },
    { 0x0720, "Output Speed Sensor Circuit Malfunction" },
    { 0x0730, "Incorrect Gear Ratio" },
    { 0x0740, "Torque Converter Clutch Circuit Malfunction" },
    { 0x0750, "Shift Solenoid A Malfunction" },
    { 0x0755, "Shift Solenoid B Malfunction" },
    { 0x0760, "Shift Solenoid C Malfunction" },
};

/* Decodes a raw 2-byte DTC per SAE J2012 into a "P0301"-style string. */
static void dtc_code_to_string(uint16_t raw, char *out, size_t out_len)
{
    static const char categories[4] = { 'P', 'C', 'B', 'U' };
    uint8_t hi = (raw >> 8) & 0xFF;
    uint8_t lo = raw & 0xFF;
    char category = categories[(hi >> 6) & 0x03];
    uint8_t digit1 = (hi >> 4) & 0x03;
    uint8_t digit2 = hi & 0x0F;
    uint8_t digit3 = (lo >> 4) & 0x0F;
    uint8_t digit4 = lo & 0x0F;
    snprintf(out, out_len, "%c%01X%01X%01X%01X", category, digit1, digit2, digit3, digit4);
}

static const char *dtc_lookup_description(uint16_t raw)
{
    for (size_t i = 0; i < sizeof(DTC_DESCRIPTIONS) / sizeof(DTC_DESCRIPTIONS[0]); i++) {
        if (DTC_DESCRIPTIONS[i].raw == raw) {
            return DTC_DESCRIPTIONS[i].description;
        }
    }
    return "Unknown fault";
}

typedef struct {
    uint32_t idle0_pct;
    uint32_t idle1_pct;
    uint32_t idle0_delta;
    uint32_t idle1_delta;
    uint32_t idle0_max_delta;
    uint32_t idle1_max_delta;
    size_t heap_free;
    size_t heap_min_free;
    size_t internal_free;
    size_t internal_min_free;
    size_t psram_free;
    size_t psram_min_free;
    uint32_t flash_size;
    uint32_t flash_partitioned;
    uint32_t app_partition_size;
    uint32_t uptime_s;
    esp_reset_reason_t restart_reason;
} health_state_t;

static health_state_t s_health_state;
static portMUX_TYPE s_health_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_idle0_count;
static volatile uint32_t s_idle1_count;

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

typedef struct {
    uint32_t id;  /* source CAN ID: which ECU (0x7E8-0x7EF) this response came from */
    uint8_t dlc;
    uint8_t data[8];
} obd_frame_t;

typedef struct {
    uint64_t sequence;
    int64_t timestamp_us;
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_capture_frame_t;

/* Sized for a real, busy vehicle CAN bus, not just this simulator's quiet
 * two-node bus -- a real powertrain bus can run into the hundreds/low
 * thousands of frames/sec across many ECUs. CAN_CAPTURE_CAPACITY is the ring
 * buffer's total burst-absorption headroom (4096 * ~32 bytes/frame = ~128KB,
 * trivial against this board's SRAM budget); CAN_CAPTURE_HTTP_BATCH bounds
 * both how many frames can be drained per can_echo_task wake-up and how many
 * one GET /api/can response returns, so raising it also raises the
 * sustained drain rate the phone app can keep up with (at its ~100ms poll
 * interval, 128/poll ~= 1280 frames/sec sustained, vs. 320/sec before). */
#define CAN_CAPTURE_CAPACITY 4096
#define CAN_CAPTURE_HTTP_BATCH 128
#define CAN_CAPTURE_RESPONSE_SIZE 24576
#define TASKS_RESPONSE_SIZE 8192

static can_capture_frame_t s_can_capture[CAN_CAPTURE_CAPACITY];
static uint64_t s_can_capture_sequence;
static portMUX_TYPE s_can_capture_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_can_rx_task;
static volatile bool s_can_passive = false;

/* True only for the brief window between sending a Mode 01/03/04 request and
 * receiving/timing out its reply. The Arduino simulator also broadcasts PID
 * data unsolicited every 250ms regardless of mode; without this flag every
 * such broadcast arriving between polls would get misrouted into
 * s_obd_response_queue as if it were the answer to our last request,
 * desyncing every request after it (see can_echo_task below). */
static volatile bool s_obd_request_pending;

/* Filled by can_echo_task whenever it sees an 0x7E8 response, drained by obd_query_task. */
static QueueHandle_t s_obd_response_queue;

/* UDS DID-sweep discovery tool -- see captures/fabia_2026/UDS_BODY_MODULE_RESEARCH.md.
 * Unlike Mode 01's functional broadcast (one shared request ID, a range of
 * possible response IDs, responder-lock to pick one), UDS body-module
 * requests are point-to-point: one exact request ID, one exact response ID,
 * known in advance -- no ambiguity to resolve, but also nothing in the
 * existing OBD response routing recognizes these IDs at all. Routed via
 * its own queue/pending-flag pair, exact-ID-match only, mirroring
 * s_obd_request_pending/s_obd_response_queue's role for Mode 01. */
static volatile bool s_uds_request_pending;
static volatile uint32_t s_uds_expected_response_id;
static QueueHandle_t s_uds_response_queue;

typedef enum {
    UDS_SCAN_IDLE = 0,
    UDS_SCAN_RUNNING,
    UDS_SCAN_DONE,
    UDS_SCAN_ERROR,
} uds_scan_state_t;

#define UDS_SCAN_MAX_RESULTS 64
#define UDS_SCAN_MAX_RANGE 512

typedef struct {
    uint16_t did;
    uint8_t dlc;
    uint8_t data[8]; /* raw positive-response payload (service+DID+value bytes), truncated to 8 */
} uds_scan_result_t;

/* Outcome of the one-shot Diagnostic Session Control (0x10 0x03) request
 * sent before a sweep -- see UDS_BODY_MODULE_RESEARCH.md's 2026-09-15
 * update. Recorded alongside the sweep results themselves so it's visible
 * (via the status JSON and the phone app's uds_scan_log_*.csv) whether the
 * module even acknowledged a session, independent of whether any DID in
 * the sweep got a positive response. */
typedef enum {
    UDS_SESSION_NOT_ATTEMPTED = 0,
    UDS_SESSION_POSITIVE,
    UDS_SESSION_NEGATIVE,
    UDS_SESSION_TIMEOUT,
} uds_session_state_t;

typedef struct {
    uint32_t request_id;
    uint32_t response_id;
    bool extended;
    uint16_t did_start;
    uint16_t did_end;
    uds_scan_state_t state;
    uds_session_state_t session_state;
    uint16_t current_did; /* progress indicator while running */
    uint8_t result_count;
    uds_scan_result_t results[UDS_SCAN_MAX_RESULTS];
} uds_scan_t;

static uds_scan_t s_uds_scan;
static portMUX_TYPE s_uds_scan_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_uds_scan_requested;

/* Address-discovery sweep: rather than probing DIDs on one known-address
 * module, probes *addresses* themselves with a single Diagnostic Session
 * Control request each, to find which physical modules actually answer on
 * this car -- see UDS_BODY_MODULE_RESEARCH.md's 2026-09-15 Gateway test
 * (session:positive, proving the request mechanism itself works; the
 * follow-up question is which addresses are real on this specific car,
 * not whether our code can talk UDS at all). Every candidate address pair
 * researched so far shares the same response = request + 0x6A offset, so
 * this only needs to sweep request IDs, not enumerate pairs. */
#define UDS_ADDR_SCAN_MAX_RESULTS 32
#define UDS_ADDR_SCAN_MAX_RANGE 256

typedef struct {
    uint32_t request_id;
    uds_session_state_t session_state; /* only POSITIVE/NEGATIVE stored -- a hit either way */
} uds_addr_scan_result_t;

typedef struct {
    uint32_t req_start;
    uint32_t req_end;
    uint32_t response_offset;
    bool extended;
    uds_scan_state_t state;
    uint32_t current_req; /* progress indicator while running */
    uint8_t result_count;
    uds_addr_scan_result_t results[UDS_ADDR_SCAN_MAX_RESULTS];
} uds_addr_scan_t;

static uds_addr_scan_t s_uds_addr_scan;
static portMUX_TYPE s_uds_addr_scan_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_uds_addr_scan_requested;

/* Deep scan: combines the address-discovery sweep above with an automatic
 * identification-DID sweep (ISO 14229-1 Annex F block, 0xF180-0xF1A0 --
 * "who are you": part number, serial number, system name, VIN, hw/sw
 * versions) against every address that responds, instead of requiring a
 * human to notice a hit and manually re-run a separate DID sweep against
 * it. Meant to run unattended (e.g. overnight) across the whole
 * 0x700-0x7FF range and produce a full module map in one pass -- added
 * 2026-09-17 after a manual address-scan-then-DID-sweep cycle on a single
 * found candidate (0x78F) turned out to be a dead end, prompting "why not
 * automate the whole space instead of one address at a time".
 *
 * Survives brownout resets (a real, repeatedly observed failure mode on
 * this board under sustained CAN-TX load -- see UDS_BODY_MODULE_RESEARCH.md
 * and project memory) by persisting progress to NVS as it goes and
 * resuming automatically on boot instead of losing an overnight run to a
 * single crash partway through: deep_scan_nvs_save_progress() after every
 * address, deep_scan_nvs_save_hits() immediately whenever a new module is
 * identified (not just at the very end), and deep_scan_resume_from_nvs()
 * called once at boot (see app_main) to pick the scan back up from
 * wherever it left off, with no need for the phone app to re-trigger it. */
#define DEEP_SCAN_ID_START 0xF180
#define DEEP_SCAN_ID_END 0xF1A0
#define DEEP_SCAN_MAX_HITS 8
#define DEEP_SCAN_MAX_DIDS_PER_HIT 16
#define DEEP_SCAN_NVS_NAMESPACE "deepscan"

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
    uint32_t current_req; /* progress indicator while running, and the NVS-persisted resume point */
    uint8_t hit_count;
    deep_scan_hit_t hits[DEEP_SCAN_MAX_HITS];
} deep_scan_t;

static deep_scan_t s_deep_scan;
static portMUX_TYPE s_deep_scan_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_deep_scan_requested;

/* VWTP 2.0 (VW Transport Protocol) connection-setup probe -- a completely
 * different addressing scheme from the UDS address-discovery sweep above.
 * Candidates here are single-byte logical/component addresses (the same
 * numbers VCDS shows, e.g. 0x46=Central Convenience, 0x42=Door Electronics
 * Driver), not raw CAN IDs. Sourced 2026-09-16 from PyVCDS's vwtp.py (see
 * UDS_BODY_MODULE_RESEARCH.md): the connection request always goes to CAN
 * ID 0x200; the response comes back on 0x200 + the logical address byte,
 * carrying a negotiated TX CAN ID if accepted. This exists because the
 * full 256-address direct UDS sweep found only the Gateway reachable --
 * body modules on this platform may only be reachable via this older,
 * session-based scheme instead of direct point-to-point CAN addressing. */
#define VWTP_CONNECT_REQUEST_ID 0x200
#define VWTP_SCAN_MAX_RESULTS 32
#define VWTP_SCAN_MAX_RANGE 256

typedef struct {
    uint8_t addr;
    bool positive;
    uint16_t tx_id;
} vwtp_scan_result_t;

typedef struct {
    uint8_t addr_start;
    uint8_t addr_end;
    uint16_t rx_id;
    uds_scan_state_t state;
    uint8_t current_addr;
    uint8_t result_count;
    vwtp_scan_result_t results[VWTP_SCAN_MAX_RESULTS];
} vwtp_scan_t;

static vwtp_scan_t s_vwtp_scan;
static portMUX_TYPE s_vwtp_scan_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_vwtp_scan_requested;

/* Which OBD-II addressing scheme has actually gotten a response so far.
 * UNKNOWN means "haven't heard back on either scheme yet" -- obd_send_request
 * probes both in that state; once one responds, every later query sticks to
 * just that scheme (matching how a real scan tool detects-once-then-commits,
 * instead of forever paying the cost of asking both ways). Reset to UNKNOWN
 * after a run of consecutive timeouts (the vehicle/simulator may have
 * changed) or right after a simulator mode-switch request. */
typedef enum {
    OBD_ADDR_UNKNOWN = 0,
    OBD_ADDR_STANDARD,
    OBD_ADDR_EXTENDED,
} obd_addressing_t;
static volatile obd_addressing_t s_obd_addressing = OBD_ADDR_UNKNOWN;
static portMUX_TYPE s_obd_addressing_lock = portMUX_INITIALIZER_UNLOCKED;
#define OBD_ADDR_RESET_AFTER_TIMEOUTS 5

/* Combines "who's on the other end of the bus" with which scheme they use,
 * for GET /api/can/partner. Determined by obd_identify_partner() pinging
 * SIM_IDENTIFY_CAN_ID: a response means the bench simulator (which also
 * reports its own current mode in the reply); no response, combined with
 * whatever s_obd_addressing has locked onto from real OBD traffic, means an
 * actual vehicle using that scheme (or UNKNOWN if neither has been seen). */
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

static void capture_can_frame(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    int64_t timestamp_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_can_capture_lock);
    uint64_t sequence = ++s_can_capture_sequence;
    can_capture_frame_t *frame = &s_can_capture[(sequence - 1) % CAN_CAPTURE_CAPACITY];
    frame->sequence = sequence;
    frame->timestamp_us = timestamp_us;
    frame->id = id;
    frame->dlc = dlc;
    memcpy(frame->data, data, dlc);
    portEXIT_CRITICAL(&s_can_capture_lock);
}

/* Counters used to verify GPIO29 wiring: isr_count only increments on a real
 * falling edge; timeout_count increments when the 100ms fallback fires instead. */
static volatile uint32_t s_can_isr_count;
static volatile uint32_t s_can_timeout_count;

static void IRAM_ATTR can_int_isr(void *arg)
{
    (void)arg;
    s_can_isr_count++;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_can_rx_task != NULL) {
        vTaskNotifyGiveFromISR(s_can_rx_task, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED)
static void ble_publish_response(uint16_t conn_handle, const char *message);

/* The board is AP-only (no home Wi-Fi/STA support) -- its IP is always the
 * fixed WIFI_AP_IP constant, never anything learned dynamically. This just
 * tracks whether the AP is up yet, so ble_gap_event() knows whether it's
 * safe to resend "WiFi connected ip=..." the moment a central (re)subscribes
 * (see BLE_GAP_EVENT_SUBSCRIBE handling below). */
static bool s_wifi_ready;
#endif

/* Shared command parser used by USB CDC and future companion transport. */
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

    blink_half_period_ms = (uint32_t)value;
    printf("Blink half-period set to %ld ms\n", value);
    snprintf(out_msg, out_msg_len, "OK freq=%ld ms\r\n", value);
    return true;
}

static const char *log_level_to_string(esp_log_level_t level)
{
    switch (level) {
        case ESP_LOG_ERROR: return "ERROR";
        case ESP_LOG_WARN:  return "WARNING";
        case ESP_LOG_INFO:  return "INFO";
        case ESP_LOG_DEBUG: return "DEBUG";
        default:            return "UNKNOWN";
    }
}

/* Levels are numbered 0-3 for the "ll" command: 0=DEBUG, 1=INFO, 2=WARNING, 3=ERROR. */
static bool log_level_from_number(const char *text, esp_log_level_t *out_level)
{
    static const esp_log_level_t levels[] = { ESP_LOG_DEBUG, ESP_LOG_INFO, ESP_LOG_WARN, ESP_LOG_ERROR };

    char *endptr = NULL;
    long value = strtol(text, &endptr, 10);
    if (endptr == text || *endptr != '\0' || value < 0 || value >= (long)(sizeof(levels) / sizeof(levels[0]))) {
        return false;
    }

    *out_level = levels[value];
    return true;
}

/* Shared command parser: "ll <0-3>" (0=DEBUG, 1=INFO, 2=WARNING, 3=ERROR), controls our own TAG's verbosity. */
static bool apply_ll_command(const char *line, char *out_msg, size_t out_msg_len)
{
    esp_log_level_t level;
    if (strncmp(line, "ll ", 3) != 0 || !log_level_from_number(line + 3, &level)) {
        snprintf(out_msg, out_msg_len, "ERR usage: ll <0-3> (0=DEBUG 1=INFO 2=WARNING 3=ERROR)\r\n");
        return false;
    }

    s_log_level = level;
    esp_log_level_set(TAG, level);
    snprintf(out_msg, out_msg_len, "OK ll=%s\r\n", log_level_to_string(level));
    return true;
}

static void init_led(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

/* Forward declaration: defined below, shared by both the active-mode
 * request/response path (obd_query_task) and the passive-mode broadcast
 * capture path (can_echo_task) so a single decoder feeds s_obd_state either way. */
static void obd_print_response(uint8_t pid, const uint8_t *buf, uint8_t len);
static void decode_engine_broadcast(const uint8_t *data, uint8_t dlc);
static void decode_vehicle_broadcast(const uint8_t *data, uint8_t dlc);

/* Phase 1 echo test frames get echoed back; OBD-II responses get routed to
 * obd_query_task via a queue instead of being echoed. Single task owns the
 * MCP2515 RX poll so the two consumers never race for the same frame. */
static void can_echo_task(void *arg)
{
    (void)arg;
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];

    while (1) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 0) {
            s_can_timeout_count++;
        }
        /* Once per wake-up, not once per frame -- see mcp2515_check_overflow. */
        mcp2515_check_overflow();
        /* Drain both hardware RX buffers before sleeping; otherwise a second
         * frame arriving while the first is still pending gets left behind
         * and can overflow/reorder under back-to-back multi-frame traffic.
         * Bounded to one batch's worth per wake-up: a two-node bus with a
         * passive/listen-only receiver never ACKs, so the transmitting node's
         * own CAN controller auto-retransmits every unacked frame forever,
         * which can otherwise turn this into an unbounded busy-loop that
         * starves this task's own 100ms notify cadence (and, on a real bus,
         * protects against any other pathologically busy traffic burst). */
        uint32_t drained = 0;
        while (drained < CAN_CAPTURE_HTTP_BATCH && mcp2515_receive(&id, NULL, &dlc, data)) {
            drained++;
            capture_can_frame(id, dlc, data);
            /* UDS scan response routing: exact-ID match only, checked before
             * everything else below since a UDS response ID is arbitrary
             * (whatever the target module answers on) and could in principle
             * collide with another branch's own ID check otherwise. Only
             * routed while a scan request is actually outstanding. */
            if (s_uds_request_pending && id == s_uds_expected_response_id) {
                obd_frame_t frame = { .id = id, .dlc = dlc };
                memcpy(frame.data, data, sizeof(frame.data));
                if (xQueueSend(s_uds_response_queue, &frame, 0) != pdPASS) {
                    ESP_LOGW(TAG, "UDS response queue full; dropping response");
                }
                continue;
            }
            if (id == SIM_IDENTIFY_CAN_ID) {
                if (s_identify_pending && dlc >= 2) {
                    s_identify_response[0] = data[0];
                    s_identify_response[1] = data[1];
                    xSemaphoreGive(s_identify_semaphore);
                }
                continue;
            }
            bool response_is_extended = (id >= OBD_RESPONSE_ID_EXT_MIN && id <= OBD_RESPONSE_ID_EXT_MAX);
            bool is_obd_response = (id >= OBD_RESPONSE_ID_MIN && id <= OBD_RESPONSE_ID_MAX) || response_is_extended;
            if (is_obd_response) {
                /* First real response locks in which scheme this vehicle uses,
                 * same as a real scan tool committing after its initial probe --
                 * see s_obd_addressing comment. */
                portENTER_CRITICAL(&s_obd_addressing_lock);
                if (s_obd_addressing == OBD_ADDR_UNKNOWN) {
                    s_obd_addressing = response_is_extended ? OBD_ADDR_EXTENDED : OBD_ADDR_STANDARD;
                }
                portEXIT_CRITICAL(&s_obd_addressing_lock);
                if (s_can_passive) {
                    /* Passive mode never sends the 0x7DF request itself, but if the
                     * bus carries unsolicited OBD-II broadcasts (e.g. a simulated ECU
                     * acting like a real car that reports its own PIDs), decode and
                     * expose them read-only via s_obd_state / GET /api/obd -- exactly
                     * like the active-mode request/response path below, just without
                     * ever transmitting anything onto the bus. */
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
                    /* Active mode, but no request currently in flight: one of the
                     * ECU's own unsolicited periodic broadcasts landing between our
                     * polls. Decode it passively instead of leaving it to rot in
                     * the (already-empty) response queue. */
                    obd_print_response(data[2], data, dlc);
                }
                continue;
            }
            /* Internal-bus-style broadcasts (see CAN_ID_ENGINE_STATE comment): always
             * decoded regardless of s_can_passive, exactly like a real instrument
             * cluster would -- this data isn't part of the diagnostic request/response
             * protocol, so "passive" (never transmitting) has no bearing on receiving it. */
            if (id == CAN_ID_ENGINE_STATE) {
                decode_engine_broadcast(data, dlc);
                continue;
            }
            if (id == CAN_ID_VEHICLE_STATE) {
                decode_vehicle_broadcast(data, dlc);
                continue;
            }
            if (!s_can_passive && id == CAN_TEST_ID) {
                printf("CAN RX id=0x%03lx dlc=%d data='%.*s' -> echoing\n",
                       (unsigned long)id, dlc, dlc, data);
                mcp2515_send(id, false, dlc, data);
            }
        }
    }
}

/* Decodes and prints one PID's response payload (buf[0]=len, buf[1]=mode+0x40,
 * buf[2]=pid, buf[3..]=data), per the SAE J1979 formulas for each PID. */
static void obd_print_response(uint8_t pid, const uint8_t *buf, uint8_t len)
{
    /* buf[0]=len, buf[1]=mode+0x40, buf[2]=pid echo, buf[3..]=actual parameter bytes. */
    switch (pid) {
        case 0x00:
            if (len >= 7) {
                uint32_t bitmask = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) |
                                    ((uint32_t)buf[5] << 8) | buf[6];
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.supported_pids = bitmask;
                portEXIT_CRITICAL(&s_obd_lock);
                ESP_LOGI(TAG, "OBD PID 0x00 (supported PIDs)   -> bitmask %02x %02x %02x %02x",
                         buf[3], buf[4], buf[5], buf[6]);
            }
            break;
        case 0x05:
            if (len >= 4) {
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.coolant_c = buf[3] - 40;
                portEXIT_CRITICAL(&s_obd_lock);
                ESP_LOGI(TAG, "OBD PID 0x05 (coolant temp)      -> %d C", buf[3] - 40);
            }
            break;
        case 0x0C:
            if (len >= 5) {
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.rpm = ((unsigned)buf[3] * 256 + buf[4]) / 4;
                portEXIT_CRITICAL(&s_obd_lock);
                ESP_LOGI(TAG, "OBD PID 0x0C (engine RPM)        -> %u rpm",
                         ((unsigned)buf[3] * 256 + buf[4]) / 4);
            }
            break;
        case 0x0D:
            if (len >= 4) {
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.speed_kmh = buf[3];
                portEXIT_CRITICAL(&s_obd_lock);
                ESP_LOGI(TAG, "OBD PID 0x0D (vehicle speed)     -> %u km/h", buf[3]);
            }
            break;
        case 0x11:
            if (len >= 4) {
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.throttle_pct = (buf[3] * 100u) / 255u;
                portEXIT_CRITICAL(&s_obd_lock);
                ESP_LOGI(TAG, "OBD PID 0x11 (throttle position) -> %u %%", (buf[3] * 100u) / 255u);
            }
            break;
        case 0x20:
            if (len >= 7) {
                uint32_t bitmask = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) |
                                    ((uint32_t)buf[5] << 8) | buf[6];
                portENTER_CRITICAL(&s_obd_lock);
                s_obd_state.supported_pids_2 = bitmask;
                portEXIT_CRITICAL(&s_obd_lock);
                ESP_LOGI(TAG, "OBD PID 0x20 (supported PIDs 21-40) -> bitmask %02x %02x %02x %02x",
                         buf[3], buf[4], buf[5], buf[6]);
            }
            break;
        default:
            ESP_LOGI(TAG, "OBD PID 0x%02x -> unrecognized response", pid);
            break;
    }
}

/* Decodes the simulator's engine-state broadcast (0x120, unsolicited, every
 * 20ms regardless of CAN bridge mode) into the same live state Mode 01
 * PID 0x0C/0x05/0x11 would report -- models a vehicle's internal bus, where
 * this kind of data is always flowing whether or not a diagnostic tool is
 * plugged in and asking. NOT a real vehicle's actual encoding; see the
 * CAN_ID_ENGINE_STATE comment above. */
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

/* Decodes the simulator's vehicle-state broadcast (0x180, unsolicited, every
 * 50ms) the same way -- see decode_engine_broadcast above. */
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

/* Reassembles one ISO-TP (ISO 15765-2) message starting from an already-
 * dequeued frame that begins it: either a complete Single Frame, or a First
 * Frame that needs Consecutive Frames collected after sending Flow Control.
 * Needed once a single ECU's DTC list has more than ~3 codes and no longer
 * fits in one CAN frame. Writes the reassembled payload (mode byte onward,
 * ISO-TP framing stripped) into out_payload (caller-sized ISOTP_MAX_PAYLOAD)
 * and *out_len. Returns true if at least some payload was assembled -- a
 * message truncated by a dropped/out-of-order frame still returns whatever
 * was received rather than nothing, so callers can use the partial data. */
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
        return false; /* stray Consecutive/Flow-Control frame with no First Frame -- ignore */
    }

    uint16_t total_len = ((uint16_t)(first->data[0] & 0x0F) << 8) | first->data[1];
    if (total_len > ISOTP_MAX_PAYLOAD) {
        total_len = ISOTP_MAX_PAYLOAD; /* clamp: still decode as many DTCs as fit */
    }
    uint8_t received = (total_len < 6) ? total_len : 6;
    memcpy(out_payload, &first->data[2], received);

    /* Flow Control: Continue-To-Send, block size 0 (send all remaining CFs
     * without waiting for further FC), separation time 0. Sent to this ECU's
     * physical request ID. Standard 11-bit: ISO 15765-4 fixes the pairs
     * 0x7E0<->0x7E8 ... 0x7E7<->0x7EF, i.e. response_id - 8. Extended 29-bit
     * (e.g. Fiat 500): physical addressing is 0x18DA<target><source>, so the
     * response 0x18DAF111 (target=tester 0xF1, source=ECU 0x11) pairs with
     * request 0x18DA11F1 -- swap the low two bytes instead of subtracting. */
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
    mcp2515_send(request_id, response_is_extended, sizeof(fc), fc);

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
            continue; /* a different ECU's frame interleaved -- not this stream */
        }
        uint8_t frame_pci = (frame.data[0] >> 4) & 0x0F;
        if (frame_pci != ISOTP_PCI_CF || (frame.data[0] & 0x0F) != (expected_seq & 0x0F)) {
            continue; /* out-of-order/unexpected -- keep waiting within the deadline */
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

/* Appends any DTCs found in a decoded Mode 03 payload (payload[0]=0x43,
 * payload[1..]=DTC byte pairs) into *collected, skipping duplicates in case
 * more than one ECU happens to report the same code. Returns true if this
 * was actually a Mode 03 positive response (so callers can count it as one
 * more responding ECU even if it reported zero DTCs). */
static bool dtc_collect_from_payload(const uint8_t *payload, uint8_t len, dtc_state_t *collected)
{
    if (len < 1 || payload[0] != 0x43) {
        return false;
    }
    uint8_t dtc_byte_count = len - 1;
    uint8_t dtc_count = dtc_byte_count / 2;
    for (uint8_t i = 0; i < dtc_count && collected->count < DTC_MAX_ACTIVE; i++) {
        uint16_t code = ((uint16_t)payload[1 + i * 2] << 8) | payload[2 + i * 2];
        bool duplicate = false;
        for (uint8_t j = 0; j < collected->count; j++) {
            if (collected->codes[j] == code) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            collected->codes[collected->count++] = code;
        }
    }
    return true;
}

/* Sends one OBD request. While s_obd_addressing is still UNKNOWN, probes
 * both the standard (0x7DF) and extended (0x18DB33F1) functional IDs, since
 * we don't know in advance which scheme a given vehicle uses -- see
 * OBD_REQUEST_ID_EXT comment. Once can_echo_task has locked onto whichever
 * scheme actually got a response, only that one is sent from then on,
 * matching how a real scan tool detects once then commits instead of
 * forever paying the cost of asking both ways. */
static void obd_send_request(const uint8_t *request, uint8_t len)
{
    portENTER_CRITICAL(&s_obd_addressing_lock);
    obd_addressing_t addressing = s_obd_addressing;
    portEXIT_CRITICAL(&s_obd_addressing_lock);

    if (addressing != OBD_ADDR_EXTENDED) {
        mcp2515_send(OBD_REQUEST_ID, false, len, request);
    }
    if (addressing != OBD_ADDR_STANDARD) {
        mcp2515_send(OBD_REQUEST_ID_EXT, true, len, request);
    }
}

/* Pings the bench simulator's private identify ID and waits briefly for its
 * reply, which also reports the simulator's own current addressing mode --
 * see SIM_IDENTIFY_CAN_ID comment. A response means the simulator is on the
 * bus (and directly tells us its mode, no need to separately probe/lock via
 * real OBD traffic); no response means either a real vehicle -- in which
 * case whatever s_obd_addressing has already locked onto from actual OBD
 * responses tells us which scheme it uses -- or nothing conclusive yet. */
static void obd_identify_partner(void)
{
    if (s_can_passive) {
        /* Listen-only mode physically can't transmit the identify ping (or
         * any OBD request), so nothing about the partner is knowable right
         * now. Report UNKNOWN instead of falling through to the "no reply
         * means real vehicle" inference below, which would otherwise
         * misreport a stale addressing lock as CAR_11/CAR_29 even though
         * the real reason is "we're not allowed to ask", not "a car
         * answered instead of the simulator". */
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
        mcp2515_send(SIM_IDENTIFY_CAN_ID, false, sizeof(ping), ping);
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

    /* All attempts failed -- only now infer a real vehicle from whatever
     * addressing scheme real OBD traffic has already locked onto (or
     * UNKNOWN if nothing has answered at all yet). */
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

/* Sends Mode 04 (clear DTCs) as a functional broadcast and collects 0x44 acks
 * from every ECU that answers within OBD_MULTI_ECU_WINDOW_MS, if the CAN
 * bridge is active. Called from obd_query_task so it shares the single
 * in-flight request/response slot with the rest of the OBD scan -- never
 * races the PID or DTC-scan requests below for s_obd_response_queue. */
static void obd_query_clear_dtcs_if_requested(void)
{
    if (!s_dtc_clear_requested) {
        return;
    }
    s_dtc_clear_requested = false;

    if (s_can_passive) {
        ESP_LOGW(TAG, "DTC clear requested while CAN bridge is passive; ignored");
        return;
    }

    uint8_t request[8] = { 0x01, OBD_MODE_CLEAR_DTC, 0, 0, 0, 0, 0, 0 };
    s_obd_request_pending = true;
    obd_send_request(request, sizeof(request));

    uint8_t acks = 0;
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)OBD_MULTI_ECU_WINDOW_MS * 1000);
    while (esp_timer_get_time() < deadline_us) {
        int64_t remaining_ms = (deadline_us - esp_timer_get_time()) / 1000;
        if (remaining_ms <= 0) {
            break;
        }
        obd_frame_t response;
        if (!xQueueReceive(s_obd_response_queue, &response, pdMS_TO_TICKS(remaining_ms))) {
            break;
        }
        if (response.dlc >= 2 && response.data[1] == 0x44) {
            acks++;
        }
    }
    s_obd_request_pending = false;

    if (acks > 0) {
        portENTER_CRITICAL(&s_dtc_lock);
        s_dtc_state.count = 0;
        portEXIT_CRITICAL(&s_dtc_lock);
        ESP_LOGI(TAG, "DTC clear -> acknowledged by %u ECU(s), all faults cleared", acks);
    } else {
        ESP_LOGW(TAG, "DTC clear -> no response (timeout)");
    }
}

/* Sends Mode 03 (request DTCs) as a functional broadcast and collects
 * responses -- potentially multi-frame, potentially from several ECUs -- from
 * everyone who answers within OBD_MULTI_ECU_WINDOW_MS. Same single-in-flight-
 * request rule as obd_query_clear_dtcs_if_requested above. */
static void obd_query_dtcs(void)
{
    if (s_can_passive) {
        return;
    }

    uint8_t request[8] = { 0x01, OBD_MODE_REQUEST_DTC, 0, 0, 0, 0, 0, 0 };
    s_obd_request_pending = true;
    obd_send_request(request, sizeof(request));

    dtc_state_t collected = { 0 };
    uint8_t responding_ecus = 0;
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)OBD_MULTI_ECU_WINDOW_MS * 1000);

    while (esp_timer_get_time() < deadline_us && collected.count < DTC_MAX_ACTIVE) {
        int64_t remaining_ms = (deadline_us - esp_timer_get_time()) / 1000;
        if (remaining_ms <= 0) {
            break;
        }
        obd_frame_t response;
        if (!xQueueReceive(s_obd_response_queue, &response, pdMS_TO_TICKS(remaining_ms))) {
            break;
        }

        uint8_t payload[ISOTP_MAX_PAYLOAD];
        uint8_t payload_len = 0;
        if (isotp_reassemble(response.id, &response, payload, &payload_len, deadline_us) &&
            dtc_collect_from_payload(payload, payload_len, &collected)) {
            responding_ecus++;
        }
    }
    s_obd_request_pending = false;

    portENTER_CRITICAL(&s_dtc_lock);
    s_dtc_state = collected;
    portEXIT_CRITICAL(&s_dtc_lock);

    if (collected.count > 0) {
        ESP_LOGI(TAG, "DTC scan -> %u active fault(s) across %u ECU(s)", collected.count, responding_ecus);
    } else if (responding_ecus > 0) {
        ESP_LOGI(TAG, "DTC scan -> %u ECU(s) responded, no active faults", responding_ecus);
    } else {
        ESP_LOGI(TAG, "DTC scan -> no response (timeout)");
    }
}

/* Sends Mode 09 PID 0x02 (VIN) as a functional broadcast and decodes the
 * first valid reply, if the CAN bridge is active. Same "flag raised by an
 * HTTP handler, serviced from obd_query_task so it shares the single
 * in-flight request/response slot" pattern as
 * obd_query_clear_dtcs_if_requested. Deliberately does NOT filter by the
 * Mode 01 responder-lock id used in the PID polling loop below: VIN is
 * commonly answered by a different physical ECU (e.g. the gateway) than the
 * one that answers engine PIDs, and this is a one-shot read rather than a
 * continuous poll where a wrong-ECU answer could masquerade as frozen live
 * data the way it did for RPM/Speed. */
static void obd_query_vin_if_requested(void)
{
    if (!s_vin_requested) {
        return;
    }
    s_vin_requested = false;

    if (s_can_passive) {
        ESP_LOGW(TAG, "VIN read requested while CAN bridge is passive; ignored");
        portENTER_CRITICAL(&s_vin_lock);
        s_vin_state.vin[0] = '\0';
        s_vin_state.status = "error";
        portEXIT_CRITICAL(&s_vin_lock);
        return;
    }

    uint8_t request[8] = { 0x02, 0x09, 0x02, 0, 0, 0, 0, 0 };
    s_obd_request_pending = true;
    obd_send_request(request, sizeof(request));

    char vin[18] = {0};
    bool ok = false;
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)OBD_MULTI_ECU_WINDOW_MS * 1000);

    while (esp_timer_get_time() < deadline_us) {
        int64_t remaining_ms = (deadline_us - esp_timer_get_time()) / 1000;
        if (remaining_ms <= 0) {
            break;
        }
        obd_frame_t response;
        if (!xQueueReceive(s_obd_response_queue, &response, pdMS_TO_TICKS(remaining_ms))) {
            break;
        }
        uint8_t payload[ISOTP_MAX_PAYLOAD];
        uint8_t payload_len = 0;
        if (!isotp_reassemble(response.id, &response, payload, &payload_len, deadline_us)) {
            continue;
        }
        if (payload_len < 3 || payload[0] != 0x49 || payload[1] != 0x02) {
            continue; /* not a Mode 09 PID 0x02 reply -- some other response interleaved */
        }
        /* payload[2] is NODI (number of data items, normally 1); the VIN's
         * up to 17 ASCII characters follow starting at payload[3]. Restrict
         * to alphanumerics (real VINs are always alnum, no I/O/Q but any
         * alnum is accepted here) so a garbled/short response can't inject
         * a stray quote or backslash into the hand-built JSON response. */
        uint8_t vin_len = payload_len - 3;
        if (vin_len > 17) {
            vin_len = 17;
        }
        for (uint8_t i = 0; i < vin_len; i++) {
            char c = (char)payload[3 + i];
            vin[i] = isalnum((unsigned char)c) ? c : '?';
        }
        vin[vin_len] = '\0';
        ok = (vin_len > 0);
        break;
    }
    s_obd_request_pending = false;

    portENTER_CRITICAL(&s_vin_lock);
    if (ok) {
        snprintf(s_vin_state.vin, sizeof(s_vin_state.vin), "%s", vin);
        s_vin_state.status = "ok";
    } else {
        s_vin_state.vin[0] = '\0';
        s_vin_state.status = "timeout";
    }
    portEXIT_CRITICAL(&s_vin_lock);

    ESP_LOGI(TAG, "VIN read -> %s", ok ? vin : "no response (timeout)");
}

/* Sends one UDS request (already-built ISO-TP single-frame payload) to a
 * specific module and waits for its response. Point-to-point (exact
 * request/response CAN ID pair, supplied by the caller) rather than
 * obd_send_request's dual-scheme functional broadcast -- see
 * UDS_BODY_MODULE_RESEARCH.md for why body-module UDS doesn't need (or
 * want) that ambiguity-resolution machinery. Reuses isotp_reassemble for
 * multi-frame responses since that logic is generic ISO-TP, not
 * Mode-01-specific. Shared by uds_read_did (service 0x22) and
 * uds_start_session (service 0x10) below. */
static bool uds_send_and_receive(uint32_t request_id, uint32_t response_id, bool extended,
                                  const uint8_t *request, uint8_t request_len,
                                  uint8_t *out_payload, uint8_t *out_len)
{
    s_uds_expected_response_id = response_id;
    s_uds_request_pending = true;
    mcp2515_send(request_id, extended, request_len, request);

    int64_t deadline_us = esp_timer_get_time() + ((int64_t)OBD_RESPONSE_TIMEOUT_MS * 1000);
    obd_frame_t response;
    bool got = xQueueReceive(s_uds_response_queue, &response, pdMS_TO_TICKS(OBD_RESPONSE_TIMEOUT_MS));
    s_uds_request_pending = false;
    if (!got) {
        return false;
    }
    return isotp_reassemble(response.id, &response, out_payload, out_len, deadline_us);
}

/* Same exact-ID-match send/wait as uds_send_and_receive, but returns the
 * raw response frame with no ISO-TP reassembly -- VWTP 2.0's connection-
 * setup frames (see vwtp_connect below) are NOT ISO-TP formatted, so
 * isotp_reassemble() would misinterpret their first byte (an echoed
 * address, not an ISO-TP PCI/length byte) as frame-type metadata and
 * corrupt the read. */
static bool raw_send_and_receive(uint32_t request_id, uint32_t response_id, bool extended,
                                  const uint8_t *request, uint8_t request_len,
                                  obd_frame_t *out_response)
{
    s_uds_expected_response_id = response_id;
    s_uds_request_pending = true;
    mcp2515_send(request_id, extended, request_len, request);

    bool got = xQueueReceive(s_uds_response_queue, out_response, pdMS_TO_TICKS(OBD_RESPONSE_TIMEOUT_MS));
    s_uds_request_pending = false;
    return got;
}

/* Sends UDS ReadDataByIdentifier (service 0x22) for one DID to a specific
 * module and waits for its response. */
static bool uds_read_did(uint32_t request_id, uint32_t response_id, bool extended,
                          uint16_t did, uint8_t *out_payload, uint8_t *out_len)
{
    uint8_t request[8] = { 0x03, 0x22, (uint8_t)(did >> 8), (uint8_t)did, 0, 0, 0, 0 };
    return uds_send_and_receive(request_id, response_id, extended, request, sizeof(request),
                                 out_payload, out_len);
}

/* Sends UDS Diagnostic Session Control (service 0x10, subfunction 0x03 --
 * "extended diagnostic session") to a target module, since some modules
 * (and possibly the Gateway's own routing) only expose/forward a fuller
 * DID set inside a non-default session -- see UDS_BODY_MODULE_RESEARCH.md's
 * 2026-09-15 update. Positive response is 0x50 0x03 (service+0x40, echoing
 * the subfunction); anything else, or a timeout, is recorded but does not
 * block the DID sweep that follows -- worth attempting the sweep either
 * way, since the session requirement (if any) is unconfirmed per module. */
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

/* Services a UDS DID-sweep request set up by uds_scan_start_http_handler:
 * reads every DID in the requested range from one target module, recording
 * only positive responses (0x62 = 0x22 + 0x40, echoing the requested DID)
 * -- a negative response (0x7F) or timeout means "nothing useful here" for
 * this discovery tool and is logged but not stored, to keep the result
 * table focused on real hits. Runs inline in obd_query_task like the VIN
 * read, since it shares the same single in-flight CAN request/response
 * slot; a full sweep can take a while (up to ~700ms per DID if it times
 * out), which is fine for a manually-triggered one-shot tool. */
static void obd_query_uds_scan_if_requested(void)
{
    if (!s_uds_scan_requested) {
        return;
    }
    s_uds_scan_requested = false;

    uint32_t request_id, response_id;
    bool extended;
    uint16_t did_start, did_end;
    portENTER_CRITICAL(&s_uds_scan_lock);
    request_id = s_uds_scan.request_id;
    response_id = s_uds_scan.response_id;
    extended = s_uds_scan.extended;
    did_start = s_uds_scan.did_start;
    did_end = s_uds_scan.did_end;
    portEXIT_CRITICAL(&s_uds_scan_lock);

    if (s_can_passive) {
        ESP_LOGW(TAG, "UDS scan requested while CAN bridge is passive; ignored");
        portENTER_CRITICAL(&s_uds_scan_lock);
        s_uds_scan.state = UDS_SCAN_ERROR;
        portEXIT_CRITICAL(&s_uds_scan_lock);
        return;
    }

    ESP_LOGI(TAG, "UDS scan starting: req=0x%lx resp=0x%lx did=0x%04x-0x%04x",
             (unsigned long)request_id, (unsigned long)response_id, did_start, did_end);

    uds_session_state_t session_state = uds_start_session(request_id, response_id, extended);
    portENTER_CRITICAL(&s_uds_scan_lock);
    s_uds_scan.session_state = session_state;
    portEXIT_CRITICAL(&s_uds_scan_lock);
    ESP_LOGI(TAG, "UDS scan: session control -> %s",
             session_state == UDS_SESSION_POSITIVE ? "positive" :
             session_state == UDS_SESSION_NEGATIVE ? "negative" : "timeout");

    for (uint32_t did = did_start; did <= did_end; did++) {
        portENTER_CRITICAL(&s_uds_scan_lock);
        s_uds_scan.current_did = (uint16_t)did;
        portEXIT_CRITICAL(&s_uds_scan_lock);

        uint8_t payload[ISOTP_MAX_PAYLOAD];
        uint8_t payload_len = 0;
        bool ok = uds_read_did(request_id, response_id, extended, (uint16_t)did, payload, &payload_len);

        if (ok && payload_len >= 3 && payload[0] == 0x62 &&
            (uint16_t)(((uint16_t)payload[1] << 8) | payload[2]) == (uint16_t)did) {
            portENTER_CRITICAL(&s_uds_scan_lock);
            if (s_uds_scan.result_count < UDS_SCAN_MAX_RESULTS) {
                uds_scan_result_t *result = &s_uds_scan.results[s_uds_scan.result_count++];
                result->did = (uint16_t)did;
                result->dlc = payload_len > 8 ? 8 : payload_len;
                memcpy(result->data, payload, result->dlc);
            }
            portEXIT_CRITICAL(&s_uds_scan_lock);
            ESP_LOGI(TAG, "UDS scan: DID 0x%04x -> positive response (%u bytes)", (unsigned)did, payload_len);
        } else if (ok && payload_len >= 3 && payload[0] == 0x7F) {
            ESP_LOGD(TAG, "UDS scan: DID 0x%04x -> negative response, NRC 0x%02x", (unsigned)did, payload[2]);
        } else if (!ok) {
            ESP_LOGD(TAG, "UDS scan: DID 0x%04x -> timeout", (unsigned)did);
        }

        vTaskDelay(pdMS_TO_TICKS(OBD_QUERY_INTERVAL_MS));
    }

    portENTER_CRITICAL(&s_uds_scan_lock);
    s_uds_scan.state = UDS_SCAN_DONE;
    portEXIT_CRITICAL(&s_uds_scan_lock);
    ESP_LOGI(TAG, "UDS scan complete: %u DID(s) responded", (unsigned)s_uds_scan.result_count);
}

/* Services a UDS address-discovery sweep set up by
 * uds_addr_scan_start_http_handler: sends one Diagnostic Session Control
 * request per candidate request ID in the range, recording any address
 * that gets a real response (positive or negative -- either means a live
 * module is actually listening there, unlike a timeout). Same in-flight-
 * request-slot sharing and pacing as the DID sweep above. */
static void obd_query_uds_addr_scan_if_requested(void)
{
    if (!s_uds_addr_scan_requested) {
        return;
    }
    s_uds_addr_scan_requested = false;

    uint32_t req_start, req_end, response_offset;
    bool extended;
    portENTER_CRITICAL(&s_uds_addr_scan_lock);
    req_start = s_uds_addr_scan.req_start;
    req_end = s_uds_addr_scan.req_end;
    response_offset = s_uds_addr_scan.response_offset;
    extended = s_uds_addr_scan.extended;
    portEXIT_CRITICAL(&s_uds_addr_scan_lock);

    if (s_can_passive) {
        ESP_LOGW(TAG, "UDS address scan requested while CAN bridge is passive; ignored");
        portENTER_CRITICAL(&s_uds_addr_scan_lock);
        s_uds_addr_scan.state = UDS_SCAN_ERROR;
        portEXIT_CRITICAL(&s_uds_addr_scan_lock);
        return;
    }

    ESP_LOGI(TAG, "UDS address scan starting: req=0x%lx-0x%lx offset=0x%lx",
             (unsigned long)req_start, (unsigned long)req_end, (unsigned long)response_offset);

    for (uint32_t req_id = req_start; req_id <= req_end; req_id++) {
        portENTER_CRITICAL(&s_uds_addr_scan_lock);
        s_uds_addr_scan.current_req = req_id;
        portEXIT_CRITICAL(&s_uds_addr_scan_lock);

        uint32_t resp_id = req_id + response_offset;
        uds_session_state_t session_state = uds_start_session(req_id, resp_id, extended);

        if (session_state != UDS_SESSION_TIMEOUT) {
            portENTER_CRITICAL(&s_uds_addr_scan_lock);
            if (s_uds_addr_scan.result_count < UDS_ADDR_SCAN_MAX_RESULTS) {
                uds_addr_scan_result_t *result = &s_uds_addr_scan.results[s_uds_addr_scan.result_count++];
                result->request_id = req_id;
                result->session_state = session_state;
            }
            portEXIT_CRITICAL(&s_uds_addr_scan_lock);
            ESP_LOGI(TAG, "UDS address scan: 0x%lx -> %s", (unsigned long)req_id,
                     session_state == UDS_SESSION_POSITIVE ? "positive" : "negative");
        } else {
            ESP_LOGD(TAG, "UDS address scan: 0x%lx -> timeout", (unsigned long)req_id);
        }

        vTaskDelay(pdMS_TO_TICKS(OBD_QUERY_INTERVAL_MS));
    }

    portENTER_CRITICAL(&s_uds_addr_scan_lock);
    s_uds_addr_scan.state = UDS_SCAN_DONE;
    portEXIT_CRITICAL(&s_uds_addr_scan_lock);
    ESP_LOGI(TAG, "UDS address scan complete: %u address(es) responded", (unsigned)s_uds_addr_scan.result_count);
}

/* Deep scan NVS persistence -- see deep_scan_t's doc comment for why this
 * exists (surviving brownout resets during an unattended overnight run).
 * Every function here opens/closes its own handle rather than holding one
 * open for the scan's whole lifetime, since these are called relatively
 * rarely (once per address, once per hit) and it keeps the failure mode of
 * a single call simple to reason about. Failures are logged and otherwise
 * ignored -- losing the ability to resume after a reset is a degradation
 * (falls back to "scan doesn't survive a crash", same as before this
 * feature existed), not a reason to abort an in-progress scan. */
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

/* Marks a deep scan as started/still-in-progress (so a boot-time check
 * knows to resume it) or clears that mark once it finishes or is aborted
 * (Passive mode, out-of-range request) -- the actual scan parameters
 * (req_start/req_end/response_offset/extended) are written once at start
 * time via a separate call from deep_scan_start_http_handler, not here. */
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

/* Called once at boot (see app_main) -- if a deep scan was still marked
 * active in NVS (i.e. the board reset, most likely a brownout, before the
 * scan reached the end of its range or was explicitly stopped), restores
 * its parameters and progress and re-requests it, so it picks back up from
 * roughly where it left off instead of the whole overnight run being lost
 * to one crash. Safe to call even if no deep scan was ever started --
 * nvs_get_u8 on a namespace/key that's never been written just returns
 * ESP_ERR_NVS_NOT_FOUND and this quietly does nothing. */
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
        /* Malformed/stale state -- don't resume into a bad range. */
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

/* Services a deep-scan request set up by deep_scan_start_http_handler (or
 * resumed at boot by deep_scan_resume_from_nvs after a reset): sweeps
 * addresses exactly like obd_query_uds_addr_scan_if_requested above, but
 * for every address that gets a real response (not a timeout), immediately
 * runs the identification-DID sweep (0xF180-0xF1A0) against it before
 * moving to the next candidate -- see deep_scan_t's doc comment for why
 * this exists instead of requiring a manual re-scan per hit. */
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

/* One VWTP 2.0 connection-setup attempt -- see the vwtp_scan_t doc comment
 * above. Always sent to CAN ID 0x200; response comes back on
 * 0x200 + dest_addr. A positive response (byte[1] == 0xD0) carries the
 * negotiated TX CAN ID in bytes[4:5] (little-endian). This discovery tool
 * only records that the address is live and what TX ID it offered -- it
 * doesn't open a full session or exchange any actual diagnostic data. */
static bool vwtp_connect(uint8_t dest_addr, uint16_t requested_rx_id, bool *out_positive, uint16_t *out_tx_id)
{
    uint8_t request[7] = {
        dest_addr, 0xC0, 0x00, 0x10,
        (uint8_t)(requested_rx_id & 0xFF), (uint8_t)(requested_rx_id >> 8),
        0x01,
    };
    obd_frame_t response;
    bool got = raw_send_and_receive(VWTP_CONNECT_REQUEST_ID, VWTP_CONNECT_REQUEST_ID + dest_addr,
                                     false, request, sizeof(request), &response);
    if (!got) {
        return false;
    }
    *out_positive = (response.dlc >= 2 && response.data[1] == 0xD0);
    *out_tx_id = (*out_positive && response.dlc >= 6)
            ? (uint16_t)((uint16_t)response.data[4] | ((uint16_t)response.data[5] << 8))
            : 0;
    return true;
}

/* VWTP scan NVS persistence -- same reasoning and pattern as the deep-scan
 * helpers above (survives brownout resets during an unattended run by
 * persisting progress/results as it goes and resuming automatically at
 * boot). Added 2026-09-17 evening after a real-car deep scan confirmed only
 * the Gateway is reachable via direct addressing, making a clean
 * (non-crashing) full VWTP sweep the next thing worth a resilient,
 * unattended run for. */
#define VWTP_SCAN_NVS_NAMESPACE "vwtpscan"

static void vwtp_scan_nvs_save_progress(uint8_t current_addr)
{
    nvs_handle_t handle;
    if (nvs_open(VWTP_SCAN_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_set_u8(handle, "cur_addr", current_addr);
    nvs_commit(handle);
    nvs_close(handle);
}

static void vwtp_scan_nvs_save_results(void)
{
    nvs_handle_t handle;
    if (nvs_open(VWTP_SCAN_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    portENTER_CRITICAL(&s_vwtp_scan_lock);
    uint8_t result_count = s_vwtp_scan.result_count;
    vwtp_scan_result_t results_copy[VWTP_SCAN_MAX_RESULTS];
    memcpy(results_copy, s_vwtp_scan.results, sizeof(results_copy));
    portEXIT_CRITICAL(&s_vwtp_scan_lock);
    nvs_set_u8(handle, "result_cnt", result_count);
    nvs_set_blob(handle, "results", results_copy, sizeof(results_copy));
    nvs_commit(handle);
    nvs_close(handle);
}

static void vwtp_scan_nvs_set_active(bool active)
{
    nvs_handle_t handle;
    if (nvs_open(VWTP_SCAN_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_set_u8(handle, "active", active ? 1 : 0);
    nvs_commit(handle);
    nvs_close(handle);
}

static void vwtp_scan_nvs_save_params(uint8_t addr_start, uint8_t addr_end, uint16_t rx_id)
{
    nvs_handle_t handle;
    if (nvs_open(VWTP_SCAN_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_set_u8(handle, "addr_start", addr_start);
    nvs_set_u8(handle, "addr_end", addr_end);
    nvs_set_u16(handle, "rx_id", rx_id);
    nvs_set_u8(handle, "cur_addr", addr_start);
    nvs_set_u8(handle, "result_cnt", 0);
    vwtp_scan_result_t empty_results[VWTP_SCAN_MAX_RESULTS] = {0};
    nvs_set_blob(handle, "results", empty_results, sizeof(empty_results));
    nvs_set_u8(handle, "active", 1);
    nvs_commit(handle);
    nvs_close(handle);
}

/* Called once at boot (see app_main), same as deep_scan_resume_from_nvs --
 * resumes an interrupted VWTP scan from roughly where it left off instead
 * of losing an unattended run to a single brownout reset. */
static void vwtp_scan_resume_from_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open(VWTP_SCAN_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return;
    }
    uint8_t active = 0;
    if (nvs_get_u8(handle, "active", &active) != ESP_OK || !active) {
        nvs_close(handle);
        return;
    }
    uint8_t addr_start = 0, addr_end = 0, current_addr = 0, result_count = 0;
    uint16_t rx_id = 0;
    nvs_get_u8(handle, "addr_start", &addr_start);
    nvs_get_u8(handle, "addr_end", &addr_end);
    nvs_get_u16(handle, "rx_id", &rx_id);
    nvs_get_u8(handle, "cur_addr", &current_addr);
    nvs_get_u8(handle, "result_cnt", &result_count);
    vwtp_scan_result_t results[VWTP_SCAN_MAX_RESULTS] = {0};
    size_t results_size = sizeof(results);
    nvs_get_blob(handle, "results", results, &results_size);
    nvs_close(handle);

    if (addr_end < addr_start || current_addr > addr_end) {
        return;
    }

    portENTER_CRITICAL(&s_vwtp_scan_lock);
    s_vwtp_scan.addr_start = addr_start;
    s_vwtp_scan.addr_end = addr_end;
    s_vwtp_scan.rx_id = rx_id;
    s_vwtp_scan.current_addr = current_addr;
    s_vwtp_scan.result_count = result_count > VWTP_SCAN_MAX_RESULTS ? VWTP_SCAN_MAX_RESULTS : result_count;
    memcpy(s_vwtp_scan.results, results, sizeof(results));
    s_vwtp_scan.state = UDS_SCAN_RUNNING;
    portEXIT_CRITICAL(&s_vwtp_scan_lock);
    s_vwtp_scan_requested = true;
    ESP_LOGI(TAG, "VWTP scan: resuming after reset at 0x%02x (range 0x%02x-0x%02x, %u hit(s) so far)",
             current_addr, addr_start, addr_end, (unsigned)result_count);
}

/* Services a VWTP 2.0 address-discovery sweep set up by
 * vwtp_scan_start_http_handler (or resumed at boot by
 * vwtp_scan_resume_from_nvs after a reset): attempts a connection-setup to
 * every logical address in the requested range, recording any that got a
 * real response (positive or negative -- either means a live module is
 * listening there, unlike a timeout). Same in-flight-request-slot sharing
 * and pacing as the UDS address sweep above. */
static void obd_query_vwtp_scan_if_requested(void)
{
    if (!s_vwtp_scan_requested) {
        return;
    }
    s_vwtp_scan_requested = false;

    uint8_t addr_start, addr_end, resume_addr;
    uint16_t rx_id;
    portENTER_CRITICAL(&s_vwtp_scan_lock);
    addr_start = s_vwtp_scan.addr_start;
    addr_end = s_vwtp_scan.addr_end;
    rx_id = s_vwtp_scan.rx_id;
    resume_addr = s_vwtp_scan.current_addr;
    portEXIT_CRITICAL(&s_vwtp_scan_lock);

    if (s_can_passive) {
        ESP_LOGW(TAG, "VWTP scan requested while CAN bridge is passive; ignored");
        portENTER_CRITICAL(&s_vwtp_scan_lock);
        s_vwtp_scan.state = UDS_SCAN_ERROR;
        portEXIT_CRITICAL(&s_vwtp_scan_lock);
        vwtp_scan_nvs_set_active(false);
        return;
    }

    ESP_LOGI(TAG, "VWTP scan starting at 0x%02x (range 0x%02x-0x%02x) rx_id=0x%03x",
             resume_addr, addr_start, addr_end, (unsigned)rx_id);

    for (uint32_t addr = resume_addr; addr <= addr_end; addr++) {
        portENTER_CRITICAL(&s_vwtp_scan_lock);
        s_vwtp_scan.current_addr = (uint8_t)addr;
        portEXIT_CRITICAL(&s_vwtp_scan_lock);
        vwtp_scan_nvs_save_progress((uint8_t)addr);

        bool positive = false;
        uint16_t tx_id = 0;
        bool got = vwtp_connect((uint8_t)addr, rx_id, &positive, &tx_id);

        if (got) {
            portENTER_CRITICAL(&s_vwtp_scan_lock);
            if (s_vwtp_scan.result_count < VWTP_SCAN_MAX_RESULTS) {
                vwtp_scan_result_t *result = &s_vwtp_scan.results[s_vwtp_scan.result_count++];
                result->addr = (uint8_t)addr;
                result->positive = positive;
                result->tx_id = tx_id;
            }
            portEXIT_CRITICAL(&s_vwtp_scan_lock);
            vwtp_scan_nvs_save_results();
            ESP_LOGI(TAG, "VWTP scan: 0x%02x -> %s (tx_id=0x%03x)", (unsigned)addr,
                     positive ? "positive" : "negative", (unsigned)tx_id);
        } else {
            ESP_LOGD(TAG, "VWTP scan: 0x%02x -> timeout", (unsigned)addr);
        }

        vTaskDelay(pdMS_TO_TICKS(OBD_QUERY_INTERVAL_MS));
    }

    portENTER_CRITICAL(&s_vwtp_scan_lock);
    s_vwtp_scan.state = UDS_SCAN_DONE;
    portEXIT_CRITICAL(&s_vwtp_scan_lock);
    vwtp_scan_nvs_set_active(false);
    ESP_LOGI(TAG, "VWTP scan complete: %u address(es) responded", (unsigned)s_vwtp_scan.result_count);
}

/* Acts as a minimal scan tool: requests each supported PID in turn on the
 * broadcast functional ID, waits for the ECU's 0x7E8 reply, and prints it. */
static void obd_query_task(void *arg)
{
    (void)arg;
    static const uint8_t pids[] = { 0x00, 0x05, 0x0C, 0x0D, 0x11, 0x20 };
    uint8_t consecutive_timeouts = 0;
    /* Which physical ECU's responses to trust, once one has answered -- see
     * the response-check below for why this exists. 0 means "not locked
     * yet" (no real CAN ID is ever exactly 0 for either addressing scheme). */
    uint32_t responder_id = 0;

    while (1) {
        obd_query_clear_dtcs_if_requested();
        obd_query_vin_if_requested();
        obd_query_uds_scan_if_requested();
        obd_query_uds_addr_scan_if_requested();
        obd_query_vwtp_scan_if_requested();
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
            /* Real cars (unlike the single-ECU bench simulator) can have more
             * than one module willing to answer a functional-broadcast Mode
             * 01 request -- e.g. a gateway or a second controller that also
             * claims a PID but only ever echoes a stale/static value. Only
             * checking the PID-echo byte (data[2]) let a secondary ECU's
             * frozen answer get accepted just as readily as the real,
             * continuously-updating one from the correct ECU, which looked
             * exactly like "reads a plausible value once, then never moves
             * again" -- confirmed on a real car (Fabia) where RPM read a
             * believable idle value but never tracked the accelerator.
             * Lock onto whichever id answers first for this PID sequence
             * and require the same id afterward, mirroring how
             * s_obd_addressing itself locks onto the first scheme that
             * answers. */
            bool id_ok = (responder_id == 0) || (response.id == responder_id);
            if (got && response.dlc >= 3 && response.data[2] == pids[i] && id_ok) {
                if (responder_id == 0) {
                    responder_id = response.id;
                    ESP_LOGI(TAG, "OBD responder locked to id 0x%lx", (unsigned long)responder_id);
                }
                obd_print_response(pids[i], response.data, response.dlc);
                consecutive_timeouts = 0;
            } else if (got && response.dlc >= 3 && response.data[2] == pids[i]) {
                /* Right PID, wrong ECU -- a secondary module answering the
                 * same broadcast request. Discard; the locked ECU's own
                 * answer (if any) either already arrived or times out below
                 * like normal silence. */
                ESP_LOGW(TAG, "OBD PID 0x%02x -> answered by id 0x%lx, not the locked responder 0x%lx; discarding",
                         pids[i], (unsigned long)response.id, (unsigned long)responder_id);
                consecutive_timeouts = 0;
            } else if (got) {
                /* A stale/misordered response for a different PID; discard rather than misdecode it. */
                ESP_LOGW(TAG, "OBD PID 0x%02x -> mismatched response (got pid 0x%02x), discarding",
                         pids[i], response.data[2]);
                consecutive_timeouts = 0;
            } else {
                ESP_LOGI(TAG, "OBD PID 0x%02x -> no response (timeout)", pids[i]);
                /* Enough consecutive silence (on whichever scheme we'd locked
                 * onto) means the vehicle/simulator may have changed --
                 * re-probe both schemes again instead of staying stuck
                 * asking only the one that used to work. Also drop the
                 * responder lock so a genuinely different/replacement ECU
                 * can be discovered fresh, same reasoning as the addressing
                 * reset. */
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

        obd_query_dtcs();
        vTaskDelay(pdMS_TO_TICKS(OBD_QUERY_INTERVAL_MS));
    }
}

static void start_can_bridge(void)
{
    mcp2515_config_t config = {
        .sck_gpio = CAN_SCK_GPIO,
        .mosi_gpio = CAN_MOSI_GPIO,
        .miso_gpio = CAN_MISO_GPIO,
        .cs_gpio = CAN_CS_GPIO,
    };

    if (mcp2515_init(&config) != ESP_OK) {
        ESP_LOGE(TAG, "CAN bridge init failed; check MCP2515 wiring/power");
        return;
    }
    /* Active by default -- Passive listen-only never ACKs frames, which
     * starves real senders into a continuous retransmit storm that badly
     * distorts observed CAN traffic (see FABIA_ANALYSIS.md). Active is the
     * accurate default; Passive stays available as an explicit opt-in via
     * the app's checkbox for anyone who specifically wants zero bus
     * interaction. */
    s_can_passive = false;

    /* 16, not 4: a multi-frame DTC response from several ECUs can legitimately
     * queue up many frames in a burst (First Frame + several Consecutive
     * Frames, times however many ECUs answer) -- see obd_query_dtcs. */
    s_obd_response_queue = xQueueCreate(16, sizeof(obd_frame_t));
    s_uds_response_queue = xQueueCreate(4, sizeof(obd_frame_t));
    s_identify_semaphore = xSemaphoreCreateBinary();
    if (s_obd_response_queue == NULL || s_uds_response_queue == NULL || s_identify_semaphore == NULL) {
        ESP_LOGE(TAG, "CAN bridge queue allocation failed");
        return;
    }

    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << CAN_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&int_conf));
    esp_err_t isr_result = gpio_install_isr_service(0);
    if (isr_result != ESP_OK && isr_result != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(isr_result);
    }
    if (xTaskCreate(can_echo_task, "can_echo", 4096, NULL, tskIDLE_PRIORITY + 1,
                    &s_can_rx_task) != pdPASS) {
        ESP_LOGE(TAG, "CAN receive task allocation failed");
        return;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(CAN_INT_GPIO, can_int_isr, NULL));
    xTaskCreate(obd_query_task, "obd_query", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    ESP_LOGI(TAG, "CAN bridge ready: INT on GPIO%d, default boot mode is PASSIVE listen-only; use POST /api/can/mode with 'active' to enable OBD queries and test echo.",
             CAN_INT_GPIO);
    ESP_LOGI(TAG, "CAN bridge ready: capturing broadcasts, echoing test id=0x%03x, querying OBD-II",
             CAN_TEST_ID);
}

/* Converts NMEA "ddmm.mmmm"/"dddmm.mmmm" plus hemisphere letter to signed
 * decimal degrees. Returns NAN on malformed input. */
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

/* Splits an NMEA sentence body (already stripped of leading '$' and the
 * trailing "*checksum") into up to max_fields comma-separated fields. Empty
 * fields between consecutive commas become empty strings, not skipped. */
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

/* buf[0]="GPRMC", [1]=time, [2]=status A/V, [3]=lat, [4]=N/S, [5]=lon,
 * [6]=E/W, [7]=speed_knots, [8]=heading_deg, [9]=date. */
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
            ESP_LOGI(TAG, "GPS fix -> lat=%.6f lon=%.6f speed=%.1f km/h heading=%.1f deg utc=%s date=%s",
                     lat, lon, (double)speed_kmh, (double)heading_deg, fields[1], fields[9]);
        } else {
            ESP_LOGI(TAG, "GPS fix -> waiting for valid fix utc=%s date=%s", fields[1], fields[9]);
        }
    }
}

/* buf[0]="GPGGA", [1]=time, [2]=lat, [3]=N/S, [4]=lon, [5]=E/W,
 * [6]=fix_quality, [7]=satellites. */
static void nmea_parse_gga(char **fields, size_t count)
{
    if (count < 8) {
        return;
    }

    portENTER_CRITICAL(&s_gps_lock);
    s_gps_state.satellites = (uint8_t)atoi(fields[7]);
    portEXIT_CRITICAL(&s_gps_lock);
    (void)fields[6];
}

/* Validates the NMEA checksum (XOR of all bytes between '$' and '*') before
 * handing the sentence to a talker-specific parser. */
static void nmea_parse_line(char *line)
{
    if (line[0] != '$') {
        return;
    }

    char *star = strchr(line, '*');
    if (star == NULL || strlen(star) < 3) {
        ESP_LOGW(TAG, "GPS: malformed sentence (no checksum)");
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

/* Reads raw bytes from the GPS UART and reassembles them into '\n'-terminated
 * NMEA lines before parsing; NMEA sentences are short so a single-line buffer
 * is enough. */
static void gps_uart_task(void *arg)
{
    (void)arg;
    static char line_buf[96];
    size_t line_len = 0;
    uint8_t rx_buf[64];

    while (1) {
        int read = uart_read_bytes(GPS_UART_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(200));
        if (read > 0) {
            /* Temporary raw-byte diagnostic while bringing up a real GPS
             * module -- shows exactly what's hitting the UART regardless of
             * whether it parses as NMEA, to tell "nothing wired" apart from
             * "wrong baud rate" (garbled bytes) at a glance. */
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, rx_buf, read, ESP_LOG_INFO);
        }
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
            } else {
                /* Line too long for the buffer; discard it and resync on the next '\n'. */
                line_len = 0;
            }
        }
    }
}

static void start_gps_bridge(void)
{
    uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_UART_TX_GPIO, GPS_UART_RX_GPIO,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (xTaskCreate(gps_uart_task, "gps_uart", 3072, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "GPS UART task allocation failed");
        return;
    }
    ESP_LOGI(TAG, "GPS bridge ready: UART1 RX on GPIO%d, TX on GPIO%d, %d baud",
             GPS_UART_RX_GPIO, GPS_UART_TX_GPIO, GPS_UART_BAUD);
}

static bool health_idle0_hook(void)
{
    s_idle0_count++;
    return false;
}

static bool health_idle1_hook(void)
{
    s_idle1_count++;
    return false;
}

static const char *reset_reason_to_string(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_POWERON: return "POWERON";
        case ESP_RST_EXT: return "EXT";
        case ESP_RST_SW: return "SW";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT: return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_SDIO: return "SDIO";
        case ESP_RST_USB: return "USB";
        case ESP_RST_JTAG: return "JTAG";
        case ESP_RST_EFUSE: return "EFUSE";
        case ESP_RST_PWR_GLITCH: return "PWR_GLITCH";
        case ESP_RST_CPU_LOCKUP: return "CPU_LOCKUP";
        default: return "UNKNOWN";
    }
}

static const char *task_state_to_string(eTaskState state)
{
    switch (state) {
        case eRunning: return "running";
        case eReady: return "ready";
        case eBlocked: return "blocked";
        case eSuspended: return "suspended";
        case eDeleted: return "deleted";
        default: return "unknown";
    }
}

static uint32_t get_partitioned_flash_size(void)
{
    uint32_t total = 0;
    esp_partition_iterator_t iterator = esp_partition_find(ESP_PARTITION_TYPE_ANY,
                                                           ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (iterator != NULL) {
        const esp_partition_t *partition = esp_partition_get(iterator);
        if (partition != NULL) {
            total += partition->size;
        }
        iterator = esp_partition_next(iterator);
    }
    esp_partition_iterator_release(iterator);
    return total;
}

static void health_task(void *arg)
{
    (void)arg;
    uint32_t last_idle0 = s_idle0_count;
    uint32_t last_idle1 = s_idle1_count;
    uint32_t max_idle0_delta = 1;
    uint32_t max_idle1_delta = 1;
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    uint32_t flash_partitioned = get_partitioned_flash_size();
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    uint32_t app_partition_size = running_partition == NULL ? 0 : running_partition->size;
    esp_reset_reason_t restart_reason = esp_reset_reason();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t idle0 = s_idle0_count;
        uint32_t idle1 = s_idle1_count;
        uint32_t idle0_delta = idle0 - last_idle0;
        uint32_t idle1_delta = idle1 - last_idle1;
        last_idle0 = idle0;
        last_idle1 = idle1;
        if (idle0_delta > max_idle0_delta) {
            max_idle0_delta = idle0_delta;
        }
        if (idle1_delta > max_idle1_delta) {
            max_idle1_delta = idle1_delta;
        }

        health_state_t state = {
            .idle0_pct = (idle0_delta * 100u) / max_idle0_delta,
            .idle1_pct = (idle1_delta * 100u) / max_idle1_delta,
            .idle0_delta = idle0_delta,
            .idle1_delta = idle1_delta,
            .idle0_max_delta = max_idle0_delta,
            .idle1_max_delta = max_idle1_delta,
            .heap_free = esp_get_free_heap_size(),
            .heap_min_free = esp_get_minimum_free_heap_size(),
            .internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
            .internal_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
            .psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
            .psram_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
            .flash_size = flash_size,
            .flash_partitioned = flash_partitioned,
            .app_partition_size = app_partition_size,
            .uptime_s = (uint32_t)(esp_timer_get_time() / 1000000),
            .restart_reason = restart_reason,
        };

        portENTER_CRITICAL(&s_health_lock);
        s_health_state = state;
        portEXIT_CRITICAL(&s_health_lock);

        ESP_LOGI(TAG, "HEALTH uptime=%lus heap=%u/%u internal=%u/%u psram=%u/%u idle=%lu%%/%lu%% restart=%s",
                 (unsigned long)state.uptime_s,
                 (unsigned int)state.heap_free, (unsigned int)state.heap_min_free,
                 (unsigned int)state.internal_free, (unsigned int)state.internal_min_free,
                 (unsigned int)state.psram_free, (unsigned int)state.psram_min_free,
                 (unsigned long)state.idle0_pct, (unsigned long)state.idle1_pct,
                 reset_reason_to_string(state.restart_reason));
    }
}

static void start_health_monitor(void)
{
    ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(health_idle0_hook, 0));
    ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(health_idle1_hook, 1));
    xTaskCreate(health_task, "health", 3072, NULL, tskIDLE_PRIORITY + 1, NULL);
}

static void cdc_send(const char *msg)
{
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t *)msg, strlen(msg));
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
}

static void send_menu(void)
{
    char menu[320];
    snprintf(menu, sizeof(menu),
             "\r\n=== JC-ESP32P4-M3 control channel ===\r\n"
             "freq <ms>  set blink half-period, %d-%d ms (current: %lu). Example: freq 250\r\n"
             "ll <0-3>   set log verbosity: 0=DEBUG 1=INFO 2=WARNING 3=ERROR (current: %s)\r\n"
             "help       show this menu\r\n",
             BLINK_HALF_PERIOD_MIN_MS, BLINK_HALF_PERIOD_MAX_MS,
             (unsigned long)blink_half_period_ms,
             log_level_to_string(s_log_level));
    cdc_send(menu);
}

/* Parses "freq <ms>" / "ll <0-3>" / "help" typed into the control channel. */
static void handle_command(char *line)
{
    char msg[96];

    if (strcmp(line, "help") == 0) {
        send_menu();
    } else if (strlen(line) > 0) {
        bool ok = (strncmp(line, "ll ", 3) == 0)
                      ? apply_ll_command(line, msg, sizeof(msg))
                      : apply_freq_command(line, msg, sizeof(msg));
        printf("USB cmd '%s' -> %s", line, ok ? "OK\n" : "ERR\n");
        cdc_send(msg);
    }
}

/* Placeholder for P4<->C6 command ingress once transport pins are confirmed. */
__attribute__((unused))
static void handle_companion_command(const char *line)
{
    char msg[96];
    bool ok = apply_freq_command(line, msg, sizeof(msg));
    printf("C6 bridge cmd '%s' -> %s", line, ok ? "OK\n" : "ERR\n");
}

static void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    static char line[16];
    static size_t line_len = 0;

    uint8_t buf[64];
    size_t rx_size = 0;

    if (tinyusb_cdcacm_read(itf, buf, sizeof(buf), &rx_size) != ESP_OK) {
        return;
    }

    for (size_t i = 0; i < rx_size; i++) {
        char c = (char)buf[i];
        if (c == '\r' || c == '\n') {
            line[line_len] = '\0';
            handle_command(line);
            line_len = 0;
        } else if (line_len < sizeof(line) - 1) {
            line[line_len++] = c;
        }
    }
}

static void tinyusb_cdc_line_state_callback(int itf, cdcacm_event_t *event)
{
    if (event->line_state_changed_data.dtr) {
        send_menu();
    }
}

/* Fallback in case a terminal's DTR toggle isn't caught in time to show the menu. */
static void control_channel_banner_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    send_menu();
    vTaskDelete(NULL);
}

/* Independent USB-CDC control channel on the board's second USB-C port. */
static void start_control_channel(void)
{
    tinyusb_config_t tusb_cfg = TINYUSB_CONFIG_HIGH_SPEED(NULL, NULL);
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    const tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_callback,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));

    xTaskCreate(control_channel_banner_task, "usb_banner", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
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
    printf("HTTP cmd '%s' -> %s", command, accepted ? "OK\n" : "ERR\n");
    httpd_resp_set_type(request, "text/plain");
    httpd_resp_sendstr(request, response);
    return accepted ? ESP_OK : ESP_FAIL;
}

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

static esp_err_t dtc_http_handler(httpd_req_t *request)
{
    dtc_state_t state;
    portENTER_CRITICAL(&s_dtc_lock);
    state = s_dtc_state;
    portEXIT_CRITICAL(&s_dtc_lock);

    /* Sized for up to DTC_MAX_ACTIVE=16 entries, longest description ~80 chars:
     * 16 * ~110 + overhead comfortably fits 2048. */
    char response[2048];
    int written = snprintf(response, sizeof(response), "{\"count\":%u,\"codes\":[", state.count);
    size_t used = (written > 0 && (size_t)written < sizeof(response)) ? (size_t)written : sizeof(response);
    for (uint8_t i = 0; i < state.count && used < sizeof(response); i++) {
        char code_str[6];
        dtc_code_to_string(state.codes[i], code_str, sizeof(code_str));
        written = snprintf(response + used, sizeof(response) - used,
                            "%s{\"code\":\"%s\",\"description\":\"%s\"}",
                            i == 0 ? "" : ",", code_str, dtc_lookup_description(state.codes[i]));
        used += (written > 0 && (size_t)written < sizeof(response) - used) ? (size_t)written : sizeof(response) - used;
    }
    if (used < sizeof(response)) {
        snprintf(response + used, sizeof(response) - used, "]}");
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

/* Actual clearing happens in obd_query_task (obd_query_clear_dtcs_if_requested),
 * which owns the single in-flight OBD request/response slot; this handler
 * just raises the request flag and returns immediately. */
static esp_err_t dtc_clear_http_handler(httpd_req_t *request)
{
    s_dtc_clear_requested = true;
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, "{\"status\":\"clear requested\"}");
    return ESP_OK;
}

/* Returns the last VIN read (if any) -- status is "none" until a read has
 * ever been requested, "ok"/"timeout"/"error" afterward. Doesn't trigger a
 * new read itself; see vin_read_http_handler for that. */
static esp_err_t vin_http_handler(httpd_req_t *request)
{
    char vin[18];
    const char *status;
    portENTER_CRITICAL(&s_vin_lock);
    snprintf(vin, sizeof(vin), "%s", s_vin_state.vin);
    status = s_vin_state.status;
    portEXIT_CRITICAL(&s_vin_lock);

    char response[48];
    snprintf(response, sizeof(response), "{\"vin\":\"%s\",\"status\":\"%s\"}", vin, status);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

/* Actual read happens in obd_query_task (obd_query_vin_if_requested), which
 * owns the single in-flight OBD request/response slot; this handler just
 * raises the request flag and returns immediately, same as
 * dtc_clear_http_handler. The app polls GET /api/obd/vin afterward for the
 * result. */
static esp_err_t vin_read_http_handler(httpd_req_t *request)
{
    s_vin_requested = true;
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, "{\"status\":\"vin read requested\"}");
    return ESP_OK;
}

/* Kicks off a UDS DID-sweep against one target module -- see
 * captures/fabia_2026/UDS_BODY_MODULE_RESEARCH.md. Body (plain text):
 * "req,resp,extended,did_start,did_end" -- req/resp/did_start/did_end in
 * hex without a 0x prefix, extended is 0 or 1, e.g. "71E,788,0,100,200"
 * to sweep DIDs 0x100-0x200 against request 0x71E / response 0x788. */
static esp_err_t uds_scan_start_http_handler(httpd_req_t *request)
{
    char body[64] = {0};
    int received = httpd_req_recv(request, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Expected req,resp,extended,did_start,did_end");
        return ESP_FAIL;
    }
    body[received] = '\0';

    unsigned long req_id = 0, resp_id = 0, extended = 0, did_start = 0, did_end = 0;
    if (sscanf(body, "%lx,%lx,%lu,%lx,%lx", &req_id, &resp_id, &extended, &did_start, &did_end) != 5) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                             "Expected req,resp,extended,did_start,did_end (hex,hex,0|1,hex,hex)");
        return ESP_FAIL;
    }
    if (did_end < did_start || (did_end - did_start + 1) > UDS_SCAN_MAX_RANGE) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "DID range invalid or too large (max 512 DIDs)");
        return ESP_FAIL;
    }

    portENTER_CRITICAL(&s_uds_scan_lock);
    s_uds_scan.request_id = (uint32_t)req_id;
    s_uds_scan.response_id = (uint32_t)resp_id;
    s_uds_scan.extended = extended != 0;
    s_uds_scan.did_start = (uint16_t)did_start;
    s_uds_scan.did_end = (uint16_t)did_end;
    s_uds_scan.current_did = (uint16_t)did_start;
    s_uds_scan.result_count = 0;
    s_uds_scan.session_state = UDS_SESSION_NOT_ATTEMPTED;
    s_uds_scan.state = UDS_SCAN_RUNNING;
    portEXIT_CRITICAL(&s_uds_scan_lock);
    s_uds_scan_requested = true;

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, "{\"status\":\"uds scan requested\"}");
    return ESP_OK;
}

/* Returns current progress/results of the last (or in-progress) UDS scan. */
static esp_err_t uds_scan_status_http_handler(httpd_req_t *request)
{
    uds_scan_t snapshot;
    portENTER_CRITICAL(&s_uds_scan_lock);
    snapshot = s_uds_scan;
    portEXIT_CRITICAL(&s_uds_scan_lock);

    const char *state_str;
    switch (snapshot.state) {
        case UDS_SCAN_RUNNING: state_str = "running"; break;
        case UDS_SCAN_DONE: state_str = "done"; break;
        case UDS_SCAN_ERROR: state_str = "error"; break;
        default: state_str = "idle"; break;
    }

    const char *session_str;
    switch (snapshot.session_state) {
        case UDS_SESSION_POSITIVE: session_str = "positive"; break;
        case UDS_SESSION_NEGATIVE: session_str = "negative"; break;
        case UDS_SESSION_TIMEOUT: session_str = "timeout"; break;
        default: session_str = "none"; break;
    }

    /* Sized for UDS_SCAN_MAX_RESULTS entries at up to ~48 bytes of JSON each. */
    static char response[UDS_SCAN_MAX_RESULTS * 48 + 192];
    int written = snprintf(response, sizeof(response),
            "{\"state\":\"%s\",\"session\":\"%s\",\"current_did\":\"0x%04x\",\"result_count\":%u,\"results\":[",
            state_str, session_str, snapshot.current_did, snapshot.result_count);
    size_t used = (written > 0 && (size_t)written < sizeof(response)) ? (size_t)written : sizeof(response);
    for (uint8_t i = 0; i < snapshot.result_count && used < sizeof(response); i++) {
        char hex[17] = {0};
        uint8_t hex_bytes = snapshot.results[i].dlc > 8 ? 8 : snapshot.results[i].dlc;
        for (uint8_t b = 0; b < hex_bytes; b++) {
            snprintf(hex + b * 2, 3, "%02X", snapshot.results[i].data[b]);
        }
        written = snprintf(response + used, sizeof(response) - used,
                "%s{\"did\":\"0x%04x\",\"data\":\"%s\"}",
                i == 0 ? "" : ",", snapshot.results[i].did, hex);
        used += (written > 0 && (size_t)written < sizeof(response) - used) ? (size_t)written : sizeof(response) - used;
    }
    if (used < sizeof(response)) {
        snprintf(response + used, sizeof(response) - used, "]}");
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

/* Kicks off a UDS address-discovery sweep -- see
 * UDS_BODY_MODULE_RESEARCH.md's 2026-09-15 update. Body (plain text):
 * "req_start,req_end,offset,extended" -- req_start/req_end/offset in hex
 * without a 0x prefix, extended is 0 or 1, e.g. "700,7FF,6A,0" to probe
 * every address 0x700-0x7FF with response = request + 0x6A. */
static esp_err_t uds_addr_scan_start_http_handler(httpd_req_t *request)
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

    portENTER_CRITICAL(&s_uds_addr_scan_lock);
    s_uds_addr_scan.req_start = (uint32_t)req_start;
    s_uds_addr_scan.req_end = (uint32_t)req_end;
    s_uds_addr_scan.response_offset = (uint32_t)offset;
    s_uds_addr_scan.extended = extended != 0;
    s_uds_addr_scan.current_req = (uint32_t)req_start;
    s_uds_addr_scan.result_count = 0;
    s_uds_addr_scan.state = UDS_SCAN_RUNNING;
    portEXIT_CRITICAL(&s_uds_addr_scan_lock);
    s_uds_addr_scan_requested = true;

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, "{\"status\":\"uds address scan requested\"}");
    return ESP_OK;
}

/* Returns current progress/results of the last (or in-progress) UDS
 * address-discovery sweep. */
static esp_err_t uds_addr_scan_status_http_handler(httpd_req_t *request)
{
    uds_addr_scan_t snapshot;
    portENTER_CRITICAL(&s_uds_addr_scan_lock);
    snapshot = s_uds_addr_scan;
    portEXIT_CRITICAL(&s_uds_addr_scan_lock);

    const char *state_str;
    switch (snapshot.state) {
        case UDS_SCAN_RUNNING: state_str = "running"; break;
        case UDS_SCAN_DONE: state_str = "done"; break;
        case UDS_SCAN_ERROR: state_str = "error"; break;
        default: state_str = "idle"; break;
    }

    /* Sized for UDS_ADDR_SCAN_MAX_RESULTS entries at up to ~48 bytes of JSON each. */
    static char response[UDS_ADDR_SCAN_MAX_RESULTS * 48 + 128];
    int written = snprintf(response, sizeof(response),
            "{\"state\":\"%s\",\"current_req\":\"0x%03lx\",\"result_count\":%u,\"results\":[",
            state_str, (unsigned long)snapshot.current_req, snapshot.result_count);
    size_t used = (written > 0 && (size_t)written < sizeof(response)) ? (size_t)written : sizeof(response);
    for (uint8_t i = 0; i < snapshot.result_count && used < sizeof(response); i++) {
        written = snprintf(response + used, sizeof(response) - used,
                "%s{\"req\":\"0x%03lx\",\"session\":\"%s\"}",
                i == 0 ? "" : ",", (unsigned long)snapshot.results[i].request_id,
                snapshot.results[i].session_state == UDS_SESSION_POSITIVE ? "positive" : "negative");
        used += (written > 0 && (size_t)written < sizeof(response) - used) ? (size_t)written : sizeof(response) - used;
    }
    if (used < sizeof(response)) {
        snprintf(response + used, sizeof(response) - used, "]}");
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

/* Kicks off a VWTP 2.0 connection-setup sweep -- see vwtp_scan_t's doc
 * comment. Body (plain text): "addr_start,addr_end,rx_id" -- addr_start/
 * addr_end/rx_id in hex without a 0x prefix, e.g. "0,FF,300" to probe
 * every logical address 0x00-0xFF requesting RX id 0x300. */
static esp_err_t vwtp_scan_start_http_handler(httpd_req_t *request)
{
    char body[32] = {0};
    int received = httpd_req_recv(request, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Expected addr_start,addr_end,rx_id");
        return ESP_FAIL;
    }
    body[received] = '\0';

    unsigned long addr_start = 0, addr_end = 0, rx_id = 0;
    if (sscanf(body, "%lx,%lx,%lx", &addr_start, &addr_end, &rx_id) != 3) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                             "Expected addr_start,addr_end,rx_id (hex,hex,hex)");
        return ESP_FAIL;
    }
    if (addr_end < addr_start || addr_end > 0xFF || (addr_end - addr_start + 1) > VWTP_SCAN_MAX_RANGE) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Address range invalid (0x00-0xFF only)");
        return ESP_FAIL;
    }

    portENTER_CRITICAL(&s_vwtp_scan_lock);
    s_vwtp_scan.addr_start = (uint8_t)addr_start;
    s_vwtp_scan.addr_end = (uint8_t)addr_end;
    s_vwtp_scan.rx_id = (uint16_t)rx_id;
    s_vwtp_scan.current_addr = (uint8_t)addr_start;
    s_vwtp_scan.result_count = 0;
    s_vwtp_scan.state = UDS_SCAN_RUNNING;
    portEXIT_CRITICAL(&s_vwtp_scan_lock);
    /* Always starts fresh from addr_start (unlike the boot-time auto-resume
     * in vwtp_scan_resume_from_nvs, which continues an interrupted one) --
     * resets the persisted resume point and result list accordingly. */
    vwtp_scan_nvs_save_params((uint8_t)addr_start, (uint8_t)addr_end, (uint16_t)rx_id);
    s_vwtp_scan_requested = true;

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, "{\"status\":\"vwtp scan requested\"}");
    return ESP_OK;
}

/* Returns current progress/results of the last (or in-progress) VWTP scan. */
static esp_err_t vwtp_scan_status_http_handler(httpd_req_t *request)
{
    vwtp_scan_t snapshot;
    portENTER_CRITICAL(&s_vwtp_scan_lock);
    snapshot = s_vwtp_scan;
    portEXIT_CRITICAL(&s_vwtp_scan_lock);

    const char *state_str;
    switch (snapshot.state) {
        case UDS_SCAN_RUNNING: state_str = "running"; break;
        case UDS_SCAN_DONE: state_str = "done"; break;
        case UDS_SCAN_ERROR: state_str = "error"; break;
        default: state_str = "idle"; break;
    }

    /* Sized for VWTP_SCAN_MAX_RESULTS entries at up to ~48 bytes of JSON each. */
    static char response[VWTP_SCAN_MAX_RESULTS * 48 + 128];
    int written = snprintf(response, sizeof(response),
            "{\"state\":\"%s\",\"current_addr\":\"0x%02x\",\"result_count\":%u,\"results\":[",
            state_str, snapshot.current_addr, snapshot.result_count);
    size_t used = (written > 0 && (size_t)written < sizeof(response)) ? (size_t)written : sizeof(response);
    for (uint8_t i = 0; i < snapshot.result_count && used < sizeof(response); i++) {
        written = snprintf(response + used, sizeof(response) - used,
                "%s{\"addr\":\"0x%02x\",\"session\":\"%s\",\"tx_id\":\"0x%03x\"}",
                i == 0 ? "" : ",", snapshot.results[i].addr,
                snapshot.results[i].positive ? "positive" : "negative",
                (unsigned)snapshot.results[i].tx_id);
        used += (written > 0 && (size_t)written < sizeof(response) - used) ? (size_t)written : sizeof(response) - used;
    }
    if (used < sizeof(response)) {
        snprintf(response + used, sizeof(response) - used, "]}");
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

/* Kicks off a deep scan -- see deep_scan_t's doc comment. Body (plain
 * text): "req_start,req_end,offset,extended", same format as the plain
 * address-discovery sweep, e.g. "700,7FF,6A,0" to sweep every address
 * 0x700-0x7FF with response = request + 0x6A and automatically identify
 * every hit. Always starts a *fresh* scan from req_start (unlike the
 * boot-time auto-resume in deep_scan_resume_from_nvs, which continues an
 * interrupted one) -- deep_scan_nvs_save_params resets the persisted
 * hit list and resume point accordingly. */
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

/* Returns current progress/results of the last (or in-progress) deep scan
 * -- each hit includes its own nested did_results[], same "did"/"data"
 * (hex) field names as the plain UDS DID sweep's results[] so app-side
 * parsing logic can be shared. */
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

    /* Sized for DEEP_SCAN_MAX_HITS hits, each with up to
     * DEEP_SCAN_MAX_DIDS_PER_HIT DID results at up to ~48 bytes of JSON
     * each, plus per-hit overhead. */
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

static esp_err_t ota_status_http_handler(httpd_req_t *request)
{
    const char *state_str;
    switch (s_ota_state) {
        case OTA_STATE_RECEIVING: state_str = "receiving"; break;
        case OTA_STATE_SUCCESS: state_str = "success"; break;
        case OTA_STATE_FAILED: state_str = "failed"; break;
        default: state_str = "idle"; break;
    }
    /* Which app slot is actually running right now -- toggles between ota_0
     * and ota_1 on every successful update, so the app can confirm a push
     * really took effect (rather than just trusting the reboot happened). */
    const esp_partition_t *running = esp_ota_get_running_partition();
    char response[128];
    snprintf(response, sizeof(response), "{\"state\":\"%s\",\"bytes_written\":%u,\"running_partition\":\"%s\"}",
             state_str, (unsigned)s_ota_bytes_written, running != NULL ? running->label : "unknown");
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

/* Streams the POST body straight into the inactive OTA app partition and,
 * once fully received and validated, sets it as the boot partition and
 * reboots into it. CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE means a new image
 * that fails to get through app_main's rollback-cancel call (e.g. it
 * crash-loops) gets automatically rolled back to the previous image by the
 * bootloader -- the safety net that makes pushing firmware from the car,
 * away from USB/JTAG, an acceptable risk. */
static esp_err_t ota_http_handler(httpd_req_t *request)
{
    if (request->content_len == 0 || request->content_len > OTA_MAX_IMAGE_SIZE) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid or oversized image");
        return ESP_FAIL;
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition available");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, request->content_len, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    s_ota_state = OTA_STATE_RECEIVING;
    s_ota_bytes_written = 0;
    ESP_LOGI(TAG, "OTA update starting: %u bytes -> partition %s", (unsigned)request->content_len, update_partition->label);

    /* Static, not stack-local: this handler already shares the httpd task's
     * stack with can_capture_http_handler's own large local buffer (see
     * config.stack_size's comment at server startup) -- adding another 2KB
     * on the stack on top of that is avoidable by putting this one in .bss
     * instead, same trick used nowhere else in this file only because
     * nothing else needed a buffer this large. */
    static uint8_t buffer[2048];
    size_t remaining = request->content_len;
    bool failed = false;
    while (remaining > 0) {
        size_t to_read = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
        int received = httpd_req_recv(request, (char *)buffer, to_read);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue; /* retry, same as other handlers reading a body */
        }
        if (received <= 0) {
            failed = true;
            break;
        }
        err = esp_ota_write(ota_handle, buffer, (size_t)received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
            failed = true;
            break;
        }
        s_ota_bytes_written += (size_t)received;
        remaining -= (size_t)received;
    }

    if (failed) {
        esp_ota_abort(ota_handle);
        s_ota_state = OTA_STATE_FAILED;
        ESP_LOGW(TAG, "OTA transfer failed after %u bytes", (unsigned)s_ota_bytes_written);
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA transfer failed");
        return ESP_FAIL;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed (image validation): %s", esp_err_to_name(err));
        s_ota_state = OTA_STATE_FAILED;
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Image validation failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA set boot partition failed: %s", esp_err_to_name(err));
        s_ota_state = OTA_STATE_FAILED;
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return ESP_FAIL;
    }

    s_ota_state = OTA_STATE_SUCCESS;
    ESP_LOGI(TAG, "OTA update written (%u bytes) -> %s, rebooting", (unsigned)s_ota_bytes_written, update_partition->label);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, "{\"status\":\"ok, rebooting\"}");

    /* Let the response actually go out over Wi-Fi before the reboot tears
     * down the network stack -- there's nothing to poll afterward the way
     * other "acknowledge then act asynchronously" flows in this file allow,
     * so this one blocks briefly instead. */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK; /* unreached */
}

/* Sends the private fault-injection CAN frame straight from this HTTP
 * handler's own task (mcp2515_send is SPI-mutex protected, so this is safe
 * to call concurrently with obd_query_task/can_echo_task) rather than
 * routing through the OBD request/response slot -- there is no response to
 * wait for on this private channel. */
static esp_err_t dtc_simulate_http_handler(httpd_req_t *request)
{
    char body[8] = {0};
    int received = httpd_req_recv(request, body, sizeof(body) - 1);
    long index = 0;
    if (received > 0) {
        body[received] = '\0';
        index = strtol(body, NULL, 10);
    }
    if (index < 0 || index >= FAULT_CODE_COUNT) {
        index = 0;
    }

    uint8_t payload = (uint8_t)index;
    mcp2515_send(FAULT_INJECT_CAN_ID, false, 1, &payload);
    ESP_LOGI(TAG, "Fault simulate: requested index=%ld", index);

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, "{\"status\":\"fault injection requested\"}");
    return ESP_OK;
}

/* Re-runs the simulator/real-vehicle + addressing-scheme identify check on
 * every call (not just cached from obd_query_task's background state) so the
 * app's status display is always current, e.g. right after switching the
 * simulator's mode below. */
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

/* Only meaningful when the bench simulator is on the bus (SIM_MODE_SWITCH_CAN_ID
 * is a private ID the simulator alone understands -- a real vehicle just
 * ignores it, harmlessly). Body is "11" or "29". Resets s_obd_addressing so
 * the next query re-probes instead of continuing to ask only the old scheme. */
static esp_err_t can_sim_mode_http_handler(httpd_req_t *request)
{
    char mode[8] = {0};
    int received = httpd_req_recv(request, mode, sizeof(mode) - 1);
    if (received <= 0) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Expected 11 or 29");
        return ESP_FAIL;
    }
    mode[received] = '\0';

    uint8_t payload;
    if (strcmp(mode, "29") == 0) {
        payload = 1;
    } else if (strcmp(mode, "11") == 0) {
        payload = 0;
    } else {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Expected 11 or 29");
        return ESP_FAIL;
    }

    mcp2515_send(SIM_MODE_SWITCH_CAN_ID, false, 1, &payload);
    portENTER_CRITICAL(&s_obd_addressing_lock);
    s_obd_addressing = OBD_ADDR_UNKNOWN;
    portEXIT_CRITICAL(&s_obd_addressing_lock);
    ESP_LOGI(TAG, "Simulator mode switch requested: %s-bit", mode);

    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, "{\"status\":\"simulator mode switch requested\"}");
    return ESP_OK;
}

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

    char *response = malloc(CAN_CAPTURE_RESPONSE_SIZE);
    if (response == NULL) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_ERR_NO_MEM;
    }

    size_t used = (size_t)snprintf(response, CAN_CAPTURE_RESPONSE_SIZE,
                                   "{\"latest\":%llu,\"dropped\":%llu,\"hardware_overflow\":%lu,"
                                   "\"passive\":%s,\"isr_count\":%lu,\"timeout_count\":%lu,\"frames\":[",
                                   (unsigned long long)latest, (unsigned long long)dropped,
                                   (unsigned long)mcp2515_get_receive_overflow_count(),
                                   s_can_passive ? "true" : "false",
                                   (unsigned long)s_can_isr_count, (unsigned long)s_can_timeout_count);
    for (size_t i = 0; i < count; i++) {
        can_capture_frame_t *frame = &batch[i];
        used += (size_t)snprintf(response + used, CAN_CAPTURE_RESPONSE_SIZE - used,
                                 "%s{\"seq\":%llu,\"time_us\":%lld,\"bus\":0,\"id\":%lu,"
                                 "\"extended\":false,\"rtr\":false,\"dlc\":%u,\"data\":\"",
                                 i == 0 ? "" : ",", (unsigned long long)frame->sequence,
                                 (long long)frame->timestamp_us, (unsigned long)frame->id, frame->dlc);
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

static esp_err_t can_mode_http_handler(httpd_req_t *request)
{
    char mode[16] = {0};
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

    if (mcp2515_set_listen_only(passive) != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "CAN mode change failed");
        return ESP_FAIL;
    }

    s_can_passive = passive;
    if (!passive) {
        xQueueReset(s_obd_response_queue);
    }
    ESP_LOGI(TAG, "CAN mode changed to %s", passive ? "passive" : "active");
    httpd_resp_set_type(request, "text/plain");
    httpd_resp_sendstr(request, passive ? "OK passive" : "OK active");
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

static esp_err_t health_http_handler(httpd_req_t *request)
{
    health_state_t state;
    portENTER_CRITICAL(&s_health_lock);
    state = s_health_state;
    portEXIT_CRITICAL(&s_health_lock);

    uint32_t flash_free = state.flash_size > state.flash_partitioned
                              ? state.flash_size - state.flash_partitioned
                              : 0;
    /* ESP-IDF auto-populates this from `git describe --always --dirty` at
     * build time (no manual versioning needed, same idea as the Android
     * app's git-hash-based versionName) -- this is the only way to
     * distinguish which firmware build is actually running on the board
     * after an OTA push, since nothing else in this API surfaces it. */
    const esp_app_desc_t *app_desc = esp_app_get_description();
    char response[640];
    snprintf(response, sizeof(response),
             "{\"firmware_version\":\"%s\",\"uptime_s\":%lu,\"restart_reason\":\"%s\","
             "\"heap_free\":%u,\"heap_min_free\":%u,"
             "\"internal_free\":%u,\"internal_min_free\":%u,"
             "\"psram_free\":%u,\"psram_min_free\":%u,"
             "\"flash_size\":%lu,\"flash_partitioned\":%lu,\"flash_free\":%lu,"
             "\"app_partition_size\":%lu,"
             "\"idle0_pct\":%lu,\"idle1_pct\":%lu,"
             "\"busy0_pct\":%lu,\"busy1_pct\":%lu,"
             "\"idle0_delta\":%lu,\"idle1_delta\":%lu,"
             "\"idle0_max_delta\":%lu,\"idle1_max_delta\":%lu}",
             app_desc->version,
             (unsigned long)state.uptime_s, reset_reason_to_string(state.restart_reason),
             (unsigned int)state.heap_free, (unsigned int)state.heap_min_free,
             (unsigned int)state.internal_free, (unsigned int)state.internal_min_free,
             (unsigned int)state.psram_free, (unsigned int)state.psram_min_free,
             (unsigned long)state.flash_size, (unsigned long)state.flash_partitioned,
             (unsigned long)flash_free, (unsigned long)state.app_partition_size,
             (unsigned long)state.idle0_pct, (unsigned long)state.idle1_pct,
             (unsigned long)(100u - state.idle0_pct), (unsigned long)(100u - state.idle1_pct),
             (unsigned long)state.idle0_delta, (unsigned long)state.idle1_delta,
             (unsigned long)state.idle0_max_delta, (unsigned long)state.idle1_max_delta);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, response);
    return ESP_OK;
}

static esp_err_t tasks_http_handler(httpd_req_t *request)
{
#if !CONFIG_FREERTOS_USE_TRACE_FACILITY
    /* uxTaskGetSystemState() is only linkable when CONFIG_FREERTOS_USE_TRACE_FACILITY
     * is enabled. That option adds per-task tracing fields that increase FreeRTOS's
     * static kernel memory footprint, which on this esp32p4 rev-<v3 board shrinks the
     * "RETENT_RAM" heap region enough to shift which early-boot failure manifests.
     * The actual root cause (an unused PMU sleep-clock-ICG REGDMA retention path,
     * see CONFIG_PM_SLEEP_CLK_ICG_ENABLE in sdkconfig.defaults) is now fixed
     * separately, but trace facility is kept off here as a safety margin against
     * further RETENT_RAM squeezes rather than trading a working board for a task
     * list. Degrade this one endpoint gracefully instead. */
    httpd_resp_set_status(request, "501 Not Implemented");
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request, "{\"error\":\"CONFIG_FREERTOS_USE_TRACE_FACILITY is disabled on this build\"}");
    return ESP_OK;
#else
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    TaskStatus_t *tasks = calloc(task_count, sizeof(TaskStatus_t));
    char *response = malloc(TASKS_RESPONSE_SIZE);
    if (tasks == NULL || response == NULL) {
        free(tasks);
        free(response);
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_ERR_NO_MEM;
    }

    UBaseType_t count = uxTaskGetSystemState(tasks, task_count, NULL);
    size_t used = (size_t)snprintf(response, TASKS_RESPONSE_SIZE,
                                   "{\"count\":%lu,\"tasks\":[", (unsigned long)count);
    for (UBaseType_t i = 0; i < count && used < TASKS_RESPONSE_SIZE; i++) {
        TaskStatus_t *task = &tasks[i];
        used += (size_t)snprintf(response + used, TASKS_RESPONSE_SIZE - used,
                                 "%s{\"name\":\"%s\",\"priority\":%lu,"
                                 "\"base_priority\":%lu,\"state\":\"%s\","
                                 "\"stack_high_watermark\":%lu}",
                                 i == 0 ? "" : ",",
                                 task->pcTaskName,
                                 (unsigned long)task->uxCurrentPriority,
                                 (unsigned long)task->uxBasePriority,
                                 task_state_to_string(task->eCurrentState),
                                 (unsigned long)task->usStackHighWaterMark);
    }
    snprintf(response + used, TASKS_RESPONSE_SIZE - used, "]}");

    httpd_resp_set_type(request, "application/json");
    esp_err_t result = httpd_resp_sendstr(request, response);
    free(tasks);
    free(response);
    return result;
#endif
}

static void start_frequency_http_server(void)
{
    if (http_server != NULL) {
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    /* Default max_uri_handlers is 8; we now register 24 (frequency, obd,
     * can_capture, can_mode, gps, health, tasks, dtc, dtc_clear, dtc_simulate,
     * can_partner, can_sim_mode, vin, vin_read, ota_status, ota,
     * uds_scan_status, uds_scan_start, uds_addr_scan_status,
     * uds_addr_scan_start, vwtp_scan_status, vwtp_scan_start,
     * deep_scan_status, deep_scan_start). Without raising this,
     * registrations past the 8th silently fail (ESP_ERR_HTTPD_HANDLERS_FULL)
     * instead of crashing, which is a much harder bug to notice -- endpoints
     * just 404. */
    config.max_uri_handlers = 28;
    /* Default stack_size is 4096. can_capture_http_handler's local
     * `batch[CAN_CAPTURE_HTTP_BATCH]` array alone is now 128*32=4096 bytes --
     * the entire default stack, with nothing left for anything else in that
     * handler (or any other handler sharing this same httpd task). */
    config.stack_size = 10240;
    httpd_uri_t frequency_uri = {
        .uri = "/api/frequency",
        .method = HTTP_POST,
        .handler = frequency_http_handler,
    };
    httpd_uri_t obd_uri = {
        .uri = "/api/obd",
        .method = HTTP_GET,
        .handler = obd_http_handler,
    };
    httpd_uri_t can_capture_uri = {
        .uri = "/api/can",
        .method = HTTP_GET,
        .handler = can_capture_http_handler,
    };
    httpd_uri_t can_mode_uri = {
        .uri = "/api/can/mode",
        .method = HTTP_POST,
        .handler = can_mode_http_handler,
    };
    httpd_uri_t gps_uri = {
        .uri = "/api/gps",
        .method = HTTP_GET,
        .handler = gps_http_handler,
    };
    httpd_uri_t health_uri = {
        .uri = "/api/health",
        .method = HTTP_GET,
        .handler = health_http_handler,
    };
    httpd_uri_t tasks_uri = {
        .uri = "/api/tasks",
        .method = HTTP_GET,
        .handler = tasks_http_handler,
    };
    httpd_uri_t dtc_uri = {
        .uri = "/api/dtc",
        .method = HTTP_GET,
        .handler = dtc_http_handler,
    };
    httpd_uri_t dtc_clear_uri = {
        .uri = "/api/dtc/clear",
        .method = HTTP_POST,
        .handler = dtc_clear_http_handler,
    };
    httpd_uri_t dtc_simulate_uri = {
        .uri = "/api/dtc/simulate",
        .method = HTTP_POST,
        .handler = dtc_simulate_http_handler,
    };
    httpd_uri_t can_partner_uri = {
        .uri = "/api/can/partner",
        .method = HTTP_GET,
        .handler = can_partner_http_handler,
    };
    httpd_uri_t can_sim_mode_uri = {
        .uri = "/api/can/sim_mode",
        .method = HTTP_POST,
        .handler = can_sim_mode_http_handler,
    };
    httpd_uri_t vin_uri = {
        .uri = "/api/obd/vin",
        .method = HTTP_GET,
        .handler = vin_http_handler,
    };
    httpd_uri_t vin_read_uri = {
        .uri = "/api/obd/vin",
        .method = HTTP_POST,
        .handler = vin_read_http_handler,
    };
    httpd_uri_t ota_status_uri = {
        .uri = "/api/ota",
        .method = HTTP_GET,
        .handler = ota_status_http_handler,
    };
    httpd_uri_t ota_uri = {
        .uri = "/api/ota",
        .method = HTTP_POST,
        .handler = ota_http_handler,
    };
    httpd_uri_t uds_scan_status_uri = {
        .uri = "/api/uds/scan",
        .method = HTTP_GET,
        .handler = uds_scan_status_http_handler,
    };
    httpd_uri_t uds_scan_start_uri = {
        .uri = "/api/uds/scan",
        .method = HTTP_POST,
        .handler = uds_scan_start_http_handler,
    };
    httpd_uri_t uds_addr_scan_status_uri = {
        .uri = "/api/uds/addrscan",
        .method = HTTP_GET,
        .handler = uds_addr_scan_status_http_handler,
    };
    httpd_uri_t uds_addr_scan_start_uri = {
        .uri = "/api/uds/addrscan",
        .method = HTTP_POST,
        .handler = uds_addr_scan_start_http_handler,
    };
    httpd_uri_t vwtp_scan_status_uri = {
        .uri = "/api/vwtp/scan",
        .method = HTTP_GET,
        .handler = vwtp_scan_status_http_handler,
    };
    httpd_uri_t vwtp_scan_start_uri = {
        .uri = "/api/vwtp/scan",
        .method = HTTP_POST,
        .handler = vwtp_scan_start_http_handler,
    };
    httpd_uri_t deep_scan_status_uri = {
        .uri = "/api/deepscan",
        .method = HTTP_GET,
        .handler = deep_scan_status_http_handler,
    };
    httpd_uri_t deep_scan_start_uri = {
        .uri = "/api/deepscan",
        .method = HTTP_POST,
        .handler = deep_scan_start_http_handler,
    };

    if (httpd_start(&http_server, &config) == ESP_OK) {
        httpd_register_uri_handler(http_server, &frequency_uri);
        httpd_register_uri_handler(http_server, &obd_uri);
        httpd_register_uri_handler(http_server, &can_capture_uri);
        httpd_register_uri_handler(http_server, &can_mode_uri);
        httpd_register_uri_handler(http_server, &gps_uri);
        httpd_register_uri_handler(http_server, &health_uri);
        httpd_register_uri_handler(http_server, &dtc_uri);
        httpd_register_uri_handler(http_server, &dtc_clear_uri);
        httpd_register_uri_handler(http_server, &dtc_simulate_uri);
        httpd_register_uri_handler(http_server, &tasks_uri);
        httpd_register_uri_handler(http_server, &can_partner_uri);
        httpd_register_uri_handler(http_server, &can_sim_mode_uri);
        httpd_register_uri_handler(http_server, &vin_uri);
        httpd_register_uri_handler(http_server, &vin_read_uri);
        httpd_register_uri_handler(http_server, &ota_status_uri);
        httpd_register_uri_handler(http_server, &ota_uri);
        httpd_register_uri_handler(http_server, &uds_scan_status_uri);
        httpd_register_uri_handler(http_server, &uds_scan_start_uri);
        httpd_register_uri_handler(http_server, &uds_addr_scan_status_uri);
        httpd_register_uri_handler(http_server, &uds_addr_scan_start_uri);
        httpd_register_uri_handler(http_server, &vwtp_scan_status_uri);
        httpd_register_uri_handler(http_server, &vwtp_scan_start_uri);
        httpd_register_uri_handler(http_server, &deep_scan_status_uri);
        httpd_register_uri_handler(http_server, &deep_scan_start_uri);
        ESP_LOGI(TAG, "WiFi frequency API ready: POST /api/frequency");
        ESP_LOGI(TAG, "OBD monitor API ready: GET /api/obd");
        ESP_LOGI(TAG, "CAN capture API ready: GET /api/can?after=<sequence>");
        ESP_LOGI(TAG, "CAN mode API ready: POST /api/can/mode");
        ESP_LOGI(TAG, "GPS API ready: GET /api/gps");
        ESP_LOGI(TAG, "Health API ready: GET /api/health");
        ESP_LOGI(TAG, "Tasks API ready: GET /api/tasks");
        ESP_LOGI(TAG, "DTC API ready: GET /api/dtc, POST /api/dtc/clear, POST /api/dtc/simulate");
        ESP_LOGI(TAG, "CAN partner API ready: GET /api/can/partner, POST /api/can/sim_mode");
        ESP_LOGI(TAG, "VIN API ready: GET /api/obd/vin, POST /api/obd/vin");
        ESP_LOGI(TAG, "OTA API ready: GET /api/ota (status), POST /api/ota (push new firmware image)");
        ESP_LOGI(TAG, "UDS scan API ready: GET /api/uds/scan (status/results), POST /api/uds/scan (req,resp,extended,did_start,did_end)");
        ESP_LOGI(TAG, "UDS address scan API ready: GET /api/uds/addrscan (status/results), POST /api/uds/addrscan (req_start,req_end,offset,extended)");
        ESP_LOGI(TAG, "VWTP scan API ready: GET /api/vwtp/scan (status/results), POST /api/vwtp/scan (addr_start,addr_end,rx_id)");
        ESP_LOGI(TAG, "Deep scan API ready: GET /api/deepscan (status/results), POST /api/deepscan (req_start,req_end,offset,extended)");
    }
}

#if defined(CONFIG_ESP_HOSTED_ENABLED) && CONFIG_ESP_HOSTED_ENABLED
static void start_hosted_wifi_link(void)
{
    esp_err_t err = esp_hosted_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ESP-Hosted init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_hosted_connect_to_slave();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ESP-Hosted slave connect failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "WiFi netif init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "WiFi event loop init failed: %s", esp_err_to_name(err));
        return;
    }

    if (esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "WiFi AP netif create failed");
        return;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(err));
        return;
    }

    /* Default country setting only scans channels 1-11; many routers (EU/IL) use 12-13. */
    wifi_country_t country = {
        .cc = "01",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_MANUAL,
    };
    esp_wifi_set_country(&country);

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi set mode failed: %s", esp_err_to_name(err));
        return;
    }

    wifi_config_t ap_config = {0};
    snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), "%s", WIFI_AP_SSID);
    ap_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    snprintf((char *)ap_config.ap.password, sizeof(ap_config.ap.password), "%s", WIFI_AP_PASSWORD);
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_config.ap.max_connection = 4;
    ap_config.ap.channel = 6;
    /* Android's WifiNetworkSpecifier (local-only network requests, used by
     * the phone app to auto-join this AP in the background) is stricter
     * about PMF/802.11w than a normal Wi-Fi-settings join -- without this,
     * newer Android versions can silently fail to connect via that API even
     * though the exact same SSID/password joins fine manually. Advertise
     * PMF-capable (but not required, for compatibility with anything else
     * that might connect) rather than leaving it at the ESP-IDF default of
     * fully disabled. */
    ap_config.ap.pmf_cfg.capable = true;
    ap_config.ap.pmf_cfg.required = false;
    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi AP config failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi max TX power limited to %.2f dBm", WIFI_MAX_TX_POWER_QDBM / 4.0f);
    } else {
        ESP_LOGW(TAG, "WiFi TX power limit failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Hosted radio link is up (P4 host, C6 co-processor).");

    /* The AP is live immediately -- the board is AP-only, so this is the
     * only Wi-Fi readiness signal there is. */
    ESP_LOGI(TAG, "SoftAP up: ssid=%s ip=%s", WIFI_AP_SSID, WIFI_AP_IP);
    start_frequency_http_server();
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED)
    s_wifi_ready = true;
    char ap_response[64];
    snprintf(ap_response, sizeof(ap_response), "WiFi connected ip=%s\r\n", WIFI_AP_IP);
    ble_publish_response(BLE_HS_CONN_HANDLE_NONE, ap_response);
#endif
}
#else
static void start_hosted_wifi_link(void)
{
    ESP_LOGI(TAG, "ESP-Hosted disabled in this build; hosted radio link not started.");
}
#endif

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED)
#define BLE_COMPANION_SERVICE_UUID 0xFFF0
#define BLE_COMPANION_CHAR_UUID    0xFFF1
#define BLE_RESPONSE_CHAR_UUID     0xFFF2

static uint8_t s_ble_addr_type = 0;
static uint16_t s_ble_response_handle;
static uint16_t s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static char s_ble_response[96] = "Ready\r\n";

static void ble_start_advertising(void);

static int ble_response_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return os_mbuf_append(ctxt->om, s_ble_response, strlen(s_ble_response)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static void ble_publish_response(uint16_t conn_handle, const char *message)
{
    snprintf(s_ble_response, sizeof(s_ble_response), "%s", message);
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        conn_handle = s_ble_conn_handle;
    }
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        printf("BLE response not sent (no active connection): %s", message);
        return;
    }
    int rc = ble_gatts_notify(conn_handle, s_ble_response_handle);
    printf("BLE notify conn_handle=%d rc=%d: %s", (int)conn_handle, rc, message);
    if (rc != 0) {
        printf("BLE response notify not sent: rc=%d\n", rc);
    }
}

static int ble_freq_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    char cmd[64] = {0};
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len >= sizeof(cmd)) {
        len = sizeof(cmd) - 1;
    }

    if (ble_hs_mbuf_to_flat(ctxt->om, cmd, len, NULL) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    cmd[len] = '\0';

    char msg[96];
    bool ok = apply_freq_command(cmd, msg, sizeof(msg));
    printf("BLE cmd '%s' -> %s", cmd, ok ? "OK\n" : "ERR\n");
    ble_publish_response(conn_handle, msg);
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_COMPANION_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(BLE_COMPANION_CHAR_UUID),
                .access_cb = ble_freq_write_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = BLE_UUID16_DECLARE(BLE_RESPONSE_CHAR_UUID),
                .access_cb = ble_response_read_cb,
                .val_handle = &s_ble_response_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0},
        },
    },
    {0},
};

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_CONNECT && event->connect.status == 0) {
        s_ble_conn_handle = event->connect.conn_handle;
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_start_advertising();
    } else if (event->type == BLE_GAP_EVENT_SUBSCRIBE) {
        /* Fires when the phone app finishes subscribing to notifications on the
         * response characteristic, right after every BLE (re)connect. The board's
         * one-shot "WiFi connected ip=..." notify (sent once, at AP startup) is
         * otherwise missed whenever the app wasn't already subscribed at that
         * moment. Resending it here means the app learns the (always-fixed)
         * AP IP on every connect. */
        if (event->subscribe.attr_handle == s_ble_response_handle &&
            event->subscribe.cur_notify && s_wifi_ready) {
            char response[64];
            snprintf(response, sizeof(response), "WiFi connected ip=%s\r\n", WIFI_AP_IP);
            ble_publish_response(event->subscribe.conn_handle, response);
        }
    }
    return 0;
}

static void ble_start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    const char *name = "JC-P4-C6";
    fields.name = (const uint8_t *)name;
    fields.name_len = strlen(name);
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
    rc = ble_gap_adv_start(s_ble_addr_type, NULL, BLE_HS_FOREVER, &adv, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE adv start failed: rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "BLE advertising started as %s", name);
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

static void start_ble_hosted(void)
{
    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "NimBLE init failed: rc=%d", rc);
        return;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE count cfg failed: rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE add service failed: rc=%d", rc);
        return;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "NimBLE host started (controller on C6 over hosted link).");
}
#else
static void start_ble_hosted(void)
{
    ESP_LOGI(TAG, "BLE disabled in this build; hosted BLE not started.");
}
#endif

void app_main(void)
{
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    /* Pick a deep scan or VWTP scan back up if the board reset (most likely
     * a brownout under sustained CAN-TX load, a real and repeatedly
     * observed failure mode -- see deep_scan_t's doc comment) while one was
     * still in progress, instead of silently losing an unattended overnight
     * run to a single crash partway through. */
    deep_scan_resume_from_nvs();
    vwtp_scan_resume_from_nvs();

    /* If this boot is into a freshly OTA-pushed image (ota_http_handler),
     * the bootloader marks it "pending verify" and will roll back to the
     * previous image on the NEXT boot unless something here confirms it's
     * good first. Getting this far without crashing is treated as
     * sufficient confirmation for this app (no deeper self-test) -- the
     * whole point of CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE is a safety net
     * for a bad push made away from USB/JTAG, not a replacement for testing
     * before pushing. */
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_img_state;
    if (running_partition != NULL &&
        esp_ota_get_state_partition(running_partition, &ota_img_state) == ESP_OK &&
        ota_img_state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "OTA image on partition %s marked valid; rollback canceled", running_partition->label);
    }

    init_led();
    esp_log_level_set(TAG, s_log_level);
    start_health_monitor();

    ESP_LOGI(TAG, "JC-ESP32P4-M3 booted. Blinking LED on GPIO %d", LED_GPIO);
    ESP_LOGI(TAG, "Free heap before hosted init: %lu bytes (internal: %lu)",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    start_control_channel();
    start_hosted_wifi_link();
    start_ble_hosted();
    start_can_bridge();
    start_gps_bridge();

    uint32_t count = 0;
    while (1) {
        uint32_t half_period = blink_half_period_ms;

        gpio_set_level(LED_GPIO, 1);
        ESP_LOGD(TAG, "ON");
        vTaskDelay(pdMS_TO_TICKS(half_period));

        gpio_set_level(LED_GPIO, 0);
        ESP_LOGD(TAG, "OFF");
        vTaskDelay(pdMS_TO_TICKS(half_period));

        count++;
    }
}
