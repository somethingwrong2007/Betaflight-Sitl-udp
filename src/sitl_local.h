/**
 * In-process Betaflight SITL API for simulators (SITL_LINK_MODE=LOCAL).
 *
 * Instead of exchanging fdm_packet / motor packets over UDP, the host engine
 * (e.g. Unreal's async physics tick at 1000 Hz) calls sitl_local_step()
 * synchronously: the flight controller receives the simulator state, runs one
 * scheduler pass on its virtual clock, and returns the motor outputs for that
 * exact state with zero network latency.
 *
 * The web configurator still works: the DLL keeps the TCP/WebSocket proxy
 * alive on 127.0.0.1:5761 / 6761, and a background thread services MSP.
 */

#ifndef SRC_SITL_LOCAL_H
#define SRC_SITL_LOCAL_H

#include <stdint.h>
#include <stdbool.h>

#if defined(_WIN32) && defined(SITL_LOCAL)
#define SITL_LOCAL_API __declspec(dllexport)
#else
#define SITL_LOCAL_API
#endif

#define SITL_LOCAL_MAX_RC_CHANNELS 16
#define SITL_LOCAL_MAX_MOTORS      16
#define SITL_LOCAL_MAX_SERVOS      8

typedef struct {
    double timestamp;                   // seconds, monotonic (logging only)
    double angular_velocity_rpy[3];     // rad/s, FRD body frame
    double linear_acceleration_xyz[3];  // m/s^2, FRD body specific force
    double orientation_quat[4];         // w,x,y,z, same convention as the UDP fdm_packet
    double velocity_xyz[3];             // m/s, earth frame ENU (east, north, up)
    double position_xyz[3];             // longitude, latitude, altitude (like UDP mode)
    double pressure;                    // Pa (ignored: derived from altitude)
    double battery_voltage;             // V
    double battery_current;             // A
    double motor_rpm[4];                // per-motor RPM (telemetry)
    uint16_t rc_channels[SITL_LOCAL_MAX_RC_CHANNELS]; // 1000..2000, AETR + aux
    // Simulated per-motor ESC temperature in degrees Celsius (telemetry).
    // Appended after rc_channels so every earlier field keeps its offset.
    // Values <= 0 mean "no data" and keep the last known temperature.
    double motor_temperature[4];
} sitl_local_input_t;

typedef struct {
    uint8_t motor_count;
    float pwm_output_raw[SITL_LOCAL_MAX_MOTORS]; // 1000..2000 (raw PWM)
    bool armed;
    // Fixed-wing / unusual mixers also produce servo outputs (after the
    // motors in the raw PWM stream). servo_count is 0 for multicopters.
    uint8_t servo_count;
    float servo_output_raw[SITL_LOCAL_MAX_SERVOS]; // 1000..2000 (raw PWM)
} sitl_local_output_t;

// Per-step snapshot of the signals that drive the rate loop. Test harnesses use
// it to tell *which* part of the loop moved when a save or a restart changes the
// motor response (stick setpoint, filtered gyro, a single PID term), instead of
// only seeing the final motor mix. Read-only, call from the stepping thread.
typedef struct {
    float gyroADC[3];        // scaled, unfiltered gyro (the simulator feed)
    float gyroADCf[3];       // filtered gyro, what the PID actually sees
    float rcCommand[4];      // roll/pitch/yaw setpoint (rc units) + throttle
    float pidP[3], pidI[3], pidD[3], pidF[3], pidSum[3];
    uint32_t pidDeltaUs;     // last measured PID loop period
    uint32_t gyroDeltaUs;    // last measured gyro loop period
    // Gyro filter chain internals: sampleSum is the downsampled (LPF2) value
    // that feeds the rest of the chain, lpf1State/lpf2State are the two
    // lowpass filter states (0 when that filter is not in use).
    float sampleSum[3];
    float lpf1State, lpf2State;
    // LPF1 (gyro lowpass 1) runtime configuration, which the dynamic lowpass
    // re-tunes from the throttle while the loop runs.
    float lpf1K;
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin, dynLpfMax;
    uint8_t dynLpfExpo;
    // Configured gains (currentPidProfile), so a test can tell a config write
    // that never reached the loop from one that did.
    uint8_t cfgP[3], cfgI[3], cfgD[3], cfgF[3];
    uint32_t cfgPidPtr, pgPidPtr;   // currentPidProfile vs the PG record
    uint8_t cfgPidIndex, pgRollP;
    // RPM filter as the firmware sees it: per-motor mechanical frequency that
    // feeds the notches, the notch frequencies it ends up using (motor 0, all
    // harmonics), the config it derives them from, and the scheduler's dt
    // compensation factor.
    float rpmMotorHz[4];
    float rpmNotchHz[3];
    uint8_t rpmHarmonics;
    uint8_t rpmMinHz;
    uint16_t rpmQ;
    uint16_t rpmFadeRangeHz;
    uint16_t rpmLpfHz;
    uint8_t rpmWeight[3];
    float cycleTimeMultiplier;
    // The RPM exactly as the host sent it (before the firmware-style lowpass),
    // so an input-data problem is visible next to what the filter derived.
    float rpmRawMotorHz[4];
} sitl_local_loop_state_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Boot the flight controller (equivalent to starting betaflight_SITL.exe).
 * Call once from the host process before the first step.
 */
SITL_LOCAL_API int sitl_local_init(void);

/**
 * Advance the virtual clock by dtUs and run one scheduler pass.
 * dtUs is normally 1000 (1 ms) for a 1000 Hz host tick.
 * Sensors/RC are taken from `in`, motor outputs are returned in `out`.
 */
SITL_LOCAL_API void sitl_local_step(const sitl_local_input_t *in, uint32_t dtUs,
                                    sitl_local_output_t *out);

/** Current virtual time in microseconds (the FC clock). */
SITL_LOCAL_API uint64_t sitl_local_time_us(void);

// Fill `out` with the current control-loop signals (0 on success).
SITL_LOCAL_API int sitl_local_get_loop_state(sitl_local_loop_state_t *out);

/**
 * Synchronous access to the in-memory flight-controller state, for hosts that
 * want to read/write the same structures the configurator sees over MSP
 * without touching the serial stream. All values below are the exact same
 * memory the MSP handlers read/write, so both sides stay consistent.
 */

/** Bitmask of arming-disable reasons; same value as MSP_STATUS_EX. 0 = may arm. */
SITL_LOCAL_API uint32_t sitl_local_get_arming_flags(void);

/** True while arming is currently blocked by any arming-disable reason. */
SITL_LOCAL_API bool sitl_local_is_arming_disabled(void);

/** True when the FC is armed (same flag as the configurator's status icon). */
SITL_LOCAL_API bool sitl_local_get_armed(void);

/**
 * Return the aircraft to the disarmed state - the same effect as the
 * configurator's Disarm button, i.e. disarm(DISARM_REASON_ARMING_DISABLED).
 * Safe to call at any time and idempotent: it only acts when the FC is
 * actually armed, and it never leaves an arming-disable flag behind, so the
 * craft can arm again normally. Arming is still driven by the RC ARM switch,
 * so a host that keeps the ARM channel high must release it too, otherwise the
 * scheduler simply arms again on its next pass.
 * Returns 0, or -1 when the FC is not running.
 */
SITL_LOCAL_API int sitl_local_disarm(void);

/** flightModeFlags bitmask (ANGLE/HORIZON/MAG/... active modes). */
SITL_LOCAL_API uint32_t sitl_local_get_flight_modes(void);

/**
 * Read a rate profile. index in [0, CONTROL_RATE_PROFILE_COUNT) selects that
 * profile; any other value reads the current profile. Each output is a
 * 3-element array in axis order ROLL, PITCH, YAW (same columns as the
 * configurator's Rates tab). Units always match the Rates tab for the current
 * rate mode: rcRate is the RC Rate column, rcExpo the Expo column and
 * superRate the Super Rate / max-velocity column (e.g. deg/s in ACTUAL mode).
 * Any output pointer may be NULL to skip that group.
 */
SITL_LOCAL_API void sitl_local_get_rate(int index,
                                        float rcRate[3], float rcExpo[3],
                                        float superRate[3]);

/**
 * Write the current rate profile per axis (arrays in ROLL, PITCH, YAW order,
 * same display units as get_rate) and persist it to the virtual EEPROM via
 * the background thread. Each pointer may be NULL to leave that axis group
 * unchanged.
 */
SITL_LOCAL_API void sitl_local_set_rate(const float rcRate[3],
                                        const float rcExpo[3],
                                        const float superRate[3]);

/**
 * Rate mode of the current profile (rates_type): 0 = BETAFLIGHT,
 * 1 = RACEFLIGHT, 2 = KISS, 3 = ACTUAL, 4 = QUICK - the same values the
 * configurator's Rates tab stores. Returns -1 before sitl_local_init().
 */
SITL_LOCAL_API int sitl_local_get_rate_mode(void);

/**
 * Set the rate mode of the current profile and persist it. Accepts the same
 * values as get_rate_mode (0..RATES_TYPE_COUNT-1). Returns 0 on success,
 * -1 for an invalid mode.
 */
SITL_LOCAL_API int sitl_local_set_rate_mode(int mode);

/**
 * Locate the ARM switch in the mode-activation conditions. On success
 * auxChannel is the RC channel index (4 = AUX1) and startStep/endStep the
 * 25us-step range (900..2100us). When no ARM condition is configured,
 * auxChannel is set to 0xFF and the steps to 0.
 */
SITL_LOCAL_API void sitl_local_get_arm_switch(uint8_t *auxChannel,
                                              uint8_t *startStep,
                                              uint8_t *endStep);

/**
 * Bind BOXARM to an aux channel and 25 us-step range and persist it (same
 * semantics as the configurator's MSP_SET_MODE_RANGE). auxChannel follows the
 * get_arm_switch convention (the raw auxChannelIndex stored in the condition,
 * e.g. the value GetSitlArmSwitch returns); 0xFF clears the ARM condition.
 * Returns 0 on success, -1 when not initialized, on invalid parameters, or
 * when no mode-condition slot is free.
 */
SITL_LOCAL_API int sitl_local_set_arm_switch(uint8_t auxChannel,
                                             uint8_t startStep,
                                             uint8_t endStep);

// ---------------------------------------------------------------------------
// Generic mode ranges (any box, keyed by its permanent ID)
//
// permanentId is the ID the firmware uses on the wire (msp_box.c), e.g.
// 0 = ARM, 1 = ANGLE, 2 = HORIZON, 28 = AIR MODE, 29 = 3D DISABLE,
// 35 = FLIP OVER AFTER CRASH. Ranges are 25 us steps from 900 us, the same
// encoding MSP_MODE_RANGES / the Configurator's Modes tab use.
// ---------------------------------------------------------------------------

/** Read the first condition of `permanentId`. auxChannel is 0xFF when the box
 *  has no condition; startStep/endStep are its 25 us-step range. */
SITL_LOCAL_API void sitl_local_get_mode_range(int permanentId,
                                              uint8_t *auxChannel,
                                              uint8_t *startStep,
                                              uint8_t *endStep);

/** Replace every condition of `permanentId` with one range and persist it
 *  (same path as MSP_SET_MODE_RANGE). auxChannel = 0xFF clears the box.
 *  Returns 0 on success, -1 when not running / unknown box / invalid range /
 *  no free condition slot. */
SITL_LOCAL_API int sitl_local_set_mode_range(int permanentId,
                                             uint8_t auxChannel,
                                             uint8_t startStep,
                                             uint8_t endStep);

/** 1 while the box is active for the current RC inputs, else 0. */
SITL_LOCAL_API int sitl_local_get_mode_active(int permanentId);

/** 1 while CRASH FLIP is actually engaged, i.e. the flight controller latched
 *  it when the craft armed and the box is still active.
 *
 *  This is the latched state, not the raw switch: crash flip only arms when the
 *  box is already ON at the arm edge (Betaflight captures
 *  `crashFlipModeActive` inside tryArm()), and it is cleared by disarming or by
 *  switching the box off. A host that flips the switch *after* arming gets 0
 *  here - and, more importantly, the motors do not move. */
SITL_LOCAL_API int sitl_local_is_crash_flip_active(void);

/** FEATURE_3D state (the Configurator's "3D" feature bit). */
SITL_LOCAL_API int sitl_local_get_3d_enabled(void);

/** Enable/disable FEATURE_3D, re-derive the active box ids and the mixer
 *  profile, and persist it. Refused while armed (returns -1). */
SITL_LOCAL_API int sitl_local_set_3d_enabled(int enabled);

/**
 * Redirect the blackbox log directory at runtime (e.g. one folder per
 * aircraft in UE). The directory is created if missing, and the log-number
 * scan is re-run for that folder so LOG00001.BFL numbering is correct and
 * existing logs are never overwritten. Returns 0 on success, -1 for a NULL,
 * too-long or invalid path. An *empty* path restores the default
 * (BF_SITL_BLACKBOX_DIR if set, else %LOCALAPPDATA%\Betaflight-SITL\blackbox).
 * Call it before arming / logging starts; a log that is already in progress
 * keeps writing to its original file, and no process/DLL restart is needed.
 */
SITL_LOCAL_API int sitl_local_set_blackbox_dir(const char *path);

/**
 * Keep at most `maxLogs` blackbox logs in the log directory (0 = unlimited).
 * The oldest logs are deleted immediately and again whenever a new log is
 * opened (default 10; BF_SITL_BLACKBOX_MAX_LOGS overrides the default).
 * Returns 0 on success, -1 for an out-of-range value.
 */
SITL_LOCAL_API int sitl_local_set_blackbox_max_logs(int maxLogs);

/**
 * Copy the directory the virtual blackbox writes LOG*.BFL into (the platform
 * default is %LOCALAPPDATA%\Betaflight-SITL\blackbox, overridable with
 * BF_SITL_BLACKBOX_DIR). Returns 0 on success, -1 if `out` is too small.
 */
SITL_LOCAL_API int sitl_local_get_blackbox_dir(char *out, int size);

/**
 * Point the virtual EEPROM at another file (one per aircraft). An empty or
 * NULL path restores the LOCAL default
 * (%LOCALAPPDATA%\Betaflight-SITL\eeprom.bin). The switch is applied by the
 * next sitl_local_step(): the aircraft being left is saved into the file it
 * came from, then the new file is opened and read. A path that does not exist
 * yet is created with factory defaults, so a new aircraft does not inherit
 * the previous one's tune. Returns 0 when the switch was queued, or -1 for a
 * NULL path or a path that is too long.
 */
SITL_LOCAL_API int sitl_local_set_eeprom_path(const char *path);

// ---------------------------------------------------------------------------
// OSD character grid (the flight controller's real OSD output).
//
// Betaflight draws its OSD into a character canvas and pushes it out through a
// "displayport". With USE_MSP_DISPLAYPORT that stream is a series of
// MSP_DISPLAYPORT (182) frames carrying MSP_DP_CLEAR_SCREEN /
// MSP_DP_WRITE_STRING / MSP_DP_SYS subcommands - the exact bytes a digital VTX
// or goggle renders. The LOCAL link decodes that stream in-process and mirrors
// the resulting canvas here, so a host can draw the very same OSD (same
// element positions, same warnings, same blink) instead of guessing.
//
// The canvas size follows the flight controller's own configuration
// (vcd_video_system + osd_canvas_width/height, as set in the Configurator's OSD
// tab), so a host should read cols/rows from the frame rather than assuming
// 53x20 or 30x16.
// ---------------------------------------------------------------------------

/** Largest canvas the host API can carry: the OSD X coordinate is 6 bits
 *  (5 + the HD extension bit) and Y is 5 bits, so 64x32 covers every layout
 *  the firmware can produce, including HD 53x20 and PAL 30x16. */
#define SITL_LOCAL_OSD_MAX_COLS 64
#define SITL_LOCAL_OSD_MAX_ROWS 32
#define SITL_LOCAL_OSD_MAX_CELLS (SITL_LOCAL_OSD_MAX_COLS * SITL_LOCAL_OSD_MAX_ROWS)

/** Cell kind, so a host can tell a glyph from a goggle-drawn system element. */
typedef enum {
    SITL_LOCAL_OSD_CELL_BLANK = 0,  // nothing drawn here
    SITL_LOCAL_OSD_CELL_CHAR = 1,   // `character` indexes the MAX7456 font
    SITL_LOCAL_OSD_CELL_SYS = 2,    // `character` is a displayPortSystemElement_e
} sitl_local_osd_cell_kind_e;

/** Video system as stored in vcd_video_system (pg/vcd.h). */
typedef enum {
    SITL_LOCAL_OSD_VIDEO_AUTO = 0,
    SITL_LOCAL_OSD_VIDEO_PAL = 1,
    SITL_LOCAL_OSD_VIDEO_NTSC = 2,
    SITL_LOCAL_OSD_VIDEO_HD = 3,
} sitl_local_osd_video_system_e;

/** One complete OSD canvas. Arrays are row-major, index = row * cols + col,
 *  and only the first cols*rows entries are meaningful. */
typedef struct {
    uint8_t cols;                 // active canvas width in characters
    uint8_t rows;                 // active canvas height in characters
    uint8_t videoSystem;          // sitl_local_osd_video_system_e
    uint8_t displayPortDevice;    // osdDisplayPortDevice_e the FC selected
    uint8_t active;               // 1 while the FC is actually running the OSD
    uint8_t reserved[3];
    uint32_t revision;            // bumped on every canvas change
    uint32_t frames;              // MSP_DP_DRAW_SCREEN count since boot
    uint8_t kind[SITL_LOCAL_OSD_MAX_CELLS];       // sitl_local_osd_cell_kind_e
    uint8_t character[SITL_LOCAL_OSD_MAX_CELLS];  // font index or system element
    /** The attribute byte exactly as displayport_msp.c puts it on the wire:
     *  bits 0-1 = font/severity bank (displayPortSeverity_e), bit 6 = blink
     *  (DISPLAYPORT_MSP_ATTR_BLINK). */
    uint8_t attribute[SITL_LOCAL_OSD_MAX_CELLS];
} sitl_local_osd_frame_t;

/** Copy the current OSD canvas into `out`. Returns 0 on success, -1 when the
 *  FC is not running. Safe to call from the host's stepping thread. */
SITL_LOCAL_API int sitl_local_get_osd_frame(sitl_local_osd_frame_t *out);

/** Canvas revision: changes whenever the grid content changes. A host can poll
 *  this cheaply and only re-read the full frame when it moves. Returns 0 when
 *  the FC is not running. */
SITL_LOCAL_API uint32_t sitl_local_get_osd_revision(void);

/** Number of MSP_DP_DRAW_SCREEN commits since boot (the OSD frame counter). */
SITL_LOCAL_API uint32_t sitl_local_get_osd_frame_count(void);

// ---------------------------------------------------------------------------
// Captured font (the one the configurator flashed)
//
// Flashing a font in the Configurator's OSD tab sends one MSP_OSD_CHAR_WRITE
// per character; the firmware forwards each to the displayport, where this
// build's wrapper caches it. A host can therefore render the OSD with the font
// the user actually flashed instead of its own .mcm fallback.
//
// The layout is character-major: SITL_LOCAL_OSD_FONT_CHAR_COUNT characters of
// SITL_LOCAL_OSD_FONT_CHAR_BYTES bytes each, i.e. 256 * 54 = 13824 bytes.
// These are the *visible* bytes (drivers/osd.h's OSD_CHAR_VISIBLE_BYTES, 12x18
// at 2 bits per pixel); the 10 trailing metadata bytes the OSD drivers may
// carry are not part of the upload and are not part of this buffer.
//
// Only the HD/MSP displayport path can capture a font: an SD OSD goes through
// the MAX7456 driver, which never writes through this vtable.
// ---------------------------------------------------------------------------

#define SITL_LOCAL_OSD_FONT_CHAR_COUNT 256
#define SITL_LOCAL_OSD_FONT_CHAR_BYTES 54
#define SITL_LOCAL_OSD_FONT_BYTES (SITL_LOCAL_OSD_FONT_CHAR_COUNT * SITL_LOCAL_OSD_FONT_CHAR_BYTES)

/** Revision of the captured font, or 0 while no *complete* font is available
 *  (before the first upload ever, and for the whole duration of an upload -
 *  a host should keep its fallback until this becomes non-zero). The value
 *  changes whenever a character is written, so "different from last time"
 *  means "read it again". */
SITL_LOCAL_API uint32_t sitl_local_get_osd_font_revision(void);

/** Copy the captured font into `out` (which must hold at least
 *  SITL_LOCAL_OSD_FONT_BYTES). Returns the number of bytes written (13824), or
 *  0 when there is no complete font yet or the buffer is too small - in which
 *  case nothing is written. */
SITL_LOCAL_API int sitl_local_get_osd_font(char *out, int size);

/** Why the OSD is (or is not) running - diagnostics for host bring-up. */
typedef struct {
    uint8_t osdFeatureEnabled;    // featureIsEnabled(FEATURE_OSD)
    uint8_t displayPortReady;     // osdGetDisplayPort(NULL) != NULL
    uint8_t wrapped;              // our capture vtable is installed
    uint8_t videoSystem;          // vcd_video_system
    uint8_t displayPortDevice;    // osdConfig()->displayPortDevice
    uint8_t osdTaskEnabled;       // reserved
    uint8_t reserved[2];
    uint8_t cols;
    uint8_t rows;
    uint32_t frameCount;
    uint32_t revision;
    /** How often the firmware actually drove the displayport. If these stay 0
     *  while frameCount rises, the OSD state machine is running but drawing no
     *  elements (all elements hidden in the active OSD profile). */
    uint32_t clearCalls;
    uint32_t writeStringCalls;
    uint32_t writeCharCalls;
    uint32_t writeSysCalls;
    /** Puts accepted (in-bounds) and the largest number of non-blank cells seen
     *  at a publish. If acceptedPuts > 0 but maxCellsAtPublish is 0, the content
     *  is being drawn and then cleared before the frame is committed. */
    uint32_t acceptedPuts;
    uint32_t maxCellsAtPublish;
    uint8_t sampleX;
    uint8_t sampleY;
    uint8_t sampleChar;
    uint8_t reserved2;
    /** featureConfig()->enabledFeatures as the FC sees it, and whether the OSD
     *  bit is set in it. configurationState is 1 = CONFIGURED. */
    uint32_t featureMask;
    uint8_t configurationState;
    uint8_t osdBitStored;
    uint8_t reserved3[2];
    /** The mask the EEPROM carried BEFORE sitlOsdEnableFeature() ran, and
     *  whether that hook is what turned the OSD on. Distinguishes "the stored
     *  config already had FEATURE_OSD" from "the hook enabled it". */
    uint32_t storedFeatureMaskBeforeHook;
    uint8_t hookEnabledOsd;
    uint8_t reserved4[3];
} sitl_local_osd_diag_t;

SITL_LOCAL_API void sitl_local_get_osd_diag(sitl_local_osd_diag_t *out);

// ---------------------------------------------------------------------------
// OSD element configuration (same records the Configurator's OSD tab writes).
//
// These write the firmware's osdElementConfig through the same path
// MSP_SET_OSD_CONFIG uses, then re-analyse the active elements so the next
// rendered canvas reflects the change. A host can therefore lay out the OSD
// itself and see the UE HUD follow.
// ---------------------------------------------------------------------------

/** OSD element indices (osd_items_e in the firmware).
 *
 *  These are a stable, commonly-used subset. The values are checked against the
 *  firmware's own OSD_* constants by static_assert in src/sitl_osd.c, so if a
 *  Betaflight update ever renumbers them the build fails loudly instead of
 *  silently placing elements in the wrong slots. Any index in
 *  [0, SITL_LOCAL_OSD_ITEM_COUNT) is accepted by the setters. */
typedef enum {
    SITL_LOCAL_OSD_ITEM_RSSI_VALUE = 0,
    SITL_LOCAL_OSD_ITEM_MAIN_BATT_VOLTAGE = 1,
    SITL_LOCAL_OSD_ITEM_CROSSHAIRS = 2,
    SITL_LOCAL_OSD_ITEM_ARTIFICIAL_HORIZON = 3,
    SITL_LOCAL_OSD_ITEM_HORIZON_SIDEBARS = 4,
    SITL_LOCAL_OSD_ITEM_TIMER_1 = 5,
    SITL_LOCAL_OSD_ITEM_TIMER_2 = 6,
    SITL_LOCAL_OSD_ITEM_FLYMODE = 7,
    SITL_LOCAL_OSD_ITEM_CRAFT_NAME = 8,
    SITL_LOCAL_OSD_ITEM_THROTTLE_POS = 9,
    SITL_LOCAL_OSD_ITEM_VTX_CHANNEL = 10,
    SITL_LOCAL_OSD_ITEM_CURRENT_DRAW = 11,
    SITL_LOCAL_OSD_ITEM_MAH_DRAWN = 12,
    SITL_LOCAL_OSD_ITEM_GPS_SPEED = 13,
    SITL_LOCAL_OSD_ITEM_GPS_SATS = 14,
    SITL_LOCAL_OSD_ITEM_ALTITUDE = 15,
    SITL_LOCAL_OSD_ITEM_ROLL_PIDS = 16,
    SITL_LOCAL_OSD_ITEM_PITCH_PIDS = 17,
    SITL_LOCAL_OSD_ITEM_YAW_PIDS = 18,
    SITL_LOCAL_OSD_ITEM_POWER = 19,
    SITL_LOCAL_OSD_ITEM_PIDRATE_PROFILE = 20,
    SITL_LOCAL_OSD_ITEM_WARNINGS = 21,
    SITL_LOCAL_OSD_ITEM_AVG_CELL_VOLTAGE = 22,
    SITL_LOCAL_OSD_ITEM_GPS_LON = 23,
    SITL_LOCAL_OSD_ITEM_GPS_LAT = 24,
    SITL_LOCAL_OSD_ITEM_DEBUG = 25,
    SITL_LOCAL_OSD_ITEM_PITCH_ANGLE = 26,
    SITL_LOCAL_OSD_ITEM_ROLL_ANGLE = 27,
    SITL_LOCAL_OSD_ITEM_MAIN_BATT_USAGE = 28,
    SITL_LOCAL_OSD_ITEM_DISARMED = 29,
    SITL_LOCAL_OSD_ITEM_HOME_DIR = 30,
    SITL_LOCAL_OSD_ITEM_HOME_DIST = 31,
    SITL_LOCAL_OSD_ITEM_NUMERICAL_HEADING = 32,
    SITL_LOCAL_OSD_ITEM_NUMERICAL_VARIO = 33,
    SITL_LOCAL_OSD_ITEM_COMPASS_BAR = 34,
    SITL_LOCAL_OSD_ITEM_ESC_TMP = 35,
    SITL_LOCAL_OSD_ITEM_ESC_RPM = 36,
    SITL_LOCAL_OSD_ITEM_REMAINING_TIME_ESTIMATE = 37,
    SITL_LOCAL_OSD_ITEM_RTC_DATETIME = 38,
    SITL_LOCAL_OSD_ITEM_ADJUSTMENT_RANGE = 39,
    SITL_LOCAL_OSD_ITEM_CORE_TEMPERATURE = 40,
    SITL_LOCAL_OSD_ITEM_ANTI_GRAVITY = 41,
    SITL_LOCAL_OSD_ITEM_G_FORCE = 42,
    SITL_LOCAL_OSD_ITEM_MOTOR_DIAG = 43,
    SITL_LOCAL_OSD_ITEM_LOG_STATUS = 44,
    SITL_LOCAL_OSD_ITEM_FLIP_ARROW = 45,
    SITL_LOCAL_OSD_ITEM_LINK_QUALITY = 46,
    SITL_LOCAL_OSD_ITEM_FLIGHT_DIST = 47,
    SITL_LOCAL_OSD_ITEM_STICK_OVERLAY_LEFT = 48,
    SITL_LOCAL_OSD_ITEM_STICK_OVERLAY_RIGHT = 49,
    SITL_LOCAL_OSD_ITEM_PILOT_NAME = 50,
    SITL_LOCAL_OSD_ITEM_ESC_RPM_FREQ = 51,
    SITL_LOCAL_OSD_ITEM_RATE_PROFILE_NAME = 52,
    SITL_LOCAL_OSD_ITEM_PID_PROFILE_NAME = 53,
    SITL_LOCAL_OSD_ITEM_PROFILE_NAME = 54,
    SITL_LOCAL_OSD_ITEM_RSSI_DBM_VALUE = 55,
    SITL_LOCAL_OSD_ITEM_RC_CHANNELS = 56,
    SITL_LOCAL_OSD_ITEM_CAMERA_FRAME = 57,
    SITL_LOCAL_OSD_ITEM_EFFICIENCY = 58,
    SITL_LOCAL_OSD_ITEM_TOTAL_FLIGHTS = 59,
    SITL_LOCAL_OSD_ITEM_UP_DOWN_REFERENCE = 60,
    SITL_LOCAL_OSD_ITEM_TX_UPLINK_POWER = 61,
    SITL_LOCAL_OSD_ITEM_WATT_HOURS_DRAWN = 62,
    SITL_LOCAL_OSD_ITEM_AUX_VALUE = 63,
    SITL_LOCAL_OSD_ITEM_READY_MODE = 64,
    SITL_LOCAL_OSD_ITEM_RSNR_VALUE = 65,
    SITL_LOCAL_OSD_ITEM_SYS_GOGGLE_VOLTAGE = 66,
    SITL_LOCAL_OSD_ITEM_SYS_VTX_VOLTAGE = 67,
    SITL_LOCAL_OSD_ITEM_SYS_BITRATE = 68,
    SITL_LOCAL_OSD_ITEM_SYS_DELAY = 69,
    SITL_LOCAL_OSD_ITEM_SYS_DISTANCE = 70,
    SITL_LOCAL_OSD_ITEM_SYS_LQ = 71,
    SITL_LOCAL_OSD_ITEM_SYS_GOGGLE_DVR = 72,
    SITL_LOCAL_OSD_ITEM_SYS_VTX_DVR = 73,
    SITL_LOCAL_OSD_ITEM_SYS_WARNINGS = 74,
    SITL_LOCAL_OSD_ITEM_SYS_VTX_TEMP = 75,
    SITL_LOCAL_OSD_ITEM_SYS_FAN_SPEED = 76,
    SITL_LOCAL_OSD_ITEM_GPS_LAP_TIME_CURRENT = 77,
    SITL_LOCAL_OSD_ITEM_GPS_LAP_TIME_PREVIOUS = 78,
    SITL_LOCAL_OSD_ITEM_GPS_LAP_TIME_BEST3 = 79,
    SITL_LOCAL_OSD_ITEM_DEBUG2 = 80,
    SITL_LOCAL_OSD_ITEM_CUSTOM_MSG0 = 81,
    SITL_LOCAL_OSD_ITEM_CUSTOM_MSG1 = 82,
    SITL_LOCAL_OSD_ITEM_CUSTOM_MSG2 = 83,
    SITL_LOCAL_OSD_ITEM_CUSTOM_MSG3 = 84,
    SITL_LOCAL_OSD_ITEM_LIDAR_DIST = 85,
    SITL_LOCAL_OSD_ITEM_CUSTOM_SERIAL_TEXT = 86,
    SITL_LOCAL_OSD_ITEM_BATTERY_PROFILE_NAME = 87,
    /** Number of element slots named here. The firmware's own OSD_ITEM_COUNT can
     *  be larger (waypoint/nav-map items are appended after this when the GPS
     *  flight-plan features are enabled), and any index in
     *  [0, OSD_ITEM_COUNT) is accepted by the setters. */
    SITL_LOCAL_OSD_ITEM_NAMED_COUNT = 88,
} sitl_local_osd_item_e;

/** Place an OSD element and mark it visible in every OSD profile (pass
 *  visible=0 to hide it). Coordinates are canvas characters and are clamped to
 *  the current canvas. The change is applied to the running OSD immediately.
 *  Returns 0 on success, -1 when the OSD is not running or the item is out of
 *  range. */
SITL_LOCAL_API int sitl_local_set_osd_element(int item, int col, int row, int visible);

/** Read back an element's position/visibility. Any output pointer may be NULL.
 *  Returns 0 on success, -1 when the OSD is not running or the item is out of
 *  range. */
SITL_LOCAL_API int sitl_local_get_osd_element(int item, int *col, int *row, int *visible);

/** Force the OSD to re-analyse its active elements (e.g. after changing several
 *  positions in one go). */
SITL_LOCAL_API void sitl_local_osd_refresh(void);


/**
 * Re-read the EEPROM at the current path and re-apply the configuration
 * without restarting the process: the LOCAL link overrides (virtual UDP
 * receiver, ADC battery shims, virtual PWM motor backend), the boot-time
 * derived state (mixer/motor/servo setup, gyro filters, debug mode) and the
 * PG config itself. A pending sitl_local_set_eeprom_path() is applied first.
 * The work runs on the next sitl_local_step() call, between steps, so it never
 * races the flight loop. Returns 0 when the reload was queued, -1 when the FC
 * is not running or is currently armed (disarm first, or the request is
 * applied on the first step after disarming).
 */
SITL_LOCAL_API int sitl_local_reload_config(void);

/** Stop the background MSP thread. Does not exit the host process. */
SITL_LOCAL_API void sitl_local_shutdown(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // SRC_SITL_LOCAL_H
