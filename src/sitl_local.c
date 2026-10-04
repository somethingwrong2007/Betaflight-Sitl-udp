/**
 * In-process local link (SITL_LINK_MODE=LOCAL).
 *
 * The host engine calls sitl_local_step() synchronously at its own tick rate
 * (normally 1000 Hz). The virtual clock is advanced by the caller-supplied
 * dt, one scheduler pass runs gyro/filter/PID, and the motor outputs for that
 * exact state are returned in the same call - no UDP, no stale reads.
 *
 * Sensor feeds mirror sitl.c updateState() conventions (Gazebo bridge): FRD
 * angular velocity/acceleration, Gazebo-format quaternion, ENU velocity and
 * lon/lat/alt position.
 */

#ifdef SITL_LOCAL

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <setjmp.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/accgyro/accgyro_virtual.h"
#include "drivers/barometer/barometer_virtual.h"
#include "drivers/compass/compass_virtual.h"
#include "drivers/dma.h"
#include "drivers/dshot.h"
#include "io/gps_virtual.h"
#include "fc/core.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "fc/rc_modes.h"
#include "scheduler/scheduler.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "sitl_gyro.h"
#include "config/config.h"
#include "config/config_streamer_impl.h"
#include "fc/runtime_config.h"
#include "config/feature.h"
#include "msp/msp.h"
#include "rx/rx.h"
#include "sensors/battery.h"
#ifdef USE_BLACKBOX
#include "blackbox/blackbox.h"
#endif

#include "sitl_local.h"
#include "sim_telemetry.h"

#include <windows.h>

#ifndef USE_GPS_LAP_TIMER
#include "pg/gps_lap_timer.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define LOCAL_RAD2DEG  (180.0 / M_PI)
#define LOCAL_ACC_SCALE (256 / 9.80665)
#define LOCAL_GYRO_SCALE (16.4)

// Boot sequence lives in main_windows.c so both the standalone exe and the
// DLL share the same initialization.
extern void sitlBoot(int argc, char *argv[]);

// wincompat.c: stepped virtual clock used by the scheduler.
extern void sitlStepTime(uint64_t stepUs);
extern uint64_t micros64(void);

// Firmware entry points used by the synchronous step.
extern void scheduler(void);
extern void rxInit(void);
extern void sitlAuditLog(const char *fmt, ...);
extern bool useDshotTelemetry;
extern bool cliMode;
extern void writeEEPROM(void);

// msp_serial.h pulls in io/serial.h, which collides with the MinGW windows.h
// include chain; declare just what the background MSP keep-alive needs (the
// enum layout matches msp_serial.h).
typedef enum {
    LOCAL_MSP_EVALUATE_NON_MSP_DATA = 0,
    LOCAL_MSP_SKIP_NON_MSP_DATA
} localMspEvaluateNonMspData_e;
// mspSerialProcess is invoked from two threads (scheduler TASK_SERIAL inside
// sitl_local_step and the background thread below). The shared MSP/CLI parser
// in msp_serial.c is not reentrant - concurrent calls can leave the port
// wedged in CLI mode - so the real implementation is renamed and both callers
// go through this mutex-serialized wrapper.
extern void sitlMspSerialProcessReal(localMspEvaluateNonMspData_e evaluateNonMspData,
                                     mspProcessCommandFnPtr mspProcessCommandFn,
                                     mspProcessReplyFnPtr mspProcessReplyFn);

static CRITICAL_SECTION gMspCrit;
static volatile LONG gMspThreadId = 0;

// Diagnostics: how much of the host's CPU the MSP/configurator link burns.
static volatile LONG64 gMspBusyUs = 0;
static volatile LONG gMspProcessCalls = 0;

void sitlLocalMspLoad(uint32_t *busyUs, uint32_t *calls)
{
    if (busyUs) { *busyUs = (uint32_t)InterlockedCompareExchange64(&gMspBusyUs, 0, 0); }
    if (calls)  { *calls = (uint32_t)InterlockedCompareExchange(&gMspProcessCalls, 0, 0); }
}

void mspSerialProcess(localMspEvaluateNonMspData_e evaluateNonMspData,
                      mspProcessCommandFnPtr mspProcessCommandFn,
                      mspProcessReplyFnPtr mspProcessReplyFn)
{
    // Only the dedicated background thread processes MSP/CLI. The scheduler's
    // TASK_SERIAL also calls this (inside the UE thread when the host steps);
    // returning immediately here guarantees the UE thread can never block on
    // the parser, the mutex, CLI processing or serial I/O.
    if (GetCurrentThreadId() != (DWORD)gMspThreadId) {
        return;
    }
    EnterCriticalSection(&gMspCrit);
    extern uint64_t micros64_real(void);
    const uint64_t mspStartUs = micros64_real();
    sitlMspSerialProcessReal(evaluateNonMspData, mspProcessCommandFn, mspProcessReplyFn);
    InterlockedExchangeAdd64(&gMspBusyUs, (LONG64)(micros64_real() - mspStartUs));
    InterlockedIncrement(&gMspProcessCalls);
    LeaveCriticalSection(&gMspCrit);
}

// udplink_windows.c captures the motor packets pwmCompleteMotorUpdate()
// produces so the DLL can return them without any network I/O.
extern void sitlLocalCaptureMotorPacket(const void *data, size_t size);
extern bool sitlLocalTakeMotorPacket(void *out, size_t size);

static bool gLocalRunning = false;
static HANDLE gMspThread = NULL;
static volatile LONG gMspThreadStop = 0;
// Boots seen by this process: 1 = the loader-zeroed fresh boot, >1 = an
// in-process restart that re-uses the previous run's RAM.
static uint32_t gLocalBootCount = 0;

// Reboot recovery for the in-process build. msp.c's mspRebootFn (MSP_SET_REBOOT
// post-processing) ends with `while (true);` because a real reboot never
// returns; LOCAL mode's systemReset() defers the EEPROM persist and returns, so
// the stock function would spin forever on the background MSP thread and every
// later configurator connection would time out. sitlSystemReset() longjmps back
// to this loop instead (the parser is already back in PORT_IDLE when the
// reboot handler runs), keeping the MSP/CLI thread alive across the reboot.
static jmp_buf gMspLoopJmp;
static volatile LONG gMspJmpReady = 0;

void sitlLocalRequestReset(void);

void sitlLocalRebootJump(void)
{
    if (!InterlockedCompareExchange(&gMspJmpReady, 0, 0)
        || GetCurrentThreadId() != (DWORD)gMspThreadId) {
        // MSP loop not armed yet (boot-time reboot before the thread started):
        // or the reboot came from a non-MSP thread (e.g. mavlink/CMS on the
        // host thread). longjmp can only unwind the MSP thread's own stack, so
        // fall back to the deferred-persist path and return.
        sitlLocalRequestReset();
        return;
    }
    // The MSP wrapper's critical section is held by this thread across the
    // whole MSP/CLI processing call; release it so the loop can re-enter it
    // cleanly on the next iteration (the wrapper's own Leave is skipped by the
    // jump).
    LeaveCriticalSection(&gMspCrit);
    longjmp(gMspLoopJmp, 1);
}

static double gGpsOriginLat = 0.0;
static double gGpsOriginLon = 0.0;
static bool gGpsOriginSet = false;

// RC is a "latest value cache": the read callback returns the cache, and the
// frame status presents a fixed 125 Hz cadence (one COMPLETE every 8 ms of
// virtual time, PENDING in between) like a real receiver. Always-COMPLETE
// made the RX task and feedforward run at the 1 kHz scheduler rate, so a
// stick snap produced two huge 1 ms feedforward samples instead of a
// 125 Hz impulse spread over ~16 ms, inflating the setpoint-speed peak and
// causing overshoot. The cache is refreshed whenever the host data changes;
// lastRcFrameTimeUs is stamped at frame ticks only.
static uint16_t gLocalRc[SITL_LOCAL_MAX_RC_CHANNELS];
static bool gLocalRcValid = false;
// The announce is *self-clocked*: it fires as soon as 8 ms of virtual time have
// passed since the last frame, whatever step the firmware's frame check happens
// to run on. A step-indexed grid (announce only when step % 8 == 0) silently
// skips frames whenever the check does not land on a multiple of 8 - the
// measured RC rate then drops and rc_smoothing's automatic setpoint cutoff is
// computed from the wrong rate. The anchor is the init moment, so an in-process
// restart (where the virtual clock keeps running) still yields the same frame
// grid as a fresh boot.
static uint64_t gLocalRcAnnounceUs = 0;
// Frames announced so far - logged in the audit trail (~125 per second of
// virtual time at the 1 kHz host rate).
static uint32_t gLocalRcFrameCount = 0;
// Last motor values handed to the host (step diagnostics / audit log).
static float gLocalLastMotors[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
// Last raw gyro values fed to the virtual gyro device (for the burst recorder;
// sensors/gyro.h cannot be included here, see the LOCAL mpuGyroReadRegister stub).
static int16_t gLocalLastGyroRaw[3] = { 0, 0, 0 };

// --- burst recorder -----------------------------------------------------------
// The audit trail samples once a second, which cannot show a tremor. Once every
// 12 s this records 2 s of 1 kHz control-loop data (gyro, the PID term
// contributions, the motors, attitude and the active modes) so a real
// oscillation can be analysed: which term drives it, how big it is and what the
// motor response looks like.
#define SITL_BURST_STEPS   2000
#define SITL_BURST_PERIOD  12000

static bool localBurstLog(void)
{
    static uint32_t stepsSinceBurst = SITL_BURST_PERIOD;
    static uint32_t burstStep = 0;
    static FILE *fp = NULL;

    if (fp == NULL) {
        if (++stepsSinceBurst < SITL_BURST_PERIOD) {
            return false;
        }
        const char *appData = getenv("LOCALAPPDATA");
        if (appData == NULL) {
            return false;
        }
        char path[MAX_PATH];
        _snprintf(path, sizeof(path), "%s\\Betaflight-SITL\\sitl-burst.log", appData);
        fp = fopen(path, "w");
        if (fp == NULL) {
            return false;
        }
        fprintf(fp, "# t_us gyroR gyroP gyroY pR pP pY iR iP iY dR dP dY fR fP fY "
                    "sumR sumP sumY m0 m1 m2 m3 attR attP attY modes armFlags "
                    "pidDeltaUs gyroDeltaUs rpm0 rpm1 rpm2 rpm3 notch1 notch2 notch3 "
                    "gyroADCfR gyroADCfP gyroADCfY\n");
        stepsSinceBurst = 0;
        burstStep = 0;
    }

    extern void sitlLocalGetRpmNotchHz(float notchHz[3]);
    float notchHz[3] = { 0.0f, 0.0f, 0.0f };
    sitlLocalGetRpmNotchHz(notchHz);
    float rpmHz[4];
    float gyroADCf[3];
    extern void sitlLocalGetGyroState(float scaled[3], float filtered[3]);
    sitlLocalGetGyroState(NULL, gyroADCf);
    for (int i = 0; i < 4; i++) {
        rpmHz[i] = getMotorFrequencyHz((uint8_t)i);
    }

    fprintf(fp, "%.0f %.1f %.1f %.1f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f "
                "%.2f %.2f %.2f %.2f %.2f %.2f %.0f %.0f %.0f %.0f %d %d %d %04X %08X "
                "%d %d %.1f %.1f %.1f %.1f %.0f %.0f %.0f %.2f %.2f %.2f\n",
            (double)micros64(),
            (double)gLocalLastGyroRaw[0], (double)gLocalLastGyroRaw[1],
            (double)gLocalLastGyroRaw[2],
            (double)pidData[0].P, (double)pidData[1].P, (double)pidData[2].P,
            (double)pidData[0].I, (double)pidData[1].I, (double)pidData[2].I,
            (double)pidData[0].D, (double)pidData[1].D, (double)pidData[2].D,
            (double)pidData[0].F, (double)pidData[1].F, (double)pidData[2].F,
            (double)pidData[0].Sum, (double)pidData[1].Sum, (double)pidData[2].Sum,
            (double)gLocalLastMotors[0], (double)gLocalLastMotors[1],
            (double)gLocalLastMotors[2], (double)gLocalLastMotors[3],
            (int)attitude.values.roll, (int)attitude.values.pitch, (int)attitude.values.yaw,
            (unsigned)flightModeFlags, (unsigned)getArmingDisableFlags(),
            (int)getTaskDeltaTimeUs(TASK_PID), (int)getTaskDeltaTimeUs(TASK_GYRO),
            (double)rpmHz[0], (double)rpmHz[1], (double)rpmHz[2], (double)rpmHz[3],
            (double)notchHz[0], (double)notchHz[1], (double)notchHz[2],
            (double)gyroADCf[0], (double)gyroADCf[1], (double)gyroADCf[2]);

    if (++burstStep >= SITL_BURST_STEPS) {
        fclose(fp);
        fp = NULL;
    }
    return true;
}
// Mirror of cliMode seen by the MSP thread (a real FC drops the CLI arming
// block on reboot; LOCAL clears it when the CLI is left). File scope so a
// in-process restart starts with a clean value.
static bool gCliWasActive = false;

static uint8_t localRcFrameStatus(rxRuntimeState_t *state)
{
    (void)state;
    const uint64_t now = micros64();
    if (gLocalRcValid && (now - gLocalRcAnnounceUs) >= 8000) {
        gLocalRcAnnounceUs = now;
        gLocalRcFrameCount++;
        rxRuntimeState.lastRcFrameTimeUs = (timeUs_t)(now & 0xFFFFFFFF);
        return RX_FRAME_COMPLETE;
    }
    return RX_FRAME_PENDING;
}

uint32_t sitlLocalRcFrameCount(void)
{
    return gLocalRcFrameCount;
}

static float localRcReadRaw(const rxRuntimeState_t *state, uint8_t channel)
{
    (void)state;
    return channel < SITL_LOCAL_MAX_RC_CHANNELS ? (float)gLocalRc[channel] : 0.0f;
}

// Called from sitlBoot() after the EEPROM config is loaded and before
// initPhase3() runs motorDevInit(). Enabling USE_DSHOT (needed for the RPM
// bridge) makes the default motor protocol DSHOT600, whose hardware init is
// a false-returning stub in this build - the motor device would become the
// null device and produce no output. The virtual PWM device is the correct
// motor backend for SITL, so pin the protocol to PWM here.
//
// This is also the hook for every other LOCAL link override that the *boot's
// derived init* reads, because it runs after initPhase1 (the EEPROM read, which
// would overwrite PG records) and before initPhase2/3 (mixer/motor setup,
// gyroInitFilters -> rpmFilterInit, pidInit).
//
// `useDshotTelemetry` is one of those: rpmFilterInit() returns early - leaving
// the RPM filter disabled for the whole run, numHarmonics = 0 - when it is
// false, and the flag is a plain global (a fresh process starts with it at 0;
// the LOCAL link turns it on so the bridged motor RPM feeds the RPM filter, the
// OSD and MSP telemetry). Setting it only *after* the boot, as the runtime
// override used to, meant:
//
//   fresh process  -> boot sees 0 -> RPM filter OFF  -> no RPM notches
//   shutdown+init  -> global still 1 from the last run -> rpmFilterInit enables
//                     the RPM filter -> the gyro is notched -> the loop answers
//                     the same stick input with a different (up to 236 us, 24%
//                     of the range) motor response.
//
// That is the "the aircraft was fine, then saving/restarting made it shake, and
// only reloading the DLL fixes it" bug: the two boots derived different filter
// chains. Setting the flag here makes every boot - first and re-init - build the
// same chain, exactly like a real FC whose config enables the RPM filter.
void sitlLocalPreMotorInit(void)
{
    motorConfigMutable()->dev.motorProtocol = MOTOR_PROTOCOL_PWM;
    useDshotTelemetry = true;
}

// The LOCAL link owns a few settings that the stock firmware would take from
// the EEPROM: the virtual UDP receiver, the ADC battery shims that read
// sitl_local_step() telemetry and the virtual PWM motor backend. sitl.c's boot
// path applies them once (sitl_local_init + main_windows.c's
// sitlLocalPreMotorInit call); a runtime config reload replaces every PG
// record, so they must be re-applied afterwards or the FC would start running
// the stored hardware settings (e.g. DSHOT600, whose device init is a stub in
// this build, so the motors would stop).
// Install the virtual receiver: seed the UDP provider's channel count (rxInit()
// snapshots it into rx.c's file-static rxChannelCount, which the channel loops
// use as their bound - if it stays 0 no channel is ever read) and take over the
// provider callbacks with the local cache.
static void localTakeOverRcProvider(void)
{
    uint16_t initRc[SITL_LOCAL_MAX_RC_CHANNELS];
    for (int i = 0; i < SITL_LOCAL_MAX_RC_CHANNELS; i++) {
        initRc[i] = 1000;
    }
    rxUpdateUdpChannels(initRc, SITL_LOCAL_MAX_RC_CHANNELS);
    rxInit();
    rxRuntimeState.rcReadRawFn = localRcReadRaw;
    rxRuntimeState.rcFrameStatusFn = localRcFrameStatus;
    rxRuntimeState.channelCount = SITL_LOCAL_MAX_RC_CHANNELS;
}

// Declared before use: defined next to localRepinOverrides() below.
bool sitlLocalRcTakeOverActive(void);

static void localApplyLinkOverrides(void)
{
    // Simulated motor RPM participates in the firmware (RPM filter, motor
    // telemetry, OSD/MSP): mark DSHOT telemetry as active so the RPM filter
    // and telemetry consumers use the bridged values from the wrappers.
    useDshotTelemetry = true;

    // The FC blocks arming for powerOnArmingGraceTime seconds after every
    // boot; a reload must not re-block a session that is already running.
    unsetArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);

    // Sensor input arrives via sitl_local_step(), not a serial receiver.
    featureEnableImmediate(FEATURE_RX_UDP);

    // Only (re)install the virtual receiver when it is not ours any more.
    // localTakeOverRcProvider() re-runs rxUpdateUdpChannels() + rxInit(), which
    // resets the RX frame timing/processing state; doing that unconditionally on
    // every reload changed the stick-transient response (feedforward / rc
    // smoothing) even when the configuration was identical. The configurator
    // write path already worked this way (it re-takes over only when the
    // callbacks were replaced); the reload path uses this same helper, so it gets
    // the same rule.
    if (!sitlLocalRcTakeOverActive()) {
        localTakeOverRcProvider();
    }

    // Voltage/current arrive as telemetry, not from a real ADC input.
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
    batteryConfigMutable()->currentMeterSource = CURRENT_METER_ADC;

    // Pin the virtual PWM backend before the next motorDevInit().
    sitlLocalPreMotorInit();
}

// systemReset() defers the EEPROM persist here so it never runs on the UE
// thread (the scheduler may execute systemReset via TASK_SERIAL while the
// configurator exits the CLI panel). The background thread picks it up.
static volatile LONG gLocalPendingReset = 0;

// The configurator's Save writes settings over MSP (motor config, features,
// battery/ESC meters, mixer, board alignment, ...) and does so *without* a
// reboot, which can undo the runtime state the LOCAL link pins at boot: the
// virtual receiver takeover, FEATURE_RX_UDP, the ADC battery meter sources, the
// virtual PWM motor protocol and the DShot-telemetry flag. Everything below is
// a plain flag/mode write (no mixer, motor, servo or filter state), so it is
// safe to apply while armed; the request is queued from the MSP write path and
// executed on the host thread between steps.
static volatile LONG gLocalRepinPending = 0;

void sitlLocalRequestRepinOverrides(void)
{
    InterlockedExchange(&gLocalRepinPending, 1);
}

bool sitlLocalRcTakeOverActive(void)
{
    return gLocalRunning && rxRuntimeState.rcReadRawFn == localRcReadRaw;
}

static void localRepinOverrides(void)
{
    InterlockedExchange(&gLocalRepinPending, 0);

    const uint8_t protocolBefore = motorConfig()->dev.motorProtocol;
    const uint32_t featuresBefore = featureConfig()->enabledFeatures;
    const uint8_t voltageBefore = batteryConfig()->voltageMeterSource;
    const uint8_t currentBefore = batteryConfig()->currentMeterSource;
    const bool rcOursBefore = (rxRuntimeState.rcReadRawFn == localRcReadRaw);
    const bool dshotTelemetryBefore = useDshotTelemetry;

    sitlLocalPreMotorInit();
    featureEnableImmediate(FEATURE_RX_UDP);
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
    batteryConfigMutable()->currentMeterSource = CURRENT_METER_ADC;
    useDshotTelemetry = true;
    if (!rcOursBefore) {
        // A configurator write re-ran rxInit(): put the virtual receiver back.
        localTakeOverRcProvider();
    }

    if (protocolBefore != motorConfig()->dev.motorProtocol
        || featuresBefore != featureConfig()->enabledFeatures
        || voltageBefore != batteryConfig()->voltageMeterSource
        || currentBefore != batteryConfig()->currentMeterSource
        || !rcOursBefore
        || !dshotTelemetryBefore) {
        sitlAuditLog("re-pin after config write: motorProtocol %u->%u features 0x%08X->0x%08X "
                     "battery %u/%u->%u/%u rcTakeover %u->%u dshotTelemetry %u->%u",
                     (unsigned)protocolBefore, (unsigned)motorConfig()->dev.motorProtocol,
                     (unsigned)featuresBefore, (unsigned)featureConfig()->enabledFeatures,
                     (unsigned)voltageBefore, (unsigned)currentBefore,
                     (unsigned)batteryConfig()->voltageMeterSource,
                     (unsigned)batteryConfig()->currentMeterSource,
                     (unsigned)rcOursBefore,
                     (unsigned)(rxRuntimeState.rcReadRawFn == localRcReadRaw),
                     (unsigned)dshotTelemetryBefore, (unsigned)useDshotTelemetry);
    }
}

void sitlLocalRequestReset(void)
{
    InterlockedExchange(&gLocalPendingReset, 1);
}

// sitl_local_set_rate() writes the RAM profile immediately and defers the
// EEPROM persist to the background thread, so a host-side rate change never
// does file I/O (or config reads) on the UE thread.
static volatile LONG gLocalPendingEepromWrite = 0;

void sitlLocalRequestEepromWrite(void)
{
    InterlockedExchange(&gLocalPendingEepromWrite, 1);
}

// --- virtual EEPROM path / runtime config reload ---
// The requested work is deferred to sitl_local_step() (host thread, between
// steps) because reading a config rewrites every PG record the flight loop
// reads, and re-initialising the mixer/motor/servo state from another thread
// crashes the host. Deferring the path switch as well keeps the "persist the
// aircraft being left" write ahead of the environment change, which is what
// makes it target the old file (and what makes a brand-new path detectable).
static volatile LONG gLocalReloadPending = 0;
// Virtual time of the newest pending reload / EEPROM path switch. Used to bound
// how long such a request may be held back for a quiet moment (see
// sitl_local_step).
static volatile LONG64 gLocalReloadRequestUs = 0;
static volatile LONG gLocalRebootPending = 0;
// Set by sitl_local_reload_config(): force the boot-equivalent re-init of the
// stateful filter chains (gyro LPF/notch/dyn-notch/RPM + PID D-term filters)
// instead of only re-applying them when their configuration changed. A fresh
// boot always rebuilds them; a configurator write can leave them inconsistent,
// and this is the repair path for that.
static volatile LONG gLocalForceFullInit = 0;
static volatile LONG gLocalPathPending = 0;
static char gLocalPendingEepromPath[1024];

extern void ensureEepromDirectory(void);
extern bool sitlEepromFileIsOpen(void);

// Persist the current RAM config and close the virtual EEPROM file. The write
// goes to the file the open handle points at (or, when the handle is closed,
// to BF_SITL_EEPROM as it is set right now), i.e. always to the aircraft being
// left. Closing matters because loadEEPROMFromFile() refuses to open a file
// while a handle is open, and configs are only written when the streamer sees
// a change - so the handle opened at boot can still be around.
static void localFlushEepromWrite(void)
{
    InterlockedExchange(&gLocalPendingEepromWrite, 0);
    writeEEPROM();
    if (sitlEepromFileIsOpen()) {
        configLock();
    }
}

int sitl_local_set_eeprom_path(const char *path)
{
    if (path == NULL) {
        return -1;
    }

    char resolved[1024];
    if (path[0] == '\0') {
        char appData[MAX_PATH];
        if (GetEnvironmentVariableA("LOCALAPPDATA", appData, sizeof(appData)) > 0) {
            _snprintf(resolved, sizeof(resolved),
                      "%s\\Betaflight-SITL\\eeprom.bin", appData);
        } else {
            resolved[0] = '\0'; // no override: eeprom.bin in the working directory
        }
    } else {
        if (strlen(path) >= sizeof(resolved)) {
            return -1;
        }
        _snprintf(resolved, sizeof(resolved), "%s", path);
    }

    // Applied by the next sitl_local_step(): the switch saves the aircraft
    // being left, then opens (and, when new, initialises) the requested file.
    strncpy(gLocalPendingEepromPath, resolved, sizeof(gLocalPendingEepromPath) - 1);
    gLocalPendingEepromPath[sizeof(gLocalPendingEepromPath) - 1] = '\0';
    InterlockedExchange64(&gLocalReloadRequestUs, (LONG64)micros64());
    InterlockedExchange(&gLocalPathPending, 1);
    return 0;
}

int sitl_local_reload_config(void)
{
    if (!gLocalRunning) {
        return -1;
    }
    // No arming check and no forced filter rebuild: the reload applies the file's
    // values and then rebuilds only the chains whose settings actually changed
    // (the same rule the configurator's Save uses). Forcing a rebuild here is what
    // used to shake the aircraft.
    InterlockedExchange64(&gLocalReloadRequestUs, (LONG64)micros64());
    InterlockedExchange(&gLocalReloadPending, 1);
    return 0;
}

// Runs on the host thread inside sitl_local_step(). Applies a pending EEPROM
// path switch and/or re-reads the selected file into the flash mirror and then
// into the PG config, followed by everything the boot path derives from it
// (LOCAL link overrides, mixer/motor/servo setup, filters, ...).
//
// `rereadEeprom` is false for the firmware-reboot path only: there the EEPROM
// was just written from RAM a few lines above, so re-reading it cannot change a
// single value - but readEEPROM() ends in activateConfig(), which re-initialises
// the stick-transient chain (initRcProcessing() -> feedforward / setpoint
// smoothing, pidInit(), rcControlsInit(), ...) while the flight loop keeps
// running. Measured with tools/sitl_local_save_compare: that alone made the same
// roll-stick input produce up to 153 us (~15% of the output range) of different
// motor output after a "Save and Reboot", on a bit-identical steady state, and
// the difference was stable rather than settling out. Everything the config
// derives is re-applied explicitly below anyway (link overrides, mixer/motor/
// servo, filters when changed, debug mode, initPhase3 profile state), so the
// reboot keeps the runtime in the operating point it was already flying in - the
// aircraft behaves the same before and after, which is what the simulator wants
// from a reboot that never actually restarts the MCU.
// FNV-1a over the virtual flash image: the reload path uses it to tell whether
// the file it is about to load is actually different from what the FC is running.
static uint32_t localEepromHash(void)
{
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < sizeof(eepromData); i++) {
        hash ^= eepromData[i];
        hash *= 16777619u;
    }
    return hash;
}

static void localRunPendingReloadInternal(bool rereadEeprom)
{
    InterlockedExchange(&gLocalReloadPending, 0);

    // Save the aircraft being left and close its file. This has to happen
    // before the environment changes so the write targets the old path, and
    // before the re-read because an open handle blocks opening another file.
    localFlushEepromWrite();

    bool freshFile = false;
    bool pathSwitch = false;
    if (InterlockedExchange(&gLocalPathPending, 0) != 0) {
        pathSwitch = true;
        // A path that does not exist yet is a brand-new aircraft. sitl.c's
        // loadEEPROMFromFile() creates it from the flash mirror, i.e. it would
        // inherit the aircraft we just left. Clear the mirror (erased flash)
        // so the new file is created empty and then re-initialised with
        // factory defaults below, exactly like a fresh FC.
        freshFile = gLocalPendingEepromPath[0] != '\0'
            && GetFileAttributesA(gLocalPendingEepromPath) == INVALID_FILE_ATTRIBUTES;
        if (freshFile) {
            memset(eepromData, 0, sizeof(eepromData));
        }
        _putenv_s("BF_SITL_EEPROM", gLocalPendingEepromPath);
        ensureEepromDirectory();
    }

    // Snapshot the gyro filter configuration first: the filter chain is
    // stateful, and re-initialising it while the host keeps feeding gyro
    // samples injects a step that the PID's D-term amplifies into a
    // full-authority spike (violent oscillation right after a reload/reboot).
    // It is therefore only rebuilt when the filter settings really changed.
    extern void sitlLocalSnapshotGyroFilterConfig(void);
    extern bool sitlLocalGyroFilterConfigChanged(void);
    extern void sitlLocalRunBootReapply(bool reinitGyroFilters);
    sitlLocalSnapshotGyroFilterConfig();
    // Same idea for the PID/D-term chain: remember the settings the running chain
    // was built from, so the re-apply below can tell whether the loaded file
    // really changed them.
    extern void sitlLocalSnapshotPidFilterConfig(void);
    sitlLocalSnapshotPidFilterConfig();

    // Open the file named by BF_SITL_EEPROM (sitlFopen resolves the env var on
    // every call) and load it into the flash mirror, then into the PG records.
    // The mirror currently holds exactly the configuration the FC is running
    // (localFlushEepromWrite() just wrote it), so comparing it with the file that
    // is about to be loaded tells us whether this reload changes anything at all.
    const uint32_t runningHash = localEepromHash();
    configUnlock();
    if (!freshFile && localEepromHash() == runningHash) {
        // Same configuration: re-applying it would only re-initialise the
        // stateful gyro/D-term/RC chains, which is what shakes the aircraft when
        // it happens in a running loop. Nothing to do - the selected file is
        // already what the FC is flying (the environment variable was switched
        // above, so the next save targets the new path).
        sitlAuditLog("reload: configuration identical, nothing to re-apply");
        return;
    }
    if (freshFile) {
        // Mirrors fc/init.c: an invalid/erased config resets to the factory
        // defaults and writes them (here into the newly created file).
        ensureEEPROMStructureIsValid();
    }
    if (rereadEeprom || pathSwitch) {
        // Load the selected file's *values* without the firmware's readEEPROM().
        // readEEPROM() ends in activateConfig(), which re-initialises the stateful
        // gyro/D-term/RC chains (initRcProcessing, pidInit, rcControlsInit, ...)
        // while the flight loop is running - the same "live re-init" that made the
        // configurator's Save shake the aircraft, and the one path that
        // "Save and Reboot" does *not* take (it skips the re-read). Everything the
        // config derives is re-applied below by the reload's own re-apply, which
        // rebuilds only the chains whose settings actually changed.
        extern bool loadEEPROM(void);
        const bool loaded = loadEEPROM();
        changePidProfile(systemConfig()->pidProfileIndex);
        changeControlRateProfile(systemConfig()->activeRateProfile);
        sitlAuditLog("reload: values-only EEPROM load (no activateConfig), ok=%u",
                     loaded ? 1u : 0u);
    } else {
        sitlAuditLog("reboot: EEPROM re-read skipped (runtime state preserved)");
    }
    const bool gyroFiltersChanged = sitlLocalGyroFilterConfigChanged();
    const bool forceFullInit = InterlockedExchange(&gLocalForceFullInit, 0) != 0;
    if (forceFullInit) {
        sitlAuditLog("reload: forced boot-equivalent re-init (gyro + PID filter chains)");
    }

    // Re-apply what readEEPROM/activateConfig does not cover: the LOCAL link
    // overrides (UDP RX provider, battery shims, PWM motor backend) and the
    // boot-time derived state (mixer mode, motor/servo setup, gyro filters,
    // debug mode).
    localApplyLinkOverrides();
    sitlLocalRunBootReapply(forceFullInit || gyroFiltersChanged);

    // Finish what fc/init.c's initPhase3 does after that (looptime + DShot
    // telemetry, board alignment, IMU/failsafe init, and the profile-derived
    // mixer runtime: dynamic idle, VBAT sag compensation, RPM limiter, ez
    // landing). Without it an in-process reboot would not match a fresh boot
    // for a configured aircraft.
    extern void sitlLocalRunBootProfileInit(void);
    sitlLocalRunBootProfileInit();
}

// Explicit reload (sitl_local_reload_config(), EEPROM path switch): the file is
// the source of truth, so read it.
static void localRunPendingReload(void)
{
    localRunPendingReloadInternal(true);
}

// Deferred half of a firmware reboot; the immediate half (disarm, CLI and
// arming state, log close) is sitlLocalRequestReboot() in wincompat.c. Queued
// here for the same reason as the config reload: the EEPROM re-read and every
// derived re-init must happen on the host thread, between scheduler passes.
void sitlLocalRequestRebootApply(void)
{
    InterlockedExchange(&gLocalRebootPending, 1);
}

// Mirrors what a real boot does after loading the config (fc/init.c):
// everything localRunPendingReload() re-applies, followed by the initPhase3
// modules a reboot re-initialises.
static void localRunPendingReboot(void)
{
    InterlockedExchange(&gLocalRebootPending, 0);

    // Persist, re-apply the LOCAL link overrides (UDP RX, ADC battery meters,
    // PWM motor backend) and the derived mixer/motor/servo/filter/debug state,
    // and apply a pending EEPROM path switch. The EEPROM itself is not re-read:
    // it was just written from RAM, so nothing can change, while readEEPROM()'s
    // activateConfig() would re-initialise the stick-transient chain under a
    // live flight loop (see localRunPendingReloadInternal).
    localRunPendingReloadInternal(false);

#ifdef USE_BLACKBOX
    // initPhase3 re-inits the blackbox (the log was closed by the reboot), so
    // a blackbox device / directory change takes effect.
    blackboxInit();
#endif
    // Real FCs calibrate the gyro on every boot. With the virtual gyro this
    // completes immediately (gyroSetCalibrationCycles() leaves 0 cycles for
    // GYRO_VIRTUAL), so it cannot delay arming in the simulator.
    // (Declared here rather than including sensors/gyro.h: the LOCAL stub
    // mpuGyroReadRegister() below deliberately uses a different signature.)
    extern void gyroStartCalibration(bool isFirstArmingCalibration);
    gyroStartCalibration(false);
}

// --- LOCAL-mode link stubs ---
// The DLL keeps a few firmware paths alive that the executable build
// garbage-collects (PE export/import bookkeeping). Their implementations are
// target-only and not compiled for SITL; provide inert definitions so the
// library links. The virtual-sensor path never calls them.
uint8_t mpuGyroReadRegister(void *dev, uint8_t reg)
{
    (void)dev;
    (void)reg;
    return 0;
}

int dmaGetHandlerCount(void)
{
    return 0;
}

dmaChannelDescriptor_t dmaDescriptors[1] = {{0}};

bool useDshotTelemetry;

// DSHOT hardware entry points are target-only and not compiled for SITL;
// provide inert versions so the USE_DSHOT code paths link. The virtual PWM
// motor device is used at runtime, so these are never called.
bool isDshotBitbangActive(const motorDevConfig_t *motorDevConfig)
{
    (void)motorDevConfig;
    return false;
}

bool dshotBitbangDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    (void)device;
    (void)motorConfig;
    return false;
}

bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    (void)device;
    (void)motorConfig;
    return false;
}

dshotBitbangStatus_e dshotBitbangGetStatus(void)
{
    return DSHOT_BITBANG_STATUS_OK;
}

dshotTelemetryCycleCounters_t dshotDMAHandlerCycleCounters;

#ifndef USE_GPS_LAP_TIMER
gpsLapTimerConfig_t gpsLapTimerConfig_System;
#endif

static DWORD WINAPI localMspThreadProc(LPVOID arg)
{
    (void)arg;
    gMspThreadId = GetCurrentThreadId();
    while (!gMspThreadStop) {
        if (setjmp(gMspLoopJmp) != 0) {
            sitlAuditLog("MSP thread recovered from reboot jump");
        }
        InterlockedExchange(&gMspJmpReady, 1);
        // cliEnter() sets ARMING_DISABLED_CLI and nothing in the firmware
        // clears it (real FCs reboot to reset it). Watch cliMode transitions
        // so "exit noreboot" (e.g. configurator closes the panel / disconnect
        // injection) also unblocks arming.
        if (gCliWasActive && !cliMode) {
            sitlAuditLog("cliMode cleared by watcher");
            unsetArmingDisabled(ARMING_DISABLED_CLI);
        }
        gCliWasActive = cliMode;

        mspSerialProcess(LOCAL_MSP_EVALUATE_NON_MSP_DATA,
                         mspFcProcessCommand, mspFcProcessReply);
        if (InterlockedExchange(&gLocalPendingReset, 0) != 0) {
            sitlAuditLog("deferred reset: persisting config");
            writeEEPROM();
            unsetArmingDisabled(ARMING_DISABLED_CLI);
        }
        if (InterlockedExchange(&gLocalPendingEepromWrite, 0) != 0) {
            sitlAuditLog("local setter: persisting config");
            writeEEPROM();
        }
        // 1 ms poll: keeps the configurator responsive when the host is not
        // stepping (no scheduler TASK_SERIAL) without adding meaningful CPU.
        Sleep(1);
    }
    InterlockedExchange(&gMspJmpReady, 0);
    return 0;
}

int sitl_local_init(void)
{
    if (gLocalRunning) {
        return 0;
    }
    gLocalBootCount++;

    // A host may choose the virtual EEPROM before boot (one config per aircraft).
    // Apply it first, so the boot below opens that file directly instead of the
    // default one - and so a tool that wants to work on a copy never writes the
    // real config at all.
    if (InterlockedExchange(&gLocalPathPending, 0) != 0 && gLocalPendingEepromPath[0] != '\0') {
        _putenv_s("BF_SITL_EEPROM", gLocalPendingEepromPath);
        extern void ensureEepromDirectory(void);
        ensureEepromDirectory();
        fprintf(stderr, "[SITL] LOCAL mode EEPROM (pre-boot): %s\n", gLocalPendingEepromPath);
        sitlAuditLog("sitl_local_init: pre-boot eeprom=%s", gLocalPendingEepromPath);
    }

    // A host can stop and start the FC inside one process (level reload, PIE
    // restart). A fresh process starts with these zeroed by the loader, so reset
    // them here as well - otherwise the second boot inherits the previous run's
    // RC frame cadence, CLI flag and pending work, and no longer behaves like a
    // clean start (measured: 315 us difference in the stick response).
    gLocalRcValid = false;
    // Anchor the frame grid at the init moment (0 for a fresh process).
    gLocalRcAnnounceUs = micros64();
    gLocalRcFrameCount = 0;
    memset(gLocalRc, 0, sizeof(gLocalRc));
    gGpsOriginSet = false;
    gCliWasActive = false;
    gLocalPendingReset = 0;
    gLocalPendingEepromWrite = 0;
    gLocalReloadPending = 0;
    gLocalRebootPending = 0;
    gLocalPathPending = 0;
    gLocalForceFullInit = 0;
    gLocalRepinPending = 0;
    gLocalPendingEepromPath[0] = '\0';
    memset(gLocalLastMotors, 0, sizeof(gLocalLastMotors));

    // A fresh process gets every firmware static zeroed by the loader; an
    // in-process restart re-uses the previous run's RAM, so reset the volatile
    // control state (PID integrators, PID runtime history, attitude, raw gyro
    // leftovers) before init() runs. The boot then derives every chain exactly
    // like a cold start instead of inheriting the last flight's operating
    // point - see wincompat.c's sitlLocalResetVolatileControlState().
    if (gLocalBootCount > 1) {
        extern void sitlLocalResetVolatileControlState(void);
        sitlLocalResetVolatileControlState();
    }

    // Use a stable, writable EEPROM location regardless of the host process's
    // working directory (UE can be launched from anywhere). Respect an
    // explicit BF_SITL_EEPROM override if the host already set one.
    const char *eepromOverride = getenv("BF_SITL_EEPROM");
    if (eepromOverride == NULL || eepromOverride[0] == '\0') {
        char appDataPath[MAX_PATH];
        if (GetEnvironmentVariableA("LOCALAPPDATA", appDataPath, sizeof(appDataPath)) > 0) {
            char eepromPath[MAX_PATH];
            _snprintf(eepromPath, sizeof(eepromPath), "%s\\Betaflight-SITL\\eeprom.bin", appDataPath);
            // _putenv_s updates the CRT environment table; SetEnvironmentVariableA
            // only touches the OS block, which getenv() in this process would not
            // see (sitlFopen redirects the EEPROM via getenv).
            _putenv_s("BF_SITL_EEPROM", eepromPath);
            fprintf(stderr, "[SITL] LOCAL mode EEPROM: %s\n", eepromPath);
            sitlAuditLog("sitl_local_init: eeprom=%s", eepromPath);
        }
    }

    // Tolerate a stale virtual-EEPROM handle, the same way AJ92/SimITL's target
    // does ("simitl can just restart without closing the fileDesc"): sitl.c's
    // loadEEPROMFromFile() refuses to start while its own eepromFd is open, so a
    // host that restarts the FC in process without a clean sitl_local_shutdown()
    // would otherwise hang in FLASH_Unlock. configLock() flushes the flash mirror
    // (which still matches the file unless the previous run changed settings
    // without saving) and clears the handle, so the boot below can open it again.
    if (sitlEepromFileIsOpen()) {
        sitlAuditLog("sitl_local_init: closing a stale EEPROM handle before boot");
        configLock();
    }

    sitlBoot(0, NULL);

    // Make the local link self-sufficient regardless of the EEPROM contents
    // (UDP RX provider, ADC battery shims, virtual PWM backend, DSHOT
    // telemetry). Shared with the runtime config reload, which replaces every
    // PG record and therefore has to re-apply the same overrides.
    localApplyLinkOverrides();

    // Record the settings the boot path built its stateful chains from (gyro
    // filters, D-term filters, rate/RC processing). The configurator's SET
    // handlers only rebuild them when the settings really changed, so a save
    // that changes nothing leaves the running control loop alone - see
    // wincompat.c's sitlMsp*Init* wrappers.
    extern void sitlLocalSnapshotGyroFilterConfig(void);
    extern void sitlLocalSnapshotPidFilterConfig(void);
    extern void sitlLocalSnapshotRcProcessingConfig(void);
    sitlLocalSnapshotGyroFilterConfig();
    sitlLocalSnapshotPidFilterConfig();
    sitlLocalSnapshotRcProcessingConfig();

    // Fingerprint the derived control chains the boot just built, so a
    // process-internal restart can be compared against a fresh process boot
    // (see wincompat.c's sitlLocalLogControlState).
    extern void sitlLocalLogControlState(const char *tag);
    sitlLocalLogControlState(gLocalBootCount == 1 ? "boot#1" : "boot#n");

    gLocalRunning = true;
    gMspThreadStop = 0;
    InitializeCriticalSection(&gMspCrit);
    gMspThread = CreateThread(NULL, 0, localMspThreadProc, NULL, 0, NULL);
    if (gMspThread == NULL) {
        gLocalRunning = false;
        return -1;
    }
    return 0;
}

// --- synchronous state access (shared with the configurator via MSP) ---
// These read/write the exact same globals the MSP handlers use, so a value
// changed from the configurator is immediately visible here and vice versa.
// Reads are plain global reads (fine from the UE tick); the only write,
// sitl_local_set_rate(), writes uint8_t fields and defers the EEPROM persist
// to the background MSP thread.

uint32_t sitl_local_get_arming_flags(void)
{
    return gLocalRunning ? (uint32_t)getArmingDisableFlags() : 0;
}

bool sitl_local_is_arming_disabled(void)
{
    return gLocalRunning && isArmingDisabled();
}

bool sitl_local_get_armed(void)
{
    return gLocalRunning && ARMING_FLAG(ARMED);
}

// Same action as the configurator's Disarm button (msp.c's MSP_SET_ARMING_DISABLED
// path): the reason only tags the blackbox disarm event, it does not set an
// arming-disable flag, so the craft can arm again as soon as the ARM switch
// asks for it. Note that rc_controls.c calls tryArm() on every scheduler pass
// while BOXARM is active, so a host that disarms through this call must also
// drive the ARM channel low - otherwise the FC re-arms on the next pass.
int sitl_local_disarm(void)
{
    if (!gLocalRunning) {
        return -1;
    }
    if (ARMING_FLAG(ARMED)) {
        disarm(DISARM_REASON_ARMING_DISABLED);
    }
    return 0;
}

uint32_t sitl_local_get_flight_modes(void)
{
    return gLocalRunning ? (uint32_t)flightModeFlags : 0;
}

// Display scaling for the Rates tab, mirroring the configurator exactly.
// The profile stores rcRates/rcExpo/rates as uint8 in hundredths; how the
// user-facing number is derived depends on the rate mode:
//   RC Rate column:      RACEFLIGHT/ACTUAL -> stored/100*1000, else stored/100
//   Super Rate / rate:   RACEFLIGHT -> stored/100*100,
//                        ACTUAL/QUICK -> stored/100*1000, else stored/100
//   Expo:                RACEFLIGHT -> stored/100*100, else stored/100
static float sitlRateScaleFactor(uint8_t ratesType)
{
    return (ratesType == RATES_TYPE_RACEFLIGHT || ratesType == RATES_TYPE_ACTUAL) ? 1000.0f : 1.0f;
}

static float sitlRateRateScaleFactor(uint8_t ratesType)
{
    switch (ratesType) {
    case RATES_TYPE_RACEFLIGHT:
        return 100.0f;
    case RATES_TYPE_ACTUAL:
    case RATES_TYPE_QUICK:
        return 1000.0f;
    default:
        return 1.0f;
    }
}

static float sitlRateExpoScaleFactor(uint8_t ratesType)
{
    return (ratesType == RATES_TYPE_RACEFLIGHT) ? 100.0f : 1.0f;
}

void sitl_local_get_rate(int index, float rcRate[3], float rcExpo[3],
                         float superRate[3])
{
    if (rcRate)    { rcRate[0] = rcRate[1] = rcRate[2] = 0.0f; }
    if (rcExpo)    { rcExpo[0] = rcExpo[1] = rcExpo[2] = 0.0f; }
    if (superRate) { superRate[0] = superRate[1] = superRate[2] = 0.0f; }
    if (!gLocalRunning) {
        return;
    }

    const controlRateConfig_t *profile =
        (index >= 0 && index < CONTROL_RATE_PROFILE_COUNT)
            ? controlRateProfiles(index)
            : currentControlRateProfile;

    const float sf  = sitlRateScaleFactor(profile->rates_type);
    const float rsf = sitlRateRateScaleFactor(profile->rates_type);
    const float esf = sitlRateExpoScaleFactor(profile->rates_type);

    for (int axis = 0; axis < 3; axis++) {
        if (rcRate)    rcRate[axis]    = profile->rcRates[axis] / 100.0f * sf;
        if (rcExpo)    rcExpo[axis]    = profile->rcExpo[axis] / 100.0f * esf;
        if (superRate) superRate[axis] = profile->rates[axis] / 100.0f * rsf;
    }
}

void sitl_local_set_rate(const float rcRate[3], const float rcExpo[3],
                         const float superRate[3])
{
    if (!gLocalRunning) {
        return;
    }

    controlRateConfig_t *profile = currentControlRateProfile;
    const uint8_t mode = profile->rates_type;
    const float sf  = sitlRateScaleFactor(mode);
    const float rsf = sitlRateRateScaleFactor(mode);
    const float esf = sitlRateExpoScaleFactor(mode);
    const ratesSettingsLimits_t *limits = &ratesSettingLimits[mode];

    // Inputs are in the same units the Rates tab displays for the current
    // mode; convert back to the stored hundredths and clamp to the mode's
    // limits (the same bounds the configurator applies).
    for (int axis = 0; axis < 3; axis++) {
        if (rcRate)    profile->rcRates[axis] = (uint8_t)constrain(lrintf(rcRate[axis]    / sf  * 100.0f), 0, limits->rc_rate_limit);
        if (rcExpo)    profile->rcExpo[axis] = (uint8_t)constrain(lrintf(rcExpo[axis]    / esf * 100.0f), 0, limits->expo_limit);
        if (superRate) profile->rates[axis] = (uint8_t)constrain(lrintf(superRate[axis] / rsf * 100.0f), 0, limits->srate_limit);
    }

    sitlLocalRequestEepromWrite();
}

int sitl_local_get_rate_mode(void)
{
    return gLocalRunning ? (int)currentControlRateProfile->rates_type : -1;
}

int sitl_local_set_rate_mode(int mode)
{
    if (!gLocalRunning) {
        return -1;
    }
    if (mode < 0 || mode >= RATES_TYPE_COUNT) {
        return -1;
    }
    currentControlRateProfile->rates_type = (uint8_t)mode;
    sitlLocalRequestEepromWrite();
    return 0;
}

void sitl_local_get_arm_switch(uint8_t *auxChannel, uint8_t *startStep,
                               uint8_t *endStep)
{
    if (auxChannel) *auxChannel = 0xFF;
    if (startStep)  *startStep  = 0;
    if (endStep)    *endStep    = 0;
    if (!gLocalRunning) {
        return;
    }

    modeActivationCondition_t emptyMac;
    memset(&emptyMac, 0, sizeof(emptyMac));
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);
        // Skip unconfigured (all-zero) slots: modeId == 0 is BOXARM, so a
        // fresh EEPROM's empty conditions would otherwise look like an arm
        // switch on AUX1 at 900us.
        if (isModeActivationConditionConfigured(mac, &emptyMac) && mac->modeId == BOXARM) {
            if (auxChannel) *auxChannel = mac->auxChannelIndex;
            if (startStep)  *startStep  = mac->range.startStep;
            if (endStep)    *endStep    = mac->range.endStep;
            return;
        }
    }
}

int sitl_local_set_arm_switch(uint8_t auxChannel, uint8_t startStep,
                              uint8_t endStep)
{
    if (!gLocalRunning) {
        return -1;
    }
    if (auxChannel != 0xFF && auxChannel >= SITL_LOCAL_MAX_RC_CHANNELS) {
        return -1;
    }
    if (startStep > MAX_MODE_RANGE_STEP || endStep > MAX_MODE_RANGE_STEP
        || startStep > endStep) {
        return -1;
    }

    // Clear every existing BOXARM condition (frees a slot at the tail).
    removeModeActivationCondition(BOXARM);

    if (auxChannel != 0xFF) {
        // Reuse the first all-zero slot.
        int slot = -1;
        modeActivationCondition_t emptyMac;
        memset(&emptyMac, 0, sizeof(emptyMac));
        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            if (!isModeActivationConditionConfigured(modeActivationConditions(i), &emptyMac)) {
                slot = i;
                break;
            }
        }
        if (slot < 0) {
            return -1; // all 20 slots used
        }

        modeActivationCondition_t *mac = modeActivationConditionsMutable(slot);
        memset(mac, 0, sizeof(*mac));
        mac->modeId = BOXARM;
        mac->auxChannelIndex = auxChannel;
        mac->range.startStep = startStep;
        mac->range.endStep = endStep;
    }

    // Same refresh as MSP_SET_MODE_RANGE: rebuild the active-condition list
    // and update stick-vs-switch arming, then persist on the background thread.
    rcControlsInit();
    sitlLocalRequestEepromWrite();
    return 0;
}

// Number of servo outputs the current mixer writes, mirroring writeServos()
// in servos.c (without the channel-forwarding extras). 0 for multicopters.
static uint8_t sitlLocalServoCount(void)
{
    switch (getMixerMode()) {
    case MIXER_TRI:
    case MIXER_CUSTOM_TRI:
        return 1;
    case MIXER_FLYING_WING:
        return 2;
    case MIXER_CUSTOM_AIRPLANE:
    case MIXER_AIRPLANE:
        return SERVO_PLANE_INDEX_MAX - SERVO_PLANE_INDEX_MIN + 1; // 6
#ifdef USE_UNCOMMON_MIXERS
    case MIXER_BICOPTER:
    case MIXER_DUALCOPTER:
        return 2;
    case MIXER_HELI_120_CCPM:
        return 4;
    case MIXER_SINGLECOPTER:
        return 4;
#endif
    default:
        break;
    }
    if (featureIsEnabled(FEATURE_SERVO_TILT) || getMixerMode() == MIXER_GIMBAL) {
        return 2;
    }
    return 0;
}

// Host-side step diagnostics (see sitlLocalStepStats()).
static uint32_t gLocalStepUsMax = 0;
static uint32_t gLocalStepUsAvg = 0;
static uint32_t gLocalStepCount = 0;
static uint32_t gLocalStepUsSum = 0;
static uint32_t gLocalZeroMotorStepsWhileArmed = 0;

static void localRecordStepStats(uint64_t stepStartUs, const sitl_local_output_t *out)
{
    extern uint64_t micros64_real(void);
    const uint32_t elapsedUs = (uint32_t)(micros64_real() - stepStartUs);
    if (elapsedUs > gLocalStepUsMax) {
        gLocalStepUsMax = elapsedUs;
    }
    gLocalStepUsSum += elapsedUs;
    if (++gLocalStepCount >= 1000) {
        gLocalStepUsAvg = gLocalStepUsSum / gLocalStepCount;
        gLocalStepUsSum = 0;
        gLocalStepUsMax = 0;
        gLocalStepCount = 0;
    }

    if (out != NULL) {
        float sum = 0.0f;
        for (int i = 0; i < 4; i++) {
            gLocalLastMotors[i] = out->pwm_output_raw[i];
            sum += out->pwm_output_raw[i];
        }
        // All-zero motor outputs are only legitimate while disarmed (the
        // disarmed value is ~1000): while armed they mean the captured motor
        // packet was missing and the host received garbage.
        if (out->armed && sum < 1.0f) {
            gLocalZeroMotorStepsWhileArmed++;
        }
    }
}

void sitl_local_step(const sitl_local_input_t *in, uint32_t dtUs,
                     sitl_local_output_t *out)
{
    if (out) {
        memset(out, 0, sizeof(*out));
    }
    if (!gLocalRunning || !in) {
        return;
    }
    extern uint64_t micros64_real(void);
    const uint64_t stepStartUs = micros64_real();

    // A configurator Save may have undone the pinned LOCAL runtime state; this
    // only rewrites flags/mode sources, so it also runs while armed.
    if (InterlockedCompareExchange(&gLocalRepinPending, 0, 0) != 0) {
        localRepinOverrides();
    }

    // State flight recorder: ~1 Hz, and only writes to the audit log when any
    // control-relevant state actually changed (see wincompat.c). This is what
    // makes "a save left the FC in a different state" visible.
    static uint32_t stepCounter = 0;
    if (++stepCounter % 1000 == 0) {
        extern void sitlLocalLogStateIfChanged(const char *tag);
        sitlLocalLogStateIfChanged("state");
    }

    // Deferred config work (firmware reboot, EEPROM reload and/or path switch)
    // runs here, on the same thread as the scheduler and between steps, so it
    // can never race the flight loop. While armed the requests stay pending and
    // are retried on a later step, so a save/reload never yanks the mixer out
    // from under a flying craft (a reboot disarms first, so it applies at once).
    if (InterlockedCompareExchange(&gLocalRebootPending, 0, 0) != 0) {
        if (!ARMING_FLAG(ARMED)) {
            // A reboot re-reads the config and re-applies it plus the
            // initPhase3 modules, mirroring a real boot.
            localRunPendingReboot();
        }
    } else if (InterlockedCompareExchange(&gLocalReloadPending, 0, 0) != 0 ||
               InterlockedCompareExchange(&gLocalPathPending, 0, 0) != 0) {
        // A pending EEPROM path switch / explicit reload is applied by the same
        // rule as the configurator's Save: the selected file's *values* are loaded
        // (loadEEPROM(), no activateConfig()), the LOCAL link overrides are
        // re-pinned, and only the stateful chains whose settings actually changed
        // are rebuilt (see localRunPendingReloadInternal). Running the firmware's
        // readEEPROM()/activateConfig() here instead re-initialises the gyro
        // filter, D-term filter and rate/RC chains under a live gyro stream, which
        // is exactly the step the D term amplifies into the "aircraft shakes"
        // oscillation - the Save path had the same problem and this is the same
        // fix.
        localRunPendingReload();
    }

    // --- virtual gyro (Gazebo bridge axis mapping) ---
    double gyroRoll, gyroPitch, gyroYaw;
    sitlGyroBodyFromSim(in->angular_velocity_rpy, ENABLE_GAZEBO_BRIDGE,
                        &gyroRoll, &gyroPitch, &gyroYaw);
    const int16_t gx = (int16_t)constrain(gyroRoll  * LOCAL_GYRO_SCALE * LOCAL_RAD2DEG, -32767, 32767);
    const int16_t gy = (int16_t)constrain(gyroPitch * LOCAL_GYRO_SCALE * LOCAL_RAD2DEG, -32767, 32767);
    const int16_t gz = (int16_t)constrain(gyroYaw   * LOCAL_GYRO_SCALE * LOCAL_RAD2DEG, -32767, 32767);
    virtualGyroSet(virtualGyroDev, gx, gy, gz);
    gLocalLastGyroRaw[0] = gx;
    gLocalLastGyroRaw[1] = gy;
    gLocalLastGyroRaw[2] = gz;

    // --- pressure derived from altitude (Gazebo bridge convention) ---
    const double altMeters = in->position_xyz[2];
    const int32_t pressure = (int32_t)(101325.0 * pow(1.0 - 2.25577e-5 * altMeters, 5.25588));
    virtualBaroSet(pressure, 2500);

    // --- attitude quaternion (Gazebo plugin format -> NWU body-to-world) ---
    const float pktQw = (float)in->orientation_quat[0];
    const float pktQx = (float)in->orientation_quat[1];
    const float pktQy = -(float)in->orientation_quat[2];
    const float pktQz = -(float)in->orientation_quat[3];
    static const float k = 0.70710678f;
    const float attQw = k * (pktQw - pktQz);
    const float attQx = k * (pktQx - pktQy);
    const float attQy = k * (pktQy + pktQx);
    const float attQz = k * (pktQz + pktQw);

    // Body->world (NWU) rotation matrix; rows select the earth axis, columns
    // the body axis (same layout as imu.c's rMat).
    const float r00 = 1.0f - 2.0f * (attQy * attQy + attQz * attQz);
    const float r01 = 2.0f * (attQx * attQy - attQw * attQz);
    const float r02 = 2.0f * (attQx * attQz + attQw * attQy);
    const float r10 = 2.0f * (attQx * attQy + attQw * attQz);
    const float r11 = 1.0f - 2.0f * (attQx * attQx + attQz * attQz);
    const float r12 = 2.0f * (attQy * attQz - attQw * attQx);
    const float r20 = 2.0f * (attQx * attQz - attQw * attQy);
    const float r21 = 2.0f * (attQy * attQz + attQw * attQx);
    const float r22 = 1.0f - 2.0f * (attQx * attQx + attQy * attQy);

    // --- virtual accelerometer (same signs/scales as sitl.c updateState) ---
    // Mahony only uses the accel when its magnitude is within 0.9..1.1 g
    // (imuIsAccelerometerHealthy); otherwise attitude is pure gyro
    // integration and roll/pitch never converge. If the host feed is missing
    // or wrong (e.g. not gravity-compensated), derive a healthy 1 g specific
    // force from the FDM attitude instead: earth-up in body = R^T * (0,0,1)
    // = row U of the rotation matrix.
    int16_t ax = (int16_t)constrain(-in->linear_acceleration_xyz[0] * LOCAL_ACC_SCALE, -32767, 32767);
    int16_t ay = (int16_t)constrain(-in->linear_acceleration_xyz[1] * LOCAL_ACC_SCALE, -32767, 32767);
    int16_t az = (int16_t)constrain(-in->linear_acceleration_xyz[2] * LOCAL_ACC_SCALE, -32767, 32767);
    const int32_t accMagSq = (int32_t)ax * ax + (int32_t)ay * ay + (int32_t)az * az;
    const int32_t healthyMin = (int32_t)(0.9f * 256.0f);
    const int32_t healthyMax = (int32_t)(1.1f * 256.0f);
    if (accMagSq < healthyMin * healthyMin || accMagSq > healthyMax * healthyMax) {
        ax = (int16_t)lrintf(r20 * 256.0f);
        ay = (int16_t)lrintf(r21 * 256.0f);
        az = (int16_t)lrintf(r22 * 256.0f);
    }
    virtualAccSet(virtualAccDev, ax, ay, az);

    // --- synthetic magnetometer feed (same earth field as sitl.c) ---
    {
        static const float fieldN = 2046.8f;
        static const float fieldW = -71.5f;
        static const float fieldU = -3547.2f;
        const float magX = r00 * fieldN + r10 * fieldW + r20 * fieldU;
        const float magY = r01 * fieldN + r11 * fieldW + r21 * fieldU;
        const float magZ = r02 * fieldN + r12 * fieldW + r22 * fieldU;
        virtualMagSet(lrintf(magX), lrintf(magY), lrintf(magZ));
    }

#if defined(SITL_ATTITUDE_DIRECT)
    imuSetAttitudeQuat(attQw, attQx, attQy, attQz);
#endif

    // --- virtual GPS (Gazebo mirror around the first packet origin) ---
    const double longitude = in->position_xyz[0];
    const double latitude = in->position_xyz[1];
    const double altitude = in->position_xyz[2];
    if (!gGpsOriginSet) {
        gGpsOriginLat = latitude;
        gGpsOriginLon = longitude;
        gGpsOriginSet = true;
    }
    const double correctedLat = 2.0 * gGpsOriginLat - latitude;
    const double correctedLon = 2.0 * gGpsOriginLon - longitude;
    const double vx = in->velocity_xyz[0];
    const double vy = in->velocity_xyz[1];
    const double vz = in->velocity_xyz[2];
    const double speed = sqrt(vx * vx + vy * vy);
    const double speed3D = sqrt(vx * vx + vy * vy + vz * vz);
    double course = atan2(vx, vy) * LOCAL_RAD2DEG;
    if (course < 0.0) {
        course += 360.0;
    }
    if (fabs(latitude) <= 90.0 && fabs(longitude) <= 180.0) {
        setVirtualGPS(correctedLat, correctedLon, altitude, speed, speed3D, course,
                      vy, vx, -vz);
    }

    // --- battery / RPM telemetry ---
    simTelemetrySet(in->battery_voltage, in->battery_current,
                    in->motor_rpm, 4,
                    in->motor_temperature, 4);

    // Publish the ESC telemetry that is read straight out of the DSHOT
    // telemetry state (configurator Motors tab, MSP_ESC_SENSOR_DATA, OSD ESC
    // alarms) - the virtual PWM backend never fills that structure itself.
    extern void sitlLocalApplyDshotTelemetry(void);
    sitlLocalApplyDshotTelemetry();

    // --- RC ---
    // Refresh the cache only when the host data actually changed. The frame
    // status callback emits COMPLETE on a fixed 8 ms cadence, so the FC sees
    // a real 125 Hz receiver (values sampled at frame boundaries).
    if (!gLocalRcValid || memcmp(gLocalRc, in->rc_channels, sizeof(gLocalRc)) != 0) {
        memcpy(gLocalRc, in->rc_channels, sizeof(gLocalRc));
        gLocalRcValid = true;
    }

    // --- run the scheduler on the same 100 us quantum grid as UDP mode ---
    // A single scheduler() call exactly on the gyro deadline only runs the
    // realtime tasks (gyro/filter/PID). Non-realtime tasks (RX, failsafe, OSD,
    // blackbox) are only selected when time is left before the next deadline
    // (schedLoopRemainingCycles > CHECK_GUARD_MARGIN_US), so stepping in
    // 100 us quanta gives the pre-deadline passes a chance to run them -
    // otherwise TASK_RX never processes RC frames and RXLOSS stays active.
    //
    // Carry the sub-quantum remainder over to the next call. Without it a host
    // that does not step in exact multiples of the quantum (any real engine tick
    // jitters a little) injects a short extra step, and that shifts the gyro/PID
    // deadline grid: the loop period then alternates instead of staying at 1 ms,
    // which the D term (delta/dt) turns into a high-frequency tremor. Carrying
    // the remainder keeps every step a whole quantum and the grid aligned.
    static uint32_t gStepRemainderUs = 0;
    uint32_t remainingUs = dtUs + gStepRemainderUs;
    const uint32_t quantumUs = 100;
    gStepRemainderUs = remainingUs % quantumUs;
    while (remainingUs >= quantumUs) {
        sitlStepTime(quantumUs);
        remainingUs -= quantumUs;
        scheduler();
    }

    // --- motor outputs captured by udpSend() in LOCAL mode ---
    if (out) {
        servo_packet_raw raw;
        if (sitlLocalTakeMotorPacket(&raw, sizeof(raw))) {
            out->motor_count = raw.motorCount;
            for (int i = 0; i < raw.motorCount && i < SITL_LOCAL_MAX_MOTORS; i++) {
                out->pwm_output_raw[i] = raw.pwm_output_raw[i];
            }
            out->servo_count = sitlLocalServoCount();
            for (int i = 0; i < out->servo_count && i < SITL_LOCAL_MAX_SERVOS; i++) {
                out->servo_output_raw[i] = raw.pwm_output_raw[raw.motorCount + i];
            }
        }
        out->armed = (armingFlags & ARMED) != 0;
    }

    // Host-side diagnostics: how long this step took in real time, and what the
    // host actually receives (all-zero motor outputs while armed mean the
    // captured motor packet was missing - the FC state looks fine, but the
    // aircraft would get garbage).
    localRecordStepStats(stepStartUs, out);

    // 1 kHz control-loop burst, once every 12 s (see localBurstLog).
    (void)localBurstLog();
}

// Exposed to the state recorder in wincompat.c.
void sitlLocalStepStats(uint32_t *maxUs, uint32_t *avgUs, uint32_t *steps,
                        float motors[4], uint32_t *zeroWhileArmed)
{
    if (maxUs)  { *maxUs = gLocalStepUsMax; }
    if (avgUs)  { *avgUs = gLocalStepUsAvg; }
    if (steps)  { *steps = gLocalStepCount; }
    if (motors) { memcpy(motors, gLocalLastMotors, sizeof(gLocalLastMotors)); }
    if (zeroWhileArmed) { *zeroWhileArmed = gLocalZeroMotorStepsWhileArmed; }
}

uint64_t sitl_local_time_us(void)
{
    return micros64();
}

// Control-loop signal snapshot for the test harnesses (see sitl_local.h).
// The filtered/scaled gyro lives in wincompat.c, which can include the sensor
// headers this file cannot (see the LOCAL mpuGyroReadRegister stub).
int sitl_local_get_loop_state(sitl_local_loop_state_t *out)
{
    if (out == NULL || !gLocalRunning) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    extern void sitlLocalGetGyroState(float scaled[3], float filtered[3]);
    sitlLocalGetGyroState(out->gyroADC, out->gyroADCf);
    extern void sitlLocalGetGyroChainState(float sampleSum[3], float *lpf1State, float *lpf2State);
    sitlLocalGetGyroChainState(out->sampleSum, &out->lpf1State, &out->lpf2State);
    extern void sitlLocalGetGyroLpfConfig(float *lpf1K, uint8_t *dynFilter, uint16_t *dynMin,
                                          uint16_t *dynMax, uint8_t *dynExpo);
    sitlLocalGetGyroLpfConfig(&out->lpf1K, &out->dynLpfFilter, &out->dynLpfMin,
                              &out->dynLpfMax, &out->dynLpfExpo);
    extern void sitlLocalGetPidProfileInfo(uint32_t *curPtr, uint32_t *pgPtr, uint8_t *idx,
                                           uint8_t *pgRollP);
    sitlLocalGetPidProfileInfo(&out->cfgPidPtr, &out->pgPidPtr, &out->cfgPidIndex,
                               &out->pgRollP);
    extern void sitlLocalGetRpmFilterInfo(float motorHz[4], float notchHz[3],
                                          uint8_t *harmonics, uint8_t *minHz, uint16_t *q,
                                          uint8_t weight[3], float *cycleTimeMultiplier);
    sitlLocalGetRpmFilterInfo(out->rpmMotorHz, out->rpmNotchHz, &out->rpmHarmonics,
                              &out->rpmMinHz, &out->rpmQ, out->rpmWeight,
                              &out->cycleTimeMultiplier);
    for (int axis = 0; axis < 3; axis++) {
        out->rcCommand[axis] = rcCommand[axis];
        out->pidP[axis] = pidData[axis].P;
        out->pidI[axis] = pidData[axis].I;
        out->pidD[axis] = pidData[axis].D;
        out->pidF[axis] = pidData[axis].F;
        out->pidSum[axis] = pidData[axis].Sum;
        out->cfgP[axis] = currentPidProfile->pid[axis].P;
        out->cfgI[axis] = currentPidProfile->pid[axis].I;
        out->cfgD[axis] = currentPidProfile->pid[axis].D;
        out->cfgF[axis] = currentPidProfile->pid[axis].F;
    }
    out->rcCommand[3] = rcCommand[THROTTLE];
    out->pidDeltaUs = (uint32_t)getTaskDeltaTimeUs(TASK_PID);
    out->gyroDeltaUs = (uint32_t)getTaskDeltaTimeUs(TASK_GYRO);
    return 0;
}

void sitl_local_shutdown(void)
{
    if (gLocalRunning) {
        extern void sitlLocalLogControlState(const char *tag);
        sitlLocalLogControlState("shutdown");
    }
    if (gMspThread != NULL) {
        InterlockedExchange(&gMspThreadStop, 1);
        WaitForSingleObject(gMspThread, 1000);
        CloseHandle(gMspThread);
        gMspThread = NULL;
    }
    gLocalRunning = false;

    // Close the virtual EEPROM. sitl.c keeps its own eepromFd and only clears it
    // in configLock(); if it is left open, the next FLASH_Unlock() refuses to
    // start ("[FLASH_Unlock] eepromFd != NULL") and boot hangs - i.e. a host that
    // restarts the FC in process (level reload, PIE restart) would never come
    // back. localFlushEepromWrite() writes the flash mirror and closes it.
    localFlushEepromWrite();

    // Stop the configurator listeners as well, otherwise the next init cannot
    // bind 5761/6761 again.
    extern void serialTcpStop(void);
    extern void wsProxyStop(void);
    serialTcpStop();
    wsProxyStop();
}

#endif // SITL_LOCAL
