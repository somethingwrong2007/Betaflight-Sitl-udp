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
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "flight/imu.h"
#include "fc/rc_modes.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "sitl_gyro.h"
#include "fc/runtime_config.h"
#include "config/feature.h"
#include "msp/msp.h"
#include "rx/rx.h"
#include "sensors/battery.h"

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
    sitlMspSerialProcessReal(evaluateNonMspData, mspProcessCommandFn, mspProcessReplyFn);
    LeaveCriticalSection(&gMspCrit);
}

// udplink_windows.c captures the motor packets pwmCompleteMotorUpdate()
// produces so the DLL can return them without any network I/O.
extern void sitlLocalCaptureMotorPacket(const void *data, size_t size);
extern bool sitlLocalTakeMotorPacket(void *out, size_t size);

static bool gLocalRunning = false;
static HANDLE gMspThread = NULL;
static volatile LONG gMspThreadStop = 0;

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
static uint64_t gLocalRcAnnounceUs = 0;

static uint8_t localRcFrameStatus(rxRuntimeState_t *state)
{
    (void)state;
    if (gLocalRcValid && (micros64() - gLocalRcAnnounceUs) >= 8000) {
        gLocalRcAnnounceUs = micros64();
        rxRuntimeState.lastRcFrameTimeUs = (timeUs_t)(gLocalRcAnnounceUs & 0xFFFFFFFF);
        return RX_FRAME_COMPLETE;
    }
    return RX_FRAME_PENDING;
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
void sitlLocalPreMotorInit(void)
{
    motorConfigMutable()->dev.motorProtocol = MOTOR_PROTOCOL_PWM;
}

// systemReset() defers the EEPROM persist here so it never runs on the UE
// thread (the scheduler may execute systemReset via TASK_SERIAL while the
// configurator exits the CLI panel). The background thread picks it up.
static volatile LONG gLocalPendingReset = 0;

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
    static bool cliWasActive = false;
    while (!gMspThreadStop) {
        if (setjmp(gMspLoopJmp) != 0) {
            sitlAuditLog("MSP thread recovered from reboot jump");
        }
        InterlockedExchange(&gMspJmpReady, 1);
        // cliEnter() sets ARMING_DISABLED_CLI and nothing in the firmware
        // clears it (real FCs reboot to reset it). Watch cliMode transitions
        // so "exit noreboot" (e.g. configurator closes the panel / disconnect
        // injection) also unblocks arming.
        if (cliWasActive && !cliMode) {
            sitlAuditLog("cliMode cleared by watcher");
            unsetArmingDisabled(ARMING_DISABLED_CLI);
        }
        cliWasActive = cliMode;

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

    sitlBoot(0, NULL);

    // Simulated motor RPM participates in the firmware (RPM filter, motor
    // telemetry, OSD/MSP): mark DSHOT telemetry as active so the RPM filter
    // and telemetry consumers use the bridged values from the wrappers.
    useDshotTelemetry = true;

    // The FC blocks arming for powerOnArmingGraceTime seconds after every
    // boot. In-process LOCAL sessions start a new boot each time the host
    // loads the DLL, so remove the grace block immediately.
    unsetArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);

    // Make the local link self-sufficient regardless of the EEPROM contents:
    // force the UDP RX provider (sensor input arrives via sitl_local_step,
    // not a serial receiver) and pin the battery meters to the ADC shims that
    // read simTelemetrySet() values.
    featureEnableImmediate(FEATURE_RX_UDP);

    // Seed the UDP provider's channel count before rxInit() snapshots it into
    // rx.c's file-static rxChannelCount. After the takeover below the frame
    // status callback no longer updates rxChannelCount (frameStatusUdp does),
    // and readRxChannelsApplyRanges()/detectAndApplySignalLossBehaviour()
    // loop over rxChannelCount - if it stays 0 no channel is ever read.
    uint16_t initRc[SITL_LOCAL_MAX_RC_CHANNELS];
    for (int i = 0; i < SITL_LOCAL_MAX_RC_CHANNELS; i++) {
        initRc[i] = 1000;
    }
    rxUpdateUdpChannels(initRc, SITL_LOCAL_MAX_RC_CHANNELS);
    rxInit();

    // Take over the RC provider functions with the cache semantics above.
    rxRuntimeState.rcReadRawFn = localRcReadRaw;
    rxRuntimeState.rcFrameStatusFn = localRcFrameStatus;
    rxRuntimeState.channelCount = SITL_LOCAL_MAX_RC_CHANNELS;

    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
    batteryConfigMutable()->currentMeterSource = CURRENT_METER_ADC;

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

void sitl_local_step(const sitl_local_input_t *in, uint32_t dtUs,
                     sitl_local_output_t *out)
{
    if (out) {
        memset(out, 0, sizeof(*out));
    }
    if (!gLocalRunning || !in) {
        return;
    }

    // A reboot requested a boot-config re-apply (mixer change, filters, ...).
    // Run it here, on the same thread as the scheduler and between steps, so
    // it cannot race the flight loop. Skip while armed and retry on a later
    // step so a save/reboot never yanks the mixer out from under a flying
    // craft.
    extern bool sitlLocalReapplyPending(void);
    extern void sitlLocalRunBootReapply(void);
    if (sitlLocalReapplyPending()) {
        if (!ARMING_FLAG(ARMED)) {
            sitlLocalRunBootReapply();
        } else {
            // Leave the request pending; the next step while disarmed applies it.
            extern void sitlLocalRestoreReapplyPending(void);
            sitlLocalRestoreReapplyPending();
        }
    }

    // --- virtual gyro (Gazebo bridge axis mapping) ---
    double gyroRoll, gyroPitch, gyroYaw;
    sitlGyroBodyFromSim(in->angular_velocity_rpy, ENABLE_GAZEBO_BRIDGE,
                        &gyroRoll, &gyroPitch, &gyroYaw);
    const int16_t gx = (int16_t)constrain(gyroRoll  * LOCAL_GYRO_SCALE * LOCAL_RAD2DEG, -32767, 32767);
    const int16_t gy = (int16_t)constrain(gyroPitch * LOCAL_GYRO_SCALE * LOCAL_RAD2DEG, -32767, 32767);
    const int16_t gz = (int16_t)constrain(gyroYaw   * LOCAL_GYRO_SCALE * LOCAL_RAD2DEG, -32767, 32767);
    virtualGyroSet(virtualGyroDev, gx, gy, gz);

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
    simTelemetrySet(in->battery_voltage, in->battery_current, in->motor_rpm, 4);

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
    uint32_t remainingUs = dtUs;
    const uint32_t quantumUs = 100;
    while (remainingUs >= quantumUs) {
        sitlStepTime(quantumUs);
        remainingUs -= quantumUs;
        scheduler();
    }
    if (remainingUs > 0) {
        sitlStepTime(remainingUs);
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
}

uint64_t sitl_local_time_us(void)
{
    return micros64();
}

void sitl_local_shutdown(void)
{
    if (gMspThread != NULL) {
        InterlockedExchange(&gMspThreadStop, 1);
        WaitForSingleObject(gMspThread, 1000);
        CloseHandle(gMspThread);
        gMspThread = NULL;
    }
    gLocalRunning = false;
}

#endif // SITL_LOCAL
