/**
 * Small POSIX compatibility shims for the MinGW build of Betaflight SITL.
 */

#include <string.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include "platform.h"
#include "drivers/io.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "sitl_local.h"
#include "config/config.h"
#include "build/debug.h"
#include "drivers/motor.h"
#include "drivers/servo_impl.h"
#include "flight/pid.h"
#include "flight/pid_init.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"
#include "flight/autopilot.h"
#include "config/feature.h"
#include "sensors/battery.h"
#include "flight/pid.h"
#include "fc/controlrate_profile.h"
#include "pg/rx.h"
#include "sensors/acceleration.h"
#include "sensors/gyro_init.h"
#include "sensors/gyro.h"
#include "sensors/boardalignment.h"
#include "drivers/dshot.h"
#include "pg/motor.h"
#include "pg/dyn_notch.h"
#include "pg/rpm_filter.h"

#ifdef _WIN32
#include <windows.h>
#include <pthread.h>
#include <dirent.h>

#include "common/time.h"
#include "sensors/voltage.h"
#include "sensors/current.h"
#include "fc/runtime_config.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "cli/cli.h"
#include "sim_telemetry.h"

extern uint64_t micros64_real(void);
extern void sitlDelayMicroseconds(uint32_t us);
extern void writeEEPROM(void);
extern void sitlSystemResetNative(void);
extern void sitlLocalRebootJump(void);
void systemReset(void);
#ifdef USE_BLACKBOX
extern void blackboxFinish(void);
#endif

// Append one line to %LOCALAPPDATA%\Betaflight-SITL\sitl-audit.log so save /
// connection problems in the in-process LOCAL build are traceable without
// capturing the host process stderr.
void sitlAuditLog(const char *fmt, ...)
{
    char path[MAX_PATH];
    if (GetEnvironmentVariableA("LOCALAPPDATA", path, sizeof(path)) <= 0) {
        return;
    }
    strncat(path, "\\Betaflight-SITL\\sitl-audit.log", sizeof(path) - strlen(path) - 1);

    FILE *log = fopen(path, "a");
    if (log == NULL) {
        return;
    }
    fprintf(log, "[%llu] ", (unsigned long long)(micros64_real() / 1000));
    va_list ap;
    va_start(ap, fmt);
    vfprintf(log, fmt, ap);
    va_end(ap);
    fprintf(log, "\n");
    fclose(log);
}

// The firmware only syncs the debugMode global from systemConfig()->debug_mode
// at boot (fc/init.c). Real hardware re-runs init on every reboot, but the
// LOCAL reboot keeps the process alive, so a debug_mode change (e.g. CHIRP for
// blackbox auto-tuning) would never reach the blackbox/DEBUG_SET path. Re-sync
// whenever a config write or reboot happens. No-op outside LOCAL mode.
void sitlLocalSyncDebugMode(void)
{
#ifdef SITL_LOCAL
    debugMode = systemConfig()->debug_mode;
#endif
}

// A firmware reboot, reproduced in-process. Every reboot path ends up here:
// the configurator's "Save and Reboot" (msp.c -> systemReset), the CLI
// "save"/"exit"/"defaults", CMS, and the DFU request. Real hardware re-runs
// init on every reboot; here that means re-reading the EEPROM and re-applying
// everything derived from it (see localRunPendingReboot() in sitl_local.c).
//
// A real reboot drops the motor output, forgets the runtime state and starts
// over from the EEPROM. The cheap, thread-safe half of that runs here - the
// caller may be the MSP thread or the host thread (a CLI save processed by
// TASK_SERIAL). The config re-read and the derived re-init are queued for
// sitl_local_step(), because re-initialising the mixer, motors, servos and IMU
// from another thread while the host is stepping crashes the process.
void sitlLocalRequestReboot(void)
{
#ifdef SITL_LOCAL
    extern void sitlLocalRequestRebootApply(void);

    // The reboot ends the log; the re-init below opens a new one.
#ifdef USE_BLACKBOX
    blackboxFinish();
#endif

    // A reboot drops the motor output. Disarming here (instead of waiting for
    // the ARM switch) also resets the arming state machines and the PID
    // integrators, and lets the queued re-init run on the very next step.
    if (ARMING_FLAG(ARMED)) {
        disarm(DISARM_REASON_ARMING_DISABLED);
    }
    resetTryingToArm();
    resetArmingDisabled();

    // Reboot-only runtime state: the CLI link is rebuilt from scratch and a
    // pending "reboot required" request is satisfied by this reboot.
    // ARMING_DISABLED_MSP belongs to the configurator (it re-asserts it while
    // it holds the FC in a config state), so that one is deliberately kept.
    cliMode = false;
    unsetArmingDisabled(ARMING_DISABLED_CLI | ARMING_DISABLED_REBOOT_REQUIRED);

    sitlLocalRequestRebootApply();
#endif
}

// --- runtime state flight recorder ------------------------------------------
// A configurator Save (or any MSP write) can change state that no configurator
// page shows: applied runtime values, the *runtime* feature mask, the failsafe /
// RX state machines, the applied mixer runtime, ... Those are exactly the
// differences a DLL restart clears, so log a fingerprint of everything
// control-relevant whenever any of it changes. Cheap: collected every ~1000
// steps, only written to the audit log when something actually differs.
static uint32_t fnv32Struct(const void *data, size_t len)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        hash = (hash ^ bytes[i]) * 16777619u;
    }
    return hash;
}

typedef struct {
    uint32_t hashSystem, hashPid, hashPidProfiles, hashRates, hashGyro, hashMotor, hashBattery,
             hashMixer, hashFeature, hashRx, hashImu, hashAcc;
    uint8_t  protocol, poles, dshotCfg, dshotEdt, bitbang, motorIdle;
    uint16_t maxthrottle, mincommand;
    uint8_t  voltSrc, currSrc, rcOurs;
    uint8_t  mixerCfg, mixerRt, motorCount, denom;
    uint16_t targetLooptime;
    uint32_t pidDtMicro;
    uint8_t  rtAirMode, rtAntiGravity, rtRxUdp, rtGps, rt3d, rtEscSensor;
    uint16_t lpf1, lpf1DynMin, lpf1DynMax, lpf2, notch1Hz, notch1Cut, notch2Hz, notch2Cut;
    uint8_t  pidProfileIdx, ratesType, armed;
    uint16_t pidP[3], pidI[3], pidD[3], pidF[3];
    uint8_t  rcRate[3], rates[3], expo[3];
    uint32_t armDisableFlags;
} sitlLocalStateFp_t;

static void sitlCollectStateFp(sitlLocalStateFp_t *fp)
{
    memset(fp, 0, sizeof(*fp));
    extern bool sitlLocalRcTakeOverActive(void);

    fp->hashSystem = fnv32Struct(systemConfig(), sizeof(systemConfig_t));
    fp->hashPid = fnv32Struct(pidConfig(), sizeof(pidConfig_t));
    // The profile contents live in a separate PG array; without this a
    // configurator Save that overwrites the tuned PIDs (and the advanced PID
    // settings) with its cached values would be invisible.
    fp->hashPidProfiles = fnv32Struct(&pidProfiles(0)[0], sizeof(pidProfile_t) * PID_PROFILE_COUNT);
    fp->hashRates = fnv32Struct(&controlRateProfiles(0)[0], sizeof(controlRateConfig_t) * CONTROL_RATE_PROFILE_COUNT);
    fp->hashGyro = fnv32Struct(gyroConfig(), sizeof(gyroConfig_t));
    fp->hashMotor = fnv32Struct(motorConfig(), sizeof(motorConfig_t));
    fp->hashBattery = fnv32Struct(batteryConfig(), sizeof(batteryConfig_t));
    fp->hashMixer = fnv32Struct(mixerConfig(), sizeof(mixerConfig_t));
    fp->hashFeature = fnv32Struct(featureConfig(), sizeof(featureConfig_t));
    fp->hashRx = fnv32Struct(rxConfig(), sizeof(rxConfig_t));
    fp->hashImu = fnv32Struct(imuConfig(), sizeof(imuConfig_t));
    fp->hashAcc = fnv32Struct(accelerometerConfig(), sizeof(accelerometerConfig_t));

    fp->protocol = motorConfig()->dev.motorProtocol;
    fp->poles = motorConfig()->motorPoleCount;
    fp->dshotCfg = motorConfig()->dev.useDshotTelemetry;
    fp->dshotEdt = motorConfig()->dev.useDshotEdt;
    fp->bitbang = motorConfig()->dev.useDshotBitbang;
    fp->motorIdle = motorConfig()->motorIdle;
    fp->maxthrottle = motorConfig()->maxthrottle;
    fp->mincommand = motorConfig()->mincommand;
    fp->voltSrc = batteryConfig()->voltageMeterSource;
    fp->currSrc = batteryConfig()->currentMeterSource;
    fp->rcOurs = sitlLocalRcTakeOverActive() ? 1 : 0;
    fp->mixerCfg = mixerConfig()->mixerMode;
    fp->mixerRt = getMixerMode();
    fp->motorCount = getMotorCount();
    fp->denom = pidConfig()->pid_process_denom;
    fp->targetLooptime = (uint16_t)gyro.targetLooptime;
    fp->pidDtMicro = (uint32_t)(pidGetDT() * 1e6f + 0.5f);
    fp->rtAirMode = featureIsEnabled(FEATURE_AIRMODE) ? 1 : 0;
    fp->rtAntiGravity = featureIsEnabled(FEATURE_ANTI_GRAVITY) ? 1 : 0;
    fp->rtRxUdp = featureIsEnabled(FEATURE_RX_UDP) ? 1 : 0;
    fp->rtGps = featureIsEnabled(FEATURE_GPS) ? 1 : 0;
    fp->rt3d = featureIsEnabled(FEATURE_3D) ? 1 : 0;
    fp->rtEscSensor = featureIsEnabled(FEATURE_ESC_SENSOR) ? 1 : 0;
    fp->lpf1 = gyroConfig()->gyro_lpf1_static_hz;
    fp->lpf1DynMin = gyroConfig()->gyro_lpf1_dyn_min_hz;
    fp->lpf1DynMax = gyroConfig()->gyro_lpf1_dyn_max_hz;
    fp->lpf2 = gyroConfig()->gyro_lpf2_static_hz;
    fp->notch1Hz = gyroConfig()->gyro_soft_notch_hz_1;
    fp->notch1Cut = gyroConfig()->gyro_soft_notch_cutoff_1;
    fp->notch2Hz = gyroConfig()->gyro_soft_notch_hz_2;
    fp->notch2Cut = gyroConfig()->gyro_soft_notch_cutoff_2;
    fp->pidProfileIdx = systemConfig()->pidProfileIndex;
    fp->ratesType = currentControlRateProfile ? currentControlRateProfile->rates_type : 0;
    fp->armed = ARMING_FLAG(ARMED) ? 1 : 0;
    for (int axis = 0; axis < 3; axis++) {
        fp->pidP[axis] = currentPidProfile->pid[axis].P;
        fp->pidI[axis] = currentPidProfile->pid[axis].I;
        fp->pidD[axis] = currentPidProfile->pid[axis].D;
        fp->pidF[axis] = currentPidProfile->pid[axis].F;
        if (currentControlRateProfile) {
            fp->rcRate[axis] = currentControlRateProfile->rcRates[axis];
            fp->rates[axis] = currentControlRateProfile->rates[axis];
            fp->expo[axis] = currentControlRateProfile->rcExpo[axis];
        }
    }
    fp->armDisableFlags = (uint32_t)getArmingDisableFlags();
}

void sitlLocalLogStateIfChanged(const char *tag)
{
#ifdef SITL_LOCAL
    static sitlLocalStateFp_t last;
    static bool haveLast = false;

    sitlLocalStateFp_t now;
    sitlCollectStateFp(&now);
    if (haveLast && memcmp(&last, &now, sizeof(now)) == 0) {
        return;
    }
    const bool first = !haveLast;
    last = now;
    haveLast = true;

    sitlAuditLog("%s%s hash sys=%08X pid=%08X pidProf=%08X rates=%08X gyro=%08X motor=%08X batt=%08X "
                 "mix=%08X feat=%08X rx=%08X imu=%08X acc=%08X",
                 tag, first ? " (initial)" : "",
                 now.hashSystem, now.hashPid, now.hashPidProfiles, now.hashRates, now.hashGyro, now.hashMotor,
                 now.hashBattery, now.hashMixer, now.hashFeature, now.hashRx, now.hashImu,
                 now.hashAcc);
    extern void sitlLocalStepStats(uint32_t *maxUs, uint32_t *avgUs, uint32_t *steps,
                                   float motors[4], uint32_t *zeroWhileArmed);
    extern void sitlLocalMotorStreamStats(uint32_t *captured, uint32_t *taken, uint32_t *missed);
    extern void sitlLocalMspLoad(uint32_t *busyUs, uint32_t *calls);
    uint32_t stepMaxUs = 0, stepAvgUs = 0, stepCount = 0, zeroArmed = 0;
    uint32_t capPkts = 0, takePkts = 0, missPkts = 0;
    uint32_t mspBusyUs = 0, mspCalls = 0;
    extern uint32_t sitlLocalRcFrameCount(void);
    const uint32_t rcFrames = sitlLocalRcFrameCount();
    float motors[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    sitlLocalStepStats(&stepMaxUs, &stepAvgUs, &stepCount, motors, &zeroArmed);
    sitlLocalMotorStreamStats(&capPkts, &takePkts, &missPkts);
    sitlLocalMspLoad(&mspBusyUs, &mspCalls);

    sitlAuditLog("%s rt protocol=%u dshot(cfg/edt/bb)=%u/%u/%u idle=%u thr=%u/%u poles=%u "
                 "battSrc=%u/%u rcOurs=%u mixer(cfg/rt/count)=%u/%u/%u denom=%u looptime=%u pidDt=%u "
                 "stepUs(max/avg/count)=%u/%u/%u motorPkts(cap/take/miss)=%u/%u/%u "
                 "motors=%.0f/%.0f/%.0f/%.0f zeroWhileArmed=%u "
                 "mspThread(busyUs/calls)=%u/%u "
                 "rcFrames=%u "
                 "featRt(air/ag/udp/gps/3d/esc)=%u/%u/%u/%u/%u/%u "
                 "lpf=%u/%u-%u/%u notch=%u/%u/%u/%u pidProf=%u ratesType=%u armed=%u armFlags=%08X "
                 "modes=%04X att=%d/%d/%d gyroADC=%.0f/%.0f/%.0f "
                 "pidR=%u/%u/%u/%u pidP=%u/%u/%u/%u pidY=%u/%u/%u/%u "
                 "dtermLpf=%u/%u-%u/%u itermRelax=%u/%u antiGrav=%u",
                 tag,
                 (unsigned)now.protocol, (unsigned)now.dshotCfg,
                 (unsigned)now.dshotEdt, (unsigned)now.bitbang, (unsigned)now.motorIdle,
                 (unsigned)now.maxthrottle, (unsigned)now.mincommand, (unsigned)now.poles,
                 (unsigned)now.voltSrc, (unsigned)now.currSrc, (unsigned)now.rcOurs,
                 (unsigned)now.mixerCfg, (unsigned)now.mixerRt, (unsigned)now.motorCount,
                 (unsigned)now.denom, (unsigned)now.targetLooptime, (unsigned)now.pidDtMicro,
                 (unsigned)stepMaxUs, (unsigned)stepAvgUs, (unsigned)stepCount,
                 (unsigned)capPkts, (unsigned)takePkts, (unsigned)missPkts,
                 (double)motors[0], (double)motors[1], (double)motors[2], (double)motors[3],
                 (unsigned)zeroArmed,
                 (unsigned)mspBusyUs, (unsigned)mspCalls,
                 (unsigned)rcFrames,
                 (unsigned)now.rtAirMode, (unsigned)now.rtAntiGravity, (unsigned)now.rtRxUdp,
                 (unsigned)now.rtGps, (unsigned)now.rt3d, (unsigned)now.rtEscSensor,
                 (unsigned)now.lpf1, (unsigned)now.lpf1DynMin, (unsigned)now.lpf1DynMax,
                 (unsigned)now.lpf2, (unsigned)now.notch1Hz, (unsigned)now.notch1Cut,
                 (unsigned)now.notch2Hz, (unsigned)now.notch2Cut, (unsigned)now.pidProfileIdx,
                 (unsigned)now.ratesType, (unsigned)now.armed, now.armDisableFlags,
                 (unsigned)flightModeFlags,
                 (int)attitude.values.roll, (int)attitude.values.pitch, (int)attitude.values.yaw,
                 (double)gyro.gyroADC[0], (double)gyro.gyroADC[1], (double)gyro.gyroADC[2],
                 now.pidP[0], now.pidI[0], now.pidD[0], now.pidF[0],
                 now.pidP[1], now.pidI[1], now.pidD[1], now.pidF[1],
                 now.pidP[2], now.pidI[2], now.pidD[2], now.pidF[2],
                 (unsigned)currentPidProfile->dterm_lpf1_static_hz,
                 (unsigned)currentPidProfile->dterm_lpf1_dyn_min_hz,
                 (unsigned)currentPidProfile->dterm_lpf1_dyn_max_hz,
                 (unsigned)currentPidProfile->dterm_lpf2_static_hz,
                 (unsigned)currentPidProfile->iterm_relax,
                 (unsigned)currentPidProfile->iterm_relax_cutoff,
                 (unsigned)currentPidProfile->anti_gravity_gain);
#else
    UNUSED(tag);
#endif
}

// A fresh process starts with every firmware module static zeroed (the loader
// clears .bss before main). An in-process restart re-runs init() on top of the
// previous run's RAM instead, and Betaflight's init() does NOT rebuild all of
// that: measured with tools/sitl_local_save_compare's reinit mode, a restarted
// FC kept the previous flight's PID integrators, part of the PID runtime (D-term
// / feedforward / anti-gravity history) and the attitude estimate, and then
// answered the same stick input with a stable ~236 us different motor output -
// up to 24% of the 1000..2000 us range during the transient. That is the
// "aircraft shakes after a restart / after saving, a fresh process is fine"
// symptom: the restart left the control loop on a different operating point.
//
// Reset those leftovers *before* init() runs, so the boot derives every chain
// from a clean slate exactly like the cold start the rest of the pipeline is
// validated against (a fresh process). Running at boot with the loop stopped,
// this cannot inject a step into a live filter - unlike the mid-flight reloads
// that caused the original shake.
void sitlLocalResetVolatileControlState(void)
{
#ifdef SITL_LOCAL
    memset(&pidRuntime, 0, sizeof(pidRuntime));
    memset(pidData, 0, sizeof(pidData));

    // Attitude / rotation state: a fresh boot starts from an all-zero
    // quaternion, which imuComputeRotationMatrix() turns into the identity.
    memset(&attitude, 0, sizeof(attitude));
    memset(&rMat, 0, sizeof(rMat));
    memset(&imuAttitudeQuaternion, 0, sizeof(imuAttitudeQuaternion));
    // The estimator keeps its own file-static quaternion, which imuInit() reads
    // to derive rMat - so it has to start at the fresh-boot value as well,
    // otherwise a restart keeps pointing the OSD/attitude (and angle modes)
    // along the previous run's last orientation until the filter converges.
    imuSetAttitudeQuat(1.0f, 0.0f, 0.0f, 0.0f);

    // Gyro sampling state the boot path does not clear (the filter *coeffs* are
    // rebuilt by gyroInitFilters(), the raw/filtered values are leftovers).
    memset(gyro.gyroADC, 0, sizeof(gyro.gyroADC));
    memset(gyro.gyroADCf, 0, sizeof(gyro.gyroADCf));
    memset(gyro.sampleSum, 0, sizeof(gyro.sampleSum));
    gyro.sampleCount = 0;
    memset(gyro.imuGyroFilter, 0, sizeof(gyro.imuGyroFilter));

    sitlAuditLog("boot: volatile control state reset (PID/PID-runtime/attitude/gyro leftovers)");
#endif
}

// Boot fingerprint of the *derived* control chains (not the config records).
//
// A fresh process starts with every firmware module static zeroed by the
// loader; an in-process restart re-runs init() on top of the previous run's
// RAM, so anything init() does not explicitly rebuild keeps the old value.
// Where the two boots end up deriving different chains, a restarted FC flies
// differently from a freshly loaded one - the "aircraft shakes after a
// restart" report (tools/sitl_local_save_compare's reinit mode measures it).
// Logged at sitl_local_init()/sitl_local_shutdown() so the two boots can be
// compared value by value in the audit log.
// Exposes the gyro signals the rate loop runs on to sitl_local.c's
// sitl_local_get_loop_state() (this file is the only one that may include
// sensors/gyro.h in the LOCAL build).
void sitlLocalGetGyroState(float scaled[3], float filtered[3])
{
#ifdef SITL_LOCAL
    for (int axis = 0; axis < 3; axis++) {
        if (scaled)   { scaled[axis] = gyro.gyroADC[axis]; }
        if (filtered) { filtered[axis] = gyro.gyroADCf[axis]; }
    }
#else
    UNUSED(scaled);
    UNUSED(filtered);
#endif
}

void sitlLocalGetGyroChainState(float sampleSum[3], float *lpf1State, float *lpf2State)
{
#ifdef SITL_LOCAL
    for (int axis = 0; axis < 3; axis++) {
        sampleSum[axis] = gyro.sampleSum[axis];
    }
    if (lpf1State) { *lpf1State = gyro.lowpassFilter[0].pt1FilterState.state; }
    if (lpf2State) { *lpf2State = gyro.lowpass2Filter[0].pt1FilterState.state; }
#else
    UNUSED(sampleSum);
    UNUSED(lpf1State);
    UNUSED(lpf2State);
#endif
}

// Notch frequencies the RPM filter is using right now (motor 0's harmonics), for
// the 1 kHz burst recorder: a shake that lines up with one of these, and the RPM
// behind it, is what tells a filter problem from a tuning problem.
// The FC's configured gyro/PID grid period, for sitl_local_step's de-jitter snap
// (sensors/gyro.h is not includable from sitl_local.c).
uint32_t sitlLocalGyroGridUs(void)
{
#ifdef SITL_LOCAL
    return gyro.targetLooptime ? gyro.targetLooptime : 1000;
#else
    return 1000;
#endif
}

// The RPM the host actually sent (before the firmware-style lowpass), for the
// input-data view.
void sitlLocalGetRpmRawMotorHz(float motorHz[4])
{
#ifdef SITL_LOCAL
    extern float simTelemetryMotorFrequencyHz(uint8_t motorIndex);
    for (int m = 0; m < 4; m++) {
        motorHz[m] = simTelemetryMotorFrequencyHz((uint8_t)m);
    }
#else
    UNUSED(motorHz);
#endif
}

// RPM filter settings as the firmware has them, for the diagnostics: fade range
// and weights decide whether a clamped harmonic is applied as a false notch.
void sitlLocalGetRpmFilterSettings(uint16_t *fadeRangeHz, uint16_t *lpfHz)
{
#ifdef SITL_LOCAL
    if (fadeRangeHz) { *fadeRangeHz = rpmFilterConfig()->rpm_filter_fade_range_hz; }
    if (lpfHz)       { *lpfHz = rpmFilterConfig()->rpm_filter_lpf_hz; }
#else
    UNUSED(fadeRangeHz);
    UNUSED(lpfHz);
#endif
}

// 1 Hz record of the RPM *input data* as the FC receives it, next to what the
// filter derives from it. Only written when the rounded values change, so it
// costs one line per second while flying and shows immediately whether the host
// is sending sane per-motor speeds (rpm -> /60 -> mechanical Hz -> notch Hz).
void sitlLocalLogRpmInput(void)
{
#ifdef SITL_LOCAL
    static int lastKey = -1;
    extern float simTelemetryMotorFrequencyHz(uint8_t motorIndex);

    float rawHz[4];
    float filtHz[4];
    for (int i = 0; i < 4; i++) {
        rawHz[i] = simTelemetryMotorFrequencyHz((uint8_t)i);
        extern float getMotorFrequencyHz(uint8_t motorIndex);
        filtHz[i] = getMotorFrequencyHz((uint8_t)i);
    }
    float notch[3];
    sitlLocalGetRpmNotchHz(notch);
    const uint8_t harmonics = rpmFilterConfig()->rpm_filter_harmonics;
    // A clamped harmonic is applied at the loop's Nyquist limit; with the
    // default fade range it is the harmful case (full-weight notch at ~0.96x
    // Nyquist). Report how many of the configured harmonics are clamped.
    const float maxHz = 0.48f * 1e6f / (float)(gyro.targetLooptime ? gyro.targetLooptime : 1000);
    unsigned clamped = 0;
    for (int h = 0; h < harmonics && h < 3; h++) {
        if ((float)(h + 1) * filtHz[0] > maxHz) { clamped++; }
    }

    const int key = (int)(rawHz[0] + 0.5f) * 1000 + (int)(notch[0] + 0.5f);
    if (key == lastKey) {
        return;
    }
    lastKey = key;

    sitlAuditLog("rpm input: host rpm=%.0f/%.0f/%.0f/%.0f (%.1f/%.1f/%.1f/%.1f Hz) "
                 "filtered=%.1f Hz notch=%.0f/%.0f/%.0f Hz harmonics=%u clamped=%u "
                 "dcut=%.0f Hz",
                 (double)(rawHz[0] * 60.0f), (double)(rawHz[1] * 60.0f),
                 (double)(rawHz[2] * 60.0f), (double)(rawHz[3] * 60.0f),
                 (double)rawHz[0], (double)rawHz[1], (double)rawHz[2], (double)rawHz[3],
                 (double)filtHz[0],
                 (double)notch[0], (double)notch[1], (double)notch[2],
                 (unsigned)harmonics, clamped, (double)maxHz);
#endif
}

// The other firmware loops that consume the bridged RPM and are gated by the
// same bidirectional-DShot flag: dynamic idle (per profile) and the RPM limiter
// (mixer config). If either is configured, the *input* RPM drives a feedback
// loop whose behaviour depends entirely on how realistic that data is.
void sitlLocalLogRpmLoops(void)
{
#ifdef SITL_LOCAL
#ifdef USE_RPM_LIMIT
    const unsigned rpmLimit = mixerConfig()->rpm_limit ? 1 : 0;
    const unsigned rpmLimitValue = mixerConfig()->rpm_limit_value;
    const unsigned rpmLimitP = mixerConfig()->rpm_limit_p;
    const unsigned rpmLimitI = mixerConfig()->rpm_limit_i;
    const unsigned rpmLimitD = mixerConfig()->rpm_limit_d;
#else
    const unsigned rpmLimit = 0, rpmLimitValue = 0, rpmLimitP = 0, rpmLimitI = 0, rpmLimitD = 0;
#endif
    sitlAuditLog("rpm loops: dyn_idle_min_rpm=%u gains=%u/%u/%u "
                 "rpm_limit=%u value=%u gains=%u/%u/%u",
                 (unsigned)currentPidProfile->dyn_idle_min_rpm,
                 (unsigned)currentPidProfile->dyn_idle_p_gain,
                 (unsigned)currentPidProfile->dyn_idle_i_gain,
                 (unsigned)currentPidProfile->dyn_idle_d_gain,
                 rpmLimit, rpmLimitValue, rpmLimitP, rpmLimitI, rpmLimitD);
#endif
}

void sitlLocalGetRpmNotchHz(float notchHz[3])
{
#ifdef SITL_LOCAL
    extern float getMotorFrequencyHz(uint8_t motorIndex);
    const uint8_t harmonics = rpmFilterConfig()->rpm_filter_harmonics;
    const float minHz = rpmFilterConfig()->rpm_filter_min_hz;
    const float maxHz = 0.48f * 1e6f / (float)(gyro.targetLooptime ? gyro.targetLooptime : 1000);
    const float f0 = getMotorFrequencyHz(0);
    for (int h = 0; h < 3; h++) {
        float f = (h < harmonics) ? (float)(h + 1) * f0 : 0.0f;
        if (f < minHz) { f = minHz; }
        if (f > maxHz) { f = maxHz; }
        notchHz[h] = f;
    }
#else
    UNUSED(notchHz);
#endif
}

// What the RPM filter actually does: the per-motor mechanical frequency the
// bridge hands it, the notch frequency it derives (clamped to minHz..0.48*1/dt),
// the config behind it, and the scheduler's dt compensation. A notch that ends
// up inside the control band (or all of them clamped to one frequency near
// Nyquist) is visible here.
void sitlLocalGetRpmFilterInfo(float motorHz[4], float notchHz[3], uint8_t *harmonics,
                               uint8_t *minHz, uint16_t *q, uint8_t weight[3],
                               float *cycleTimeMultiplier)
{
#ifdef SITL_LOCAL
    extern float getMotorFrequencyHz(uint8_t motorIndex);
    extern float schedulerGetCycleTimeMultiplier(void);
    extern float simTelemetryMotorFrequencyHz(uint8_t motorIndex);
    const rpmFilterConfig_t *cfg = rpmFilterConfig();
    const uint8_t numHarmonics = cfg->rpm_filter_harmonics;
    const float maxHz = 0.48f * 1e6f / (float)gyro.targetLooptime;

    for (int m = 0; m < 4; m++) {
        motorHz[m] = getMotorFrequencyHz((uint8_t)m);
    }
    for (int h = 0; h < 3; h++) {
        float f = (float)(h + 1) * motorHz[0];
        if (f < (float)cfg->rpm_filter_min_hz) { f = (float)cfg->rpm_filter_min_hz; }
        if (f > maxHz) { f = maxHz; }
        notchHz[h] = f;
        weight[h] = h < RPM_FILTER_HARMONICS_MAX ? cfg->rpm_filter_weights[h] : 0;
    }
    if (harmonics) { *harmonics = numHarmonics; }
    if (minHz)  { *minHz = cfg->rpm_filter_min_hz; }
    if (q)      { *q = cfg->rpm_filter_q; }
    if (cycleTimeMultiplier) { *cycleTimeMultiplier = schedulerGetCycleTimeMultiplier(); }
#else
    UNUSED(motorHz); UNUSED(notchHz); UNUSED(harmonics); UNUSED(minHz); UNUSED(q);
    UNUSED(weight); UNUSED(cycleTimeMultiplier);
#endif
}

// Diagnostic: which profile object does the loop read, and what does the PG
// array hold? MSP writes land in the PG record; if these disagree, a config
// write is going somewhere the loop never looks.
void sitlLocalGetPidProfileInfo(uint32_t *curPtr, uint32_t *pgPtr, uint8_t *idx,
                                uint8_t *pgRollP)
{
#ifdef SITL_LOCAL
    if (curPtr)  { *curPtr = (uint32_t)(uintptr_t)currentPidProfile; }
    if (pgPtr)   { *pgPtr = (uint32_t)(uintptr_t)&pidProfiles(0)[0]; }
    if (idx)     { *idx = (uint8_t)systemConfig()->pidProfileIndex; }
    if (pgRollP) { *pgRollP = (uint8_t)pidProfiles(0)[0].pid[0].P; }
#else
    UNUSED(curPtr); UNUSED(pgPtr); UNUSED(idx); UNUSED(pgRollP);
#endif
}

void sitlLocalGetGyroLpfConfig(float *lpf1K, uint8_t *dynFilter, uint16_t *dynMin,
                               uint16_t *dynMax, uint8_t *dynExpo)
{
#ifdef SITL_LOCAL
    if (lpf1K)    { *lpf1K = gyro.lowpassFilter[0].pt1FilterState.k; }
    if (dynFilter){ *dynFilter = gyro.dynLpfFilter; }
    if (dynMin)   { *dynMin = gyro.dynLpfMin; }
    if (dynMax)   { *dynMax = gyro.dynLpfMax; }
    if (dynExpo)  { *dynExpo = gyro.dynLpfCurveExpo; }
#else
    UNUSED(lpf1K); UNUSED(dynFilter); UNUSED(dynMin); UNUSED(dynMax); UNUSED(dynExpo);
#endif
}

void sitlLocalLogControlState(const char *tag)
{
#ifdef SITL_LOCAL
    sitlAuditLog("ctrl %s: gyro(sr=%u sl=%u tl=%u scale=%.6f down=%u) denom=%u dT=%.9g f=%.6f "
                 "lpf1k=%.9g ntc1a1=%.9g ntc1fq=%.9g dtermLpfK=%.9g itermRelaxCut=%u",
                 tag,
                 (unsigned)gyro.sampleRateHz, (unsigned)gyro.sampleLooptime,
                 (unsigned)gyro.targetLooptime, (double)gyro.scale,
                 (unsigned)(gyro.downsampleFilterEnabled ? 1 : 0),
                 (unsigned)activePidLoopDenom,
                 (double)pidRuntime.dT, (double)pidRuntime.pidFrequency,
                 (double)gyro.lowpassFilter[0].pt1FilterState.k,
                 (double)gyro.notchFilter1[0].a1,
                 (double)gyro.notchFilter1[0].fq,
                 (double)pidRuntime.dtermLowpass[0].pt1Filter.k,
                 (unsigned)pidRuntime.itermRelaxCutoff);
    sitlAuditLog("ctrl %s: h(gyroLpf1=%08X lpf2=%08X ntc1=%08X ntc2=%08X imuG=%08X) "
                 "h(pidRuntime=%08X pidData=%08X att=%08X rMat=%08X quat=%08X) "
                 "fn(lpf1=%p lpf2=%p ntc1=%p ntc2=%p)",
                 tag,
                 fnv32Struct(&gyro.lowpassFilter, sizeof(gyro.lowpassFilter)),
                 fnv32Struct(&gyro.lowpass2Filter, sizeof(gyro.lowpass2Filter)),
                 fnv32Struct(&gyro.notchFilter1, sizeof(gyro.notchFilter1)),
                 fnv32Struct(&gyro.notchFilter2, sizeof(gyro.notchFilter2)),
                 fnv32Struct(&gyro.imuGyroFilter, sizeof(gyro.imuGyroFilter)),
                 fnv32Struct(&pidRuntime, sizeof(pidRuntime)),
                 fnv32Struct(&pidData, sizeof(pidData)),
                 fnv32Struct(&attitude, sizeof(attitude)),
                 fnv32Struct(&rMat, sizeof(rMat)),
                 fnv32Struct(&imuAttitudeQuaternion, sizeof(imuAttitudeQuaternion)),
                 (void *)gyro.lowpassFilterApplyFn, (void *)gyro.lowpass2FilterApplyFn,
                 (void *)gyro.notchFilter1ApplyFn, (void *)gyro.notchFilter2ApplyFn);
    sitlAuditLog("ctrl %s: gyroADC=%.3f/%.3f/%.3f gyroADCf=%.3f/%.3f/%.3f "
                 "pidData(P=%.3f/%.3f/%.3f I=%.3f/%.3f/%.3f D=%.3f/%.3f/%.3f)",
                 tag,
                 (double)gyro.gyroADC[0], (double)gyro.gyroADC[1], (double)gyro.gyroADC[2],
                 (double)gyro.gyroADCf[0], (double)gyro.gyroADCf[1], (double)gyro.gyroADCf[2],
                 (double)pidData[0].P, (double)pidData[1].P, (double)pidData[2].P,
                 (double)pidData[0].I, (double)pidData[1].I, (double)pidData[2].I,
                 (double)pidData[0].D, (double)pidData[1].D, (double)pidData[2].D);
#else
    UNUSED(tag);
#endif
}

// The gyro filter chain (LPF1/LPF2, notches, dynamic notch, RPM filter) is
// stateful, and zeroing that state while the host keeps feeding gyro samples
// injects a step into the filtered rate: the PID's D-term (delta/dt) turns it
// into a full-authority spike, which shows up as violent oscillation right
// after a reload/reboot. Real hardware only rebuilds these filters at boot or
// when the filter settings change (a real reboot also restarts the gyro
// stream, so a zero state means something), while the LOCAL link's gyro stream
// never stops. Snapshot the filter configuration around the EEPROM read and
// re-init only when it actually changed.
static gyroConfig_t gGyroFilterConfigSnapshot;
// rpmFilterInit() returns early when useDshotTelemetry is false, so the EEPROM's
// bidirectional-DShot flag decides whether the RPM filter exists at all - a
// change to it has to count as a chain change (and re-derive the filter on a
// reboot/reload, exactly like a real FC after enabling bdshot).
static uint8_t gGyroFilterDshotConfigSnapshot;
#ifdef USE_DYN_NOTCH_FILTER
static dynNotchConfig_t gDynNotchConfigSnapshot;
#endif
#ifdef USE_RPM_FILTER
static rpmFilterConfig_t gRpmFilterConfigSnapshot;
#endif
static bool gGyroFilterConfigSnapshotValid = false;

void sitlLocalSnapshotGyroFilterConfig(void)
{
#ifdef SITL_LOCAL
    gGyroFilterConfigSnapshot = *gyroConfig();
    gGyroFilterDshotConfigSnapshot = motorConfig()->dev.useDshotTelemetry;
#ifdef USE_DYN_NOTCH_FILTER
    gDynNotchConfigSnapshot = *dynNotchConfig();
#endif
#ifdef USE_RPM_FILTER
    gRpmFilterConfigSnapshot = *rpmFilterConfig();
#endif
    gGyroFilterConfigSnapshotValid = true;
#endif
}

bool sitlLocalGyroFilterConfigChanged(void)
{
#ifdef SITL_LOCAL
    if (!gGyroFilterConfigSnapshotValid) {
        return true;
    }
    if (memcmp(&gGyroFilterConfigSnapshot, gyroConfig(), sizeof(gyroConfig_t)) != 0) {
        return true;
    }
    if (gGyroFilterDshotConfigSnapshot != motorConfig()->dev.useDshotTelemetry) {
        return true;
    }
#ifdef USE_DYN_NOTCH_FILTER
    if (memcmp(&gDynNotchConfigSnapshot, dynNotchConfig(), sizeof(dynNotchConfig_t)) != 0) {
        return true;
    }
#endif
#ifdef USE_RPM_FILTER
    if (memcmp(&gRpmFilterConfigSnapshot, rpmFilterConfig(), sizeof(rpmFilterConfig_t)) != 0) {
        return true;
    }
#endif
    return false;
#else
    return true;
#endif
}

// The D-term filter chain (pidInitFilters) and the RC/rate processing
// (initRcProcessing) have the same problem as the gyro filter chain: they are
// stateful, and re-deriving them under a live flight loop changes how the
// aircraft responds to the sticks. Snapshot their inputs as well, so the
// configurator's SET handlers only rebuild them when the settings really
// changed.
static pidProfile_t gPidFilterConfigSnapshot;
// Snapshot of the PID *config* as well (pid_process_denom drives the loop dt, so
// it decides the D-term filter coefficients).
static pidConfig_t gPidFilterConfigSnapshotPidCfg;
static bool gPidFilterConfigSnapshotValid = false;

void sitlLocalSnapshotPidFilterConfig(void)
{
#ifdef SITL_LOCAL
    gPidFilterConfigSnapshot = *currentPidProfile;
    gPidFilterConfigSnapshotPidCfg = *pidConfig();
    gPidFilterConfigSnapshotValid = true;
#endif
}

bool sitlLocalPidFilterConfigChanged(void)
{
#ifdef SITL_LOCAL
    if (!gPidFilterConfigSnapshotValid) {
        return true;
    }
    return memcmp(&gPidFilterConfigSnapshot, currentPidProfile, sizeof(pidProfile_t)) != 0
        || memcmp(&gPidFilterConfigSnapshotPidCfg, pidConfig(), sizeof(pidConfig_t)) != 0;
#else
    return true;
#endif
}

static bool pidFilterConfigChanged(void)
{
#ifdef SITL_LOCAL
    return sitlLocalPidFilterConfigChanged();
#else
    return true;
#endif
}

static controlRateConfig_t gRcProcRateSnapshot;
static rcControlsConfig_t gRcProcControlsSnapshot;
static rxConfig_t gRcProcRxSnapshot;
static pidProfile_t gRcProcPidSnapshot;
static bool gRcProcSnapshotValid = false;

void sitlLocalSnapshotRcProcessingConfig(void)
{
#ifdef SITL_LOCAL
    gRcProcRateSnapshot = *currentControlRateProfile;
    gRcProcControlsSnapshot = *rcControlsConfig();
    gRcProcRxSnapshot = *rxConfig();
    gRcProcPidSnapshot = *currentPidProfile;
    gRcProcSnapshotValid = true;
#endif
}

static bool rcProcessingConfigChanged(void)
{
#ifdef SITL_LOCAL
    if (!gRcProcSnapshotValid) {
        return true;
    }
    return memcmp(&gRcProcRateSnapshot, currentControlRateProfile,
                  sizeof(controlRateConfig_t)) != 0
        || memcmp(&gRcProcControlsSnapshot, rcControlsConfig(),
                  sizeof(rcControlsConfig_t)) != 0
        || memcmp(&gRcProcRxSnapshot, rxConfig(), sizeof(rxConfig_t)) != 0
        || memcmp(&gRcProcPidSnapshot, currentPidProfile,
                  sizeof(pidProfile_t)) != 0;
#else
    return true;
#endif
}

void sitlLocalRunBootReapply(bool reinitGyroFilters)
{
#ifdef SITL_LOCAL
    // Re-pin the virtual PWM motor backend first: the EEPROM may hold DSHOT600
    // (the default once USE_DSHOT is compiled in) and its device init is a
    // false-returning stub in this build, which would leave the craft without
    // motor output.
    extern void sitlLocalPreMotorInit(void);
    sitlLocalPreMotorInit();
    if (reinitGyroFilters) {
        sitlAuditLog("reload: gyro filter configuration changed, filters re-inited");
        gyroInitFilters();
    }
    // Gains can be refreshed cheaply; the D-term filter *chain* is stateful, so it
    // is only rebuilt when the profile (or the loop dt) really changed - the same
    // rule the configurator's SET handlers use. Rebuilding it here on every reload
    // is what changed the stick-transient response (and, in a real flight, what
    // tipped the rate loop into the 120 Hz limit cycle called "the shake").
    pidInitConfig(currentPidProfile);
    // Mixer / motor / servo config: re-applies the mixer mode (motor count,
    // fixed-wing surfaces) from the saved setting. motorDevInit leaves the
    // device disabled, so re-enable it like initPhase3 does.
    mixerInit(mixerConfig()->mixerMode);
    motorDevInit(getMotorCount());
#ifdef USE_SERVOS
    servosInit();
    if (isMixerUsingServos()) {
        servoDevInit(&servoConfig()->dev);
    }
    servosFilterInit();
#endif
    motorPostInit();
    motorEnable();
    sitlLocalSyncDebugMode();

    // Everything above is now built from the current settings: remember them,
    // so a later configurator write with unchanged settings does not rebuild the
    // stateful chains for nothing.
    extern void sitlLocalSnapshotGyroFilterConfig(void);
    sitlLocalSnapshotGyroFilterConfig();
    sitlLocalSnapshotRcProcessingConfig();
#endif
}

// The rest of fc/init.c's initPhase3 that the reload path does not cover.
// Without it an in-process reboot is *not* the same state as a fresh boot: the
// gyro/dshot looptime derivation, the board alignment, IMU and failsafe init,
// and above all the profile-derived mixer runtime that mixerInitProfile()
// builds (dynamic idle gains and the DShot minimum-output override, VBAT sag
// compensation, RPM limiter gains, ez-landing thresholds) would keep whatever
// the previous boot left behind - a configured aircraft then flies with a
// different motor output range / idle behaviour than a freshly started one.
// Must run after sitlLocalRunBootReapply() (mixerInit + pidInit) and after the
// EEPROM was re-read, mirroring initPhase3's order.
void sitlLocalRunBootProfileInit(void)
{
#ifdef SITL_LOCAL
    // initPhase3 derives the looptime from the (validated) pid_process_denom
    // and hands it to the DShot telemetry backend.
    gyroSetTargetLooptime(pidConfig()->pid_process_denom);
    validateAndFixGyroConfig();
    gyroSetTargetLooptime(pidConfig()->pid_process_denom);
    initDshotTelemetry(gyro.targetLooptime);

    initBoardAlignment(boardAlignment());
    imuInit();
    failsafeInit();

    // initPhase3 runs pidInit() here so the PID dt matches the looptime derived
    // above. Refresh the gains (cheap, stateless) always, but only rebuild the
    // stateful D-term filter chain when the profile or the loop dt really changed -
    // rebuilding it on every reload disturbed the running control loop (see
    // sitlLocalRunBootReapply()).
    pidInitConfig(currentPidProfile);
    if (sitlLocalPidFilterConfigChanged()) {
        sitlAuditLog("reload: PID profile/dt changed, D-term filters re-inited");
        pidInitFilters(currentPidProfile);
    }
    sitlLocalSnapshotPidFilterConfig();
    mixerInitProfile();

    positionInit();
    autopilotInit();
#endif
}

// msp.c's writeEEPROM() calls are renamed to this in LOCAL mode so a save
// attempt is visible in the audit log (including whether the FC was armed,
// which makes MSP_EEPROM_WRITE get rejected before writeEEPROM is reached).
void sitlMspWriteEEPROM(void)
{
    sitlAuditLog("MSP writeEEPROM reached (armed=%u)", (unsigned)(ARMING_FLAG(ARMED) != 0));
    writeEEPROM();
    sitlLocalSyncDebugMode();
#ifdef SITL_LOCAL
    // A configurator Save writes settings over MSP without rebooting; log what
    // it changed (full control-relevant fingerprint) and queue the re-pin.
    extern void sitlLocalLogStateIfChanged(const char *tag);
    extern void sitlLocalRequestRepinOverrides(void);
    sitlLocalLogStateIfChanged("save");
    sitlLocalRequestRepinOverrides();
#endif
}

#ifdef SITL_LOCAL
// msp.c's MSP_EEPROM_WRITE handler is writeReadEeprom():
//
//     writeEEPROM();
//     readEEPROM();
//
// The re-read is there so the RAM copy matches what was actually stored and so
// the validation/activation path runs again. In this build the virtual EEPROM is
// a byte copy of the RAM image that was just written, so no config value can
// change; what the re-read *does* change is runtime state: readEEPROM() ends in
// activateConfig(), which re-initialises the stick-transient chain
// (initRcProcessing() -> feedforward / setpoint smoothing, pidInit(),
// rcControlsInit(), failsafeReset(), accInitFilters(), ...) while the flight loop
// keeps running. Measured with tools/sitl_local_save_compare: after a plain Save
// the same roll-stick input produced up to 153 us - about 15% of the 1000..2000
// output range - of different motor output, on a bit-identical steady state, and
// the difference was stable (the craft keeps responding differently until the
// process is restarted). A real FC only re-runs that at boot, with the motors off
// and the gyro stream restarting.
//
// So the Save path persists and keeps flying with the config it already has.
// PID/rate writes apply live through their own MSP handlers (pidInitConfig), and
// boot-time settings (filters, mixer, features, ...) apply on "Save and Reboot"
// or through sitl_local_reload_config(), exactly like real hardware.
void sitlMspReadEEPROM(void)
{
#ifdef SITL_SAVE_REREAD
    // A/B variant: behave like the stock firmware again - the save re-reads the
    // EEPROM and re-runs activateConfig() under the live flight loop.
    sitlAuditLog("save: EEPROM re-read (A/B variant)");
    readEEPROM();
#else
    sitlAuditLog("save: EEPROM re-read skipped (runtime state preserved)");
#endif
}

// msp.c's configurator SET handlers re-initialise stateful chains that a real FC
// only ever rebuilds at boot:
//
//   MSP_SET_RC_TUNING     -> initRcProcessing()
//   MSP_SET_FILTER_CONFIG -> validateAndFixGyroConfig(); gyroInitFilters();
//                            pidInitFilters(currentPidProfile);
//
// This build renames those three calls for msp.c and gates them on "the
// underlying settings really changed". Measured with
// tools/sitl_local_save_compare, replaying the configurator's read-modify-write
// burst byte for byte with the config verified unchanged:
//
//   MSP_SET_RC_TUNING     -> up to 153 us different motor output
//   MSP_SET_FILTER_CONFIG -> up to 318 us
//
// while the steady state stayed bit-identical, and the difference was stable -
// i.e. after a Save the aircraft kept responding differently to the same sticks
// until the process was restarted. Rebuilding only on a real change keeps a
// genuine filter or rate edit applying immediately (exactly like a real FC),
// while a save that changes nothing no longer touches the running control loop.
void sitlMspInitRcProcessing(void)
{
    if (rcProcessingConfigChanged()) {
        sitlAuditLog("config write: rate/RC settings changed, rc processing re-inited");
        initRcProcessing();
        sitlLocalSnapshotRcProcessingConfig();
    } else {
        sitlAuditLog("config write: rate/RC settings unchanged, rc processing kept");
    }
}

void sitlMspGyroInitFilters(void)
{
    extern void sitlLocalSnapshotGyroFilterConfig(void);
    extern bool sitlLocalGyroFilterConfigChanged(void);

    if (sitlLocalGyroFilterConfigChanged()) {
        sitlAuditLog("config write: gyro filter settings changed, filters re-inited");
        gyroInitFilters();
        sitlLocalSnapshotGyroFilterConfig();
    } else {
        sitlAuditLog("config write: gyro filter settings unchanged, chain kept");
    }
}

void sitlMspPidInitFilters(const pidProfile_t *pidProfile)
{
    if (pidFilterConfigChanged()) {
        sitlAuditLog("config write: dterm filter settings changed, filters re-inited");
        pidInitFilters(pidProfile);
        sitlLocalSnapshotPidFilterConfig();
    } else {
        sitlAuditLog("config write: dterm filter settings unchanged, chain kept");
    }
}
#endif

#ifdef SITL_LOCAL
// Simulated motor RPM bridge. dshot.c's getDshotRpm/getDshotRpmAverage/
// getDshotErpm/getMotorFrequencyHz/getMinMotorFrequencyHz are renamed to the
// sitl*Real symbols below; these wrappers return the simulator-provided RPM
// (sitl_local_input_t.motor_rpm / the UDP extended tail) whenever it is
// nonzero, and fall back to the real (always-zero in SITL) telemetry state.
#include "pg/motor.h"
#include "drivers/dshot.h"

#define SITL_ERPM_PER_LSB 100.0f
#define SITL_SIM_MOTOR_COUNT 4

extern float sitlDshotRpmReal(uint8_t motorIndex);
extern float sitlDshotRpmAverageReal(void);
extern uint16_t sitlDshotErpmReal(uint8_t motorIndex);
extern float sitlMotorFrequencyHzReal(uint8_t motorIndex);
extern float sitlMinMotorFrequencyHzReal(void);

static float sitlSimMotorHz(uint8_t motorIndex)
{
    return simTelemetryMotorFrequencyHz(motorIndex);
}

// The firmware's getMotorFrequencyHz() does not return the raw RPM: dshot.c's
// dshotTelemetryProcess() lowpass-filters it with rpm_filter_lpf_hz (default
// 150 Hz, dt = gyro.targetLooptime) before the RPM filter, the OSD and the
// dynamic idle use it. The bridge used to hand the RPM filter the raw value, so
// its notch frequencies tracked every PID-driven RPM change instead of the
// smoothed estimate a real FC works from. Rebuild the same filter here (lazily,
// because gyro.targetLooptime is only known after the boot's initPhase3).
static pt1Filter_t gSimMotorFreqLpf[SITL_SIM_MOTOR_COUNT];
static uint32_t gSimMotorFreqLpfLooptimeUs = 0;
static bool gSimMotorFreqLpfReady = false;

static void sitlSimMotorFreqLpfInit(void)
{
    const uint32_t looptimeUs = gyro.targetLooptime ? gyro.targetLooptime : 1000;
    if (gSimMotorFreqLpfReady && gSimMotorFreqLpfLooptimeUs == looptimeUs) {
        return;
    }
    const float gain = pt1FilterGain(rpmFilterConfig()->rpm_filter_lpf_hz, looptimeUs * 1e-6f);
    for (int i = 0; i < SITL_SIM_MOTOR_COUNT; i++) {
        pt1FilterInit(&gSimMotorFreqLpf[i], gain);
    }
    gSimMotorFreqLpfLooptimeUs = looptimeUs;
    gSimMotorFreqLpfReady = true;
}

float getDshotRpm(uint8_t motorIndex)
{
    const float hz = sitlSimMotorHz(motorIndex);
    return hz > 0.0f ? hz * 60.0f : sitlDshotRpmReal(motorIndex);
}

float getDshotRpmAverage(void)
{
    float sumHz = 0.0f;
    int count = 0;
    for (int i = 0; i < SITL_SIM_MOTOR_COUNT; i++) {
        const float hz = sitlSimMotorHz((uint8_t)i);
        if (hz > 0.0f) {
            sumHz += hz;
            count++;
        }
    }
    return count > 0 ? (sumHz / (float)count) * 60.0f : sitlDshotRpmAverageReal();
}

uint16_t getDshotErpm(uint8_t motorIndex)
{
    const float hz = sitlSimMotorHz(motorIndex);
    if (hz > 0.0f) {
        // eRPM = mechanical RPM * pole pairs; raw dshot LSB = eRPM / 100
        const float polePairs = (float)motorConfig()->motorPoleCount / 2.0f;
        return (uint16_t)(hz * 60.0f * polePairs / SITL_ERPM_PER_LSB);
    }
    return sitlDshotErpmReal(motorIndex);
}

// ESC telemetry bridge for the parts that are *not* read through a getter:
// MSP_MOTOR_TELEMETRY (configurator Motors tab), MSP_ESC_SENSOR_DATA (DJI FPV)
// and getDshotSensorData() (OSD ESC alarms) read dshotTelemetryState directly.
// The DSHOT decoder never runs in SITL - the virtual PWM motor backend does -
// so nothing would ever fill that structure. sitl_local_step() calls this once
// per step (the temperature follows the host's thermal model at 1 kHz).
//
// The telemetry type bits matter as much as the values:
//   TEMPERATURE -> the ACK'd DSHOT values are reported at all,
//   eRPM        -> isDshotMotorTelemetryActive(), which getDshotSensorData()
//                  requires before it hands the data to the OSD warnings.
void sitlLocalApplyDshotTelemetry(void)
{
    unsigned motorCount = getMotorCount();
    if (motorCount > MAX_SUPPORTED_MOTORS) {
        motorCount = MAX_SUPPORTED_MOTORS;
    }
    for (unsigned i = 0; i < motorCount; i++) {
        dshotTelemetryMotorState_t *state = &dshotTelemetryState.motorState[i];
        const uint8_t temperature = simTelemetryEscTemperatureCelsius((uint8_t)i);

        state->telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE] = temperature;
        state->telemetryTypes |= (1 << DSHOT_TELEMETRY_TYPE_TEMPERATURE);
        if (temperature > state->maxTemp) {
            state->maxTemp = temperature;
        }

        state->telemetryData[DSHOT_TELEMETRY_TYPE_eRPM] = getDshotErpm((uint8_t)i);
        state->telemetryTypes |= (1 << DSHOT_TELEMETRY_TYPE_eRPM);
    }
}

float getMotorFrequencyHz(uint8_t motorIndex)
{
    const float hz = sitlSimMotorHz(motorIndex);
    const float raw = hz > 0.0f ? hz : sitlMotorFrequencyHzReal(motorIndex);
    if (motorIndex >= SITL_SIM_MOTOR_COUNT) {
        return raw;
    }
    sitlSimMotorFreqLpfInit();
    return pt1FilterApply(&gSimMotorFreqLpf[motorIndex], raw);
}

float getMinMotorFrequencyHz(void)
{
    float minHz = 0.0f;
    int count = 0;
    for (int i = 0; i < SITL_SIM_MOTOR_COUNT; i++) {
        const float hz = sitlSimMotorHz((uint8_t)i);
        if (hz > 0.0f) {
            minHz = (count++ == 0) ? hz : (hz < minHz ? hz : minHz);
        }
    }
    return count > 0 ? minHz : sitlMinMotorFrequencyHzReal();
}

bool isDshotTelemetryActive(void)
{
    // The LOCAL link always bridges host-provided motor RPM, so DSHOT
    // telemetry is active from the FC's perspective. The real implementation
    // requires decoded per-motor telemetry frames (telemetryTypes eRPM bit),
    // which the virtual PWM path never produces; without this override the
    // FC blocks arming with ARMING_DISABLED_DSHOT_TELEM (core.c).
    return true;
}

// motor.c's motorShutdown() is called by every reboot path (CLI exit, MSP
// reboot, CMS/mavlink reboot) to stop the ESC outputs before the MCU resets.
// LOCAL mode never actually resets the FC - systemReset() defers the EEPROM
// persist and the FC keeps running - so the real shutdown would set
// motorDevice.initialized = false and motorEnable() would never re-enable the
// outputs after the next arming (the FC shows armed, but PWM stops forever).
// A no-op keeps the virtual motor output alive across configurator reboots.
void motorShutdown(void)
{
}

// The virtual blackbox writes LOG*.BFL and scans for the next log number in
// the process working directory. Inside a DLL that is the host engine's CWD,
// which is unpredictable, so redirect both to a stable folder: the default is
// the same %LOCALAPPDATA%\Betaflight-SITL as the virtual EEPROM, and the host
// can override it per aircraft at runtime with sitl_local_set_blackbox_dir().
static char gBlackboxDir[MAX_PATH] = "";

static const char *sitlBlackboxDir(char *buf, size_t size)
{
    if (gBlackboxDir[0] != '\0') {
        if (strlen(gBlackboxDir) + 1 > size) {
            return NULL;
        }
        memcpy(buf, gBlackboxDir, strlen(gBlackboxDir) + 1);
        CreateDirectoryA(buf, NULL);
        return buf;
    }
    if (GetEnvironmentVariableA("LOCALAPPDATA", buf, (DWORD)size) > 0) {
        _snprintf(buf + strlen(buf), size - strlen(buf), "\\Betaflight-SITL");
        CreateDirectoryA(buf, NULL);
        return buf;
    }
    return NULL;
}

FILE *sitlBlackboxFopen(const char *filename, const char *mode)
{
    char dir[MAX_PATH];
    if (sitlBlackboxDir(dir, sizeof(dir)) != NULL) {
        char path[MAX_PATH];
        _snprintf(path, sizeof(path), "%s\\%s", dir, filename);
        return fopen(path, mode);
    }
    return fopen(filename, mode);
}

DIR *sitlBlackboxOpendir(const char *path)
{
    (void)path;
    char dir[MAX_PATH];
    if (sitlBlackboxDir(dir, sizeof(dir)) != NULL) {
        return opendir(dir);
    }
    return opendir(".");
}

// blackbox_virtual.c's blackboxVirtualOpen() is renamed to
// sitlBlackboxVirtualOpenReal() in LOCAL builds; this forwarding version keeps
// the boot-time behavior (scan the current blackbox directory for the largest
// log number) and lets sitl_local_set_blackbox_dir() re-run the scan after a
// directory change so per-folder numbering never overwrites existing logs.
extern bool sitlBlackboxVirtualOpenReal(void);

bool blackboxVirtualOpen(void)
{
    return sitlBlackboxVirtualOpenReal();
}

int sitl_local_set_blackbox_dir(const char *path)
{
    if (path == NULL || path[0] == '\0' || strlen(path) >= MAX_PATH) {
        return -1;
    }
    strncpy(gBlackboxDir, path, sizeof(gBlackboxDir) - 1);
    gBlackboxDir[sizeof(gBlackboxDir) - 1] = '\0';
    CreateDirectoryA(gBlackboxDir, NULL);
    blackboxVirtualOpen(); // re-scan for correct numbering in the new folder
    return 0;
}

// sitl.c's systemResetToBootloader() calls exit(0), which would terminate the
// host process from a DLL. "Enter bootloader / DFU" has no meaning for the
// in-process FC, so treat it like any other firmware reboot: persist, reset the
// runtime state, re-read the config, and jump back to the MSP thread loop
// (mspRebootFn's bootloader branch spins in a `while (true);` after this
// returns, exactly like the firmware case).
void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);
    writeEEPROM();
    sitlLocalRequestReboot();
    sitlLocalRebootJump();
}

#endif // SITL_LOCAL

// sitl.c's fopen() calls are renamed to sitlFopen() by CMakeLists.txt so the
// virtual EEPROM file can be redirected without touching the Betaflight
// submodule. Set BF_SITL_EEPROM to a file path to keep separate configs
// (e.g. "E:\sim\unreal.bin"); the default stays eeprom.bin in the working
// directory.
#define SITL_EEPROM_FILENAME "eeprom.bin"

// sitl.c's virtual EEPROM handle. sitl.c assigns eepromFd from the fopens
// below and only ever clears it in configLock(), so mirroring it here tells
// the LOCAL link whether the file is currently open. loadEEPROMFromFile()
// refuses to reopen a file while a handle is open, so switching to another
// EEPROM path requires closing this one first.
static FILE *gSitlEepromFd = NULL;

FILE *sitlFopen(const char *filename, const char *mode)
{
    if (strcmp(filename, SITL_EEPROM_FILENAME) == 0) {
        const char *eeprom = getenv("BF_SITL_EEPROM");
        FILE *fp = (eeprom != NULL && eeprom[0] != '\0')
            ? fopen(eeprom, mode)
            : fopen(filename, mode);
        gSitlEepromFd = fp;
        return fp;
    }
    return fopen(filename, mode);
}

int sitlFclose(FILE *fp)
{
    if (fp == gSitlEepromFd) {
        gSitlEepromFd = NULL;
    }
    return fclose(fp);
}

bool sitlEepromFileIsOpen(void)
{
    return gSitlEepromFd != NULL;
}

void dyad_update(void)
{
    // The dyad TCP bridge is replaced by serial_tcp_win.c on Windows, so the
    // SITL tcpWorker thread only spins here. With no streams, select() returns
    // immediately on Windows, so dyad_update() would burn a full CPU core.
    // Sleep instead; 10 ms matches the update timeout the SITL would use.
    Sleep(10);
}

#ifdef SITL_UDP_TIME
// Unreal UDP-driven virtual clock. The clock starts at 0, so the scheduler
// anchors its gyro deadline grid at 0 (schedulerInit reads getCycleCounter
// during init) and the run loop fast-forwards to the first deadline in fixed
// 100 us quanta. After that the clock only advances by FDM packet timestamp
// deltas: gyro/PID run once per 1000 us of Unreal time, and with no packets
// the clock freezes so the flight loop idles while serial/MSP stays alive.
static uint64_t sitlVirtualTimeUs = 0;

void sitlStepTime(uint64_t stepUs)
{
    sitlVirtualTimeUs += stepUs;
}
#endif

// msp_serial.c's millis() calls are renamed to sitlMspMillis() by
// CMakeLists.txt so the CLI entry guard and configurator-activity timeout
// use real time; they must work while the UDP-driven virtual clock is frozen
// during idle (no packets arriving).
uint32_t sitlMspMillis(void)
{
    return (uint32_t)(micros64_real() / 1000);
}

// sitl.c's pthread_mutex_trylock/unlock calls are renamed to these stubs by
// CMakeLists.txt. The virtual-EEPROM motor-output path has a broken mutex: a
// trylock that is never unlocked on the main thread plus a mis-owned unlock
// from the FDM receive thread (undefined behaviour), which can silently skip
// motor packets. The stubs make that path lock-free, matching the upstream
// fix of simply removing the gate.
int sitlMutexTrylock(pthread_mutex_t *mutex)
{
    (void)mutex;
    return 0;
}

int sitlMutexUnlock(pthread_mutex_t *mutex)
{
    (void)mutex;
#ifdef SITL_UDP_TIME
    // updateState() ends with pthread_mutex_unlock(&updateLock), renamed to
    // this stub by CMakeLists.txt. Commit the FDM packet's virtual time only
    // now, i.e. after the virtual sensors for that packet have been written
    // (see sitlUdpFdmCommitPending() in udplink_windows.c). In REALTIME mode
    // there is no virtual clock, so this stays a no-op.
    extern void sitlUdpFdmCommitPending(void);
    sitlUdpFdmCommitPending();
#endif
    return 0;
}

// msp.c's systemReset() calls are renamed to sitlSystemReset() by
// CMakeLists.txt. A firmware reboot terminates the SITL process, so persist
// the current RAM config first - this makes the configurator's "Save and
// Reboot" always save, even if the MSP_EEPROM_WRITE step was skipped or lost.
// systemReset() itself is our custom implementation below, which relaunches
// the simulator so the reboot does not need a manual restart.
void sitlSystemReset(void)
{
    writeEEPROM();
#ifdef SITL_LOCAL
    // msp.c's mspRebootFn (MSP_SET_REBOOT post-processing) spins in a
    // `while (true);` loop after systemReset() returns, because a real reboot
    // never returns. LOCAL mode's systemReset() defers the persist and returns,
    // so the stock function would hang the background MSP thread forever and
    // every later configurator connection would time out (the configurator
    // sends MSP_SET_REBOOT right after a CLI "exit"). Jump back to the MSP
    // thread loop instead; the parser is already back in PORT_IDLE when the
    // reboot handler runs, so the next MSP request is processed normally.
    sitlLocalRequestReboot();
    sitlLocalRebootJump();
#else
    systemReset();
#endif
}

#ifndef SITL_LOCAL
// Spawn a hidden copy of ourselves before the firmware reboot exits this
// process. The child sees BF_SITL_REBOOT_CHILD=1 in its environment and
// waits a couple of seconds (see main_windows.c) for the parent to release
// the TCP/UDP ports, then comes up as the "reborn" flight controller. An
// environment variable is used instead of a command-line flag so the
// firmware's targetParseArgs() does not reject it as an unknown argument.
static void sitlRelaunchSelf(void)
{
    const char *cmdline = GetCommandLineA();
    if (cmdline == NULL) {
        return;
    }

    const size_t len = strlen(cmdline);
    char *childCmdline = (char *)malloc(len + 1);
    if (childCmdline == NULL) {
        return;
    }
    memcpy(childCmdline, cmdline, len + 1);

    STARTUPINFOA si;
    PROCESS_INFORMATION pi;
    memset(&si, 0, sizeof(si));
    memset(&pi, 0, sizeof(pi));
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES;
    si.wShowWindow = SW_HIDE;
    si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
    si.hStdError = GetStdHandle(STD_ERROR_HANDLE);

    SetEnvironmentVariableA("BF_SITL_REBOOT_CHILD", "1");
    if (CreateProcessA(NULL, childCmdline, NULL, NULL, TRUE, CREATE_NO_WINDOW,
                       NULL, NULL, &si, &pi)) {
        CloseHandle(pi.hThread);
        CloseHandle(pi.hProcess);
        fprintf(stderr, "[SITL] auto-restart: spawned child PID %lu\n",
                (unsigned long)pi.dwProcessId);
    } else {
        fprintf(stderr, "[SITL] auto-restart failed (error %lu), manual restart required\n",
                (unsigned long)GetLastError());
    }
    SetEnvironmentVariableA("BF_SITL_REBOOT_CHILD", NULL);
    free(childCmdline);
}
#endif

// Custom firmware reboot installed for every systemReset() caller (CLI save /
// exit, MSP reboot, CMS, ...). sitl.c's original implementation is compiled
// as sitlSystemResetNative() so the symbol is free for this wrapper.
void systemReset(void)
{
#ifdef SITL_LOCAL
    // In-process library mode: there is no standalone process to relaunch and
    // exiting would kill the host engine. Defer the (potentially slow)
    // EEPROM persist to the background thread so systemReset() never blocks
    // the UE thread that may be executing it via the scheduler's TASK_SERIAL
    // (the MSP caller already persisted via sitlSystemReset when applicable;
    // the reboot re-persists as its own first step anyway).
    extern void sitlLocalRequestReset(void);
    sitlLocalRequestReset();
    sitlLocalRequestReboot();
#else
#ifdef USE_BLACKBOX
    // Close any in-progress blackbox log cleanly (writes the end-of-log event)
    // before relaunching, so a reboot never leaves a truncated .BFL file.
    blackboxFinish();
#endif
    sitlRelaunchSelf();
    sitlSystemResetNative();
#endif
}

// Battery voltage/current and motor RPM fed from the extended FDM packet
// (see sim_telemetry.h). battery.c's ADC meter calls are renamed to these by
// CMakeLists.txt so the virtual FC reports what the simulator sends.
void sitlBatteryVoltageRefresh(void)
{
    // No-op: the value comes from the UDP telemetry feed.
}

void sitlBatteryVoltageRead(voltageSensorADC_e adcChannel, voltageMeter_t *voltageMeter)
{
    UNUSED(adcChannel);
    const uint16_t v = simTelemetryVoltageCentiVolts();
    voltageMeter->displayFiltered = v;
    voltageMeter->unfiltered = v;
#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    voltageMeter->sagFiltered = v;
#endif
}

void sitlBatteryCurrentRefresh(int32_t lastUpdateAt)
{
    simTelemetryCurrentRefresh(lastUpdateAt);
}

void sitlBatteryCurrentRead(currentMeter_t *meter)
{
    const int32_t centiAmps = (int32_t)(simTelemetryCurrentAmps() * 100.0f);
    meter->amperage = centiAmps;
    meter->amperageLatest = centiAmps;
    meter->mAhDrawn = (int32_t)simTelemetryMahDrawn();
}

// Official real-time time base. sitl.c's time functions are renamed to
// sitl* by CMakeLists.txt; these wrappers keep the original symbols available
// to the rest of the firmware.
uint64_t micros64(void)
{
#ifdef SITL_UDP_TIME
    return sitlVirtualTimeUs;
#else
    // Use the raw monotonic clock directly. The official sitlMicros64() is
    // scaled by the external simulator's simRate, which is left at a stale
    // value after an Unreal session ends and would otherwise make the
    // "official" 1 kHz lock drift from wall time.
    return micros64_real();
#endif
}

uint32_t micros(void)
{
    return (uint32_t)(micros64() & 0xFFFFFFFF);
}

uint64_t millis64(void)
{
    return micros64() / 1000;
}

uint32_t millis(void)
{
    return (uint32_t)((micros64() / 1000) & 0xFFFFFFFF);
}

uint32_t getCycleCounter(void)
{
    return (uint32_t)(micros64() & 0xFFFFFFFF);
}

void delayMicroseconds(uint32_t us)
{
#ifdef SITL_UDP_TIME
    // In UDP mode the sim clock is owned by the incoming packet stream;
    // firmware delayMicroseconds() calls must not advance it or sleep the
    // scheduler thread.
    UNUSED(us);
#else
    sitlDelayMicroseconds(us);
#endif
}
#endif

struct tm *gmtime_r(const time_t *timep, struct tm *result)
{
    if (gmtime_s(result, timep) == 0) {
        return result;
    }
    return NULL;
}

char *strsep(char **stringp, const char *delim)
{
    char *start = *stringp;
    char *p;

    if (start == NULL) {
        return NULL;
    }

    p = start + strcspn(start, delim);
    if (*p) {
        *p = '\0';
        *stringp = p + 1;
    } else {
        *stringp = NULL;
    }

    return start;
}

/* No-op audio stubs (pidaudio task is compiled in for the SITL source list). */
void audioGenerateWhiteNoise(void)
{
}

void audioSilence(void)
{
}

void audioPlayTone(uint8_t tone)
{
    (void)tone;
}

void audioSetupIO(void)
{
}

float clockCyclesToMicrosf(int32_t clockCycles)
{
    return (float)clockCycles;
}

bool IORead(IO_t io)
{
    (void)io;
    return false;
}

void IOWrite(IO_t io, bool value)
{
    (void)io;
    (void)value;
}

/* The official build lets LTO drop these PG storage symbols on SITL; without
 * LTO on MinGW we provide zeroed storage so serial config code can link. */
serialPinConfig_t serialPinConfig_System;
serialPinConfig_t serialPinConfig_Copy;
uint32_t serialPinConfig_fnv_hash;
