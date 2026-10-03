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
#include "scheduler/scheduler.h"
#include "fc/tasks.h"
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
    float motors[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    sitlLocalStepStats(&stepMaxUs, &stepAvgUs, &stepCount, motors, &zeroArmed);
    sitlLocalMotorStreamStats(&capPkts, &takePkts, &missPkts);
    sitlLocalMspLoad(&mspBusyUs, &mspCalls);

    sitlAuditLog("%s rt protocol=%u dshot(cfg/edt/bb)=%u/%u/%u idle=%u thr=%u/%u poles=%u "
                 "battSrc=%u/%u rcOurs=%u mixer(cfg/rt/count)=%u/%u/%u denom=%u looptime=%u pidDt=%u "
                 "stepUs(max/avg/count)=%u/%u/%u motorPkts(cap/take/miss)=%u/%u/%u "
                 "motors=%.0f/%.0f/%.0f/%.0f zeroWhileArmed=%u "
                 "mspThread(busyUs/calls)=%u/%u "
                 "featRt(air/ag/udp/gps/3d/esc)=%u/%u/%u/%u/%u/%u "
                 "lpf=%u/%u-%u/%u notch=%u/%u/%u/%u pidProf=%u ratesType=%u armed=%u armFlags=%08X "
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
                 (unsigned)now.rtAirMode, (unsigned)now.rtAntiGravity, (unsigned)now.rtRxUdp,
                 (unsigned)now.rtGps, (unsigned)now.rt3d, (unsigned)now.rtEscSensor,
                 (unsigned)now.lpf1, (unsigned)now.lpf1DynMin, (unsigned)now.lpf1DynMax,
                 (unsigned)now.lpf2, (unsigned)now.notch1Hz, (unsigned)now.notch1Cut,
                 (unsigned)now.notch2Hz, (unsigned)now.notch2Cut, (unsigned)now.pidProfileIdx,
                 (unsigned)now.ratesType, (unsigned)now.armed, now.armDisableFlags,
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
    pidInit(currentPidProfile);
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

    // pidInit() already ran in sitlLocalRunBootReapply(); redo it here so the
    // PID dt matches the looptime derived above, exactly like initPhase3.
    pidInit(currentPidProfile);
    mixerInitProfile();

    positionInit();
    autopilotInit();
#endif
}

// A configurator Save perturbs the scheduler's own state (its deadline grid
// anchor, task queue, task ages and periods are all built by
// tasksInit()/schedulerInit() and only ever at boot; the save path runs
// schedulerIgnoreTaskStateTime() and re-runs various init routines). The tasks
// then stop lining up with the host's fixed stepping grid and the craft
// trembles until a *new process* re-runs tasksInit(). Rebuild it in-process,
// between steps, and re-pin the three realtime periods to the LOCAL loop time.
void sitlLocalRunSchedulerRepin(void)
{
#ifdef SITL_LOCAL
    tasksInit();
    const uint32_t periodUs = (gyro.targetLooptime > 0) ? (uint32_t)gyro.targetLooptime : 1000u;
    rescheduleTask(TASK_GYRO, periodUs);
    rescheduleTask(TASK_FILTER, periodUs);
    rescheduleTask(TASK_PID, periodUs);
    sitlAuditLog("scheduler re-anchored after config write (tasksInit, period=%u us)", (unsigned)periodUs);
#else
    // Only the LOCAL link needs this: the standalone builds own the process, so
    // a "reboot" really restarts it and re-runs tasksInit() anyway.
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
    extern void sitlLocalRequestSchedulerRepin(void);
    sitlLocalLogStateIfChanged("save");
    sitlLocalRequestRepinOverrides();
    // The save path also perturbs the scheduler's own state (see
    // sitlLocalRequestSchedulerRepin()); rebuild it like a boot would.
    sitlLocalRequestSchedulerRepin();
#endif
}

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
    return hz > 0.0f ? hz : sitlMotorFrequencyHzReal(motorIndex);
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
