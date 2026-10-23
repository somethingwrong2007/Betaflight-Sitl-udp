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
#include "fc/rc_modes.h"
#include "rx/rx.h"
#include "io/beeper.h"
#include "blackbox/blackbox.h"
#include "rx/crsf.h"
#include "osd/osd.h"
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

// Cheap monotonic wall clock for the flight-loop hot path.
//
// micros64_real() goes through clock_gettime(CLOCK_MONOTONIC), which on
// Windows/MinGW costs ~1.5 us per call - and sitl_local_step() reads the clock
// twice per step (once to start the timing, once in localRecordStepStats). At a
// 4 kHz loop that is 3 us out of every 250 us step, i.e. the single largest item
// of the wrapper's own overhead. QueryPerformanceCounter is ~20 ns, so use it
// for the per-step timing and anything else that runs per frame; the audit log
// and other rare callers keep micros64_real().
uint64_t sitlWallUs(void)
{
    static LARGE_INTEGER freq;
    if (freq.QuadPart == 0) {
        QueryPerformanceFrequency(&freq);
    }
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (uint64_t)((now.QuadPart * 1000000LL) / freq.QuadPart);
}

// Append one line to %LOCALAPPDATA%\Betaflight-SITL\sitl-audit.log so save /
// connection problems in the in-process LOCAL build are traceable without
// capturing the host process stderr. BF_SITL_AUDIT_LOG redirects the file, so a
// test tool can keep its own trail instead of appending to the host's.
//
// The log is opened once and kept open (line-buffered, so every line still
// reaches the file immediately). It used to fopen/fprintf/fclose per line, and
// this runs on the flight thread for the state/flight-recorder lines: three file
// system calls per line, every 250 ms, is exactly the kind of spike that shows
// up in stepUs(max). A critical section keeps the several threads that log
// (flight, MSP, host) from interleaving a line.
void sitlAuditLog(const char *fmt, ...)
{
    static FILE *log = NULL;
    // A spin lock, not a CRITICAL_SECTION: this runs on the flight, MSP and host
    // threads, and a lazily-initialised critical section races (a second thread
    // can EnterCriticalSection() while the first is still inside
    // InitializeCriticalSection(), which is undefined behaviour and took the
    // process down). A CAS spin lock needs no initialisation at all; the log is
    // written a few times a second, so the spin is never hot.
    static volatile LONG lock = 0;
    while (InterlockedExchange(&lock, 1) != 0) {
        Sleep(0);
    }

    if (log == NULL) {
        char path[MAX_PATH];
        if (GetEnvironmentVariableA("BF_SITL_AUDIT_LOG", path, sizeof(path)) > 0
            || (GetEnvironmentVariableA("LOCALAPPDATA", path, sizeof(path)) > 0
                && strncat(path, "\\Betaflight-SITL\\sitl-audit.log",
                           sizeof(path) - strlen(path) - 1) != NULL)) {
            log = fopen(path, "a");
            if (log != NULL) {
                setvbuf(log, NULL, _IOLBF, 0);
            }
        }
    }
    if (log == NULL) {
        InterlockedExchange(&lock, 0);
        return;
    }

    fprintf(log, "[%llu] ", (unsigned long long)(micros64_real() / 1000));
    va_list ap;
    va_start(ap, fmt);
    vfprintf(log, fmt, ap);
    va_end(ap);
    fprintf(log, "\n");
    InterlockedExchange(&lock, 0);
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

    // Drop the configurator's connections, like real hardware does when the MCU
    // resets. A link that stays open across the reboot is silent from then on
    // (nothing services MSP while the reboot is applied), and the peer never
    // learns the session is gone - which is exactly the state that leaves a
    // ground station unable to reconnect on its own.
    {
        extern void serialTcpDropClients(void);
        serialTcpDropClients();
    }

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
                 "battSrc=%u/%u batt(cells=%u v=%.2f a=%.2f mah=%u) rcOurs=%u "
                 "mixer(cfg/rt/count)=%u/%u/%u denom=%u looptime=%u pidDt=%u "
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
                 (unsigned)now.voltSrc, (unsigned)now.currSrc,
                 (unsigned)getBatteryCellCount(), (double)getBatteryVoltage() * 0.01,
                 (double)getAmperage() * 0.01, (unsigned)getMAhDrawn(),
                 (unsigned)now.rcOurs,
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

// OSD runtime state a reboot has to reset. Split out of
// sitlLocalResetVolatileControlState() because the in-process reboot handler
// (localRunPendingReboot(): the configurator's Save and Reboot, CLI
// save/exit/defaults, CMS, Enter bootloader/DFU) needs exactly this part too -
// a real FC redraws its boot splash after *every* reboot, and the HUD would
// otherwise keep the previous session's frame.
//
// Nothing here is read by the control loop (osd.c's state machine, the
// statistics the OSD/MSP report, and the consumed-charge counter), so unlike
// the control leftovers below it is safe to run between scheduler passes on a
// live loop.
void sitlLocalResetOsdRuntimeState(void)
{
#ifdef SITL_LOCAL
    // osd.c keeps these pieces of per-session state that no boot path rebuilds:
    //   * osdState (its private osdState_e, OSD_STATE_INIT == 0) - the frame
    //     state machine. The splash (BF logo + version + the menu hint) is only
    //     drawn from OSD_STATE_INIT, so a restart has to put it back there for
    //     the host to see a "just powered up" HUD again.
    //   * the flight statistics (fc/stats.c's statistic_t, exposed through
    //     osdGetStats()) - max current/speed/altitude/G, min voltage/RSSI/...,
    //     armed time. They are reset by osdResetStats() on every *arm*, so a
    //     restart would otherwise show the previous session's numbers on the
    //     post-flight stats page until the next arm.
    //   * three globals osd.h exports: resumeRefreshAt (the splash/post-flight
    //     stats suppression window), osdFlyTime and osdLaunchTime. A restart
    //     inheriting a resumeRefreshAt still in the future would hold the
    //     screen frozen for up to the 60 s stats window, and the fly-time
    //     element would carry the previous session's total.
    // The field list below is a verbatim copy of osd.c's osdResetStats(); the
    // function is file-static there and this build deliberately does not modify
    // the pinned submodule. Keep the two in sync if upstream adds a field.
    // (One static is deliberately left alone: osd.c's osdStatsEnabled /
    // osdStatsVisible are file-static show/hide flags for the stats page. They
    // cannot be reached from here without editing the submodule; the worst case
    // is that the stats page - with the fresh numbers written below - is shown
    // for its 60 s window after the reboot instead of the plain disarmed HUD,
    // and the next arm/disarm edge clears it.)
    extern uint32_t osdState;          // osd.c: osdState_e osdState = OSD_STATE_INIT
    extern statistic_t *osdGetStats(void);

    osdState = 0;                      // OSD_STATE_INIT
    resumeRefreshAt = 0;
    osdFlyTime = 0;
    osdLaunchTime = 0;
    // Consumed charge (the OSD/configurator mAh element): a fresh process
    // starts at 0, an in-process restart would carry the previous flight's
    // total. Kept here rather than in the statistics because it is integrated
    // by the virtual current meter, not by osd.c.
    {
        const float beforeMahDrawn = simTelemetryMahDrawn();
        simTelemetryResetMahDrawn();
        sitlAuditLog("osd: mAh counter reset: %.1f -> 0.0", (double)beforeMahDrawn);
    }

    statistic_t *s = osdGetStats();
    if (s != NULL) {
        // Snapshot the previous session's numbers first. Clearing them is the
        // point of this function, so the log line below is only evidence if its
        // left-hand side is non-zero - the harness arms and flies before it
        // reboots, so it is.
        const int16_t  beforeMaxCurrent  = s->max_current;
        const int16_t  beforeMaxSpeed    = s->max_speed;
        const int32_t  beforeMaxAltitude = s->max_altitude;
        const int16_t  beforeMinVoltage  = s->min_voltage;
        const uint32_t beforeArmedTimeMs = (uint32_t)(s->armed_time / 1000);
        const float    beforeMaxGForce   = s->max_g_force;

        s->max_current     = 0;
        s->max_speed       = 0;
        s->min_voltage     = 5000;
        s->end_voltage     = 0;
        s->min_rssi        = 99;       // percent
        s->max_altitude    = 0;
        s->max_distance    = 0;
        s->armed_time      = 0;
        s->max_g_force     = 0;
        s->max_esc_temp_ix = 0;
        s->max_esc_temp    = 0;
        s->max_esc_rpm     = 0;
        s->min_link_quality = (linkQualitySource == LQ_SOURCE_NONE) ? 99 : 100;
        s->min_rssi_dbm    = CRSF_RSSI_MAX;
        s->min_rsnr        = CRSF_SNR_MAX;

        // "before -> after" is the whole verification that a reboot starts a
        // fresh flight's statistics: the left-hand side is whatever the
        // previous session accumulated.
        sitlAuditLog("osd: stats reset: armedTime %u->0 ms maxCurrent %d->0 "
                     "maxSpeed %d->0 maxAlt %d->0 minVoltage %d->5000 maxG %.2f->0.00",
                     (unsigned)beforeArmedTimeMs, (int)beforeMaxCurrent,
                     (int)beforeMaxSpeed, (int)beforeMaxAltitude,
                     (int)beforeMinVoltage, (double)beforeMaxGForce);
    }
    // Logged after the statistics so this line is the last thing a reader sees
    // for the block (and the one the "did it run at all" check greps).
    sitlAuditLog("osd: runtime state reset for a fresh boot (splash + stats)");
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

    // OSD runtime state (splash + statistics + mAh): the same reset the
    // in-process reboot handler runs, so a host-side restart and a "Save and
    // Reboot" end up in the identical "just powered up" HUD.
    sitlLocalResetOsdRuntimeState();

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
    return gyro.targetLooptime ? gyro.targetLooptime : 1000;
}

// The gyro/filter/PID periods the loop should run at: the gyro task follows the
// *sample* period, the filter and PID tasks the (denominator-scaled) PID period.
uint32_t sitlLocalGyroSampleUs(void)
{
    return gyro.sampleLooptime ? gyro.sampleLooptime : 1000;
}

uint16_t sitlLocalAccSampleRateHz(void)
{
    return gyro.accSampleRateHz;
}

// Make the virtual gyro's *sample* rate match the loop rate the tasks actually
// run at, and re-derive everything that depends on it. The compiled
// VIRTUAL_GYRO_SAMPLE_RATE_HZ and SITL_GYRO_HZ are the same value, so this is a
// no-op for a plain build; it exists so a runtime BF_SITL_GYRO_HZ override cannot
// leave the gyro sampling at the compiled rate while the scheduler runs faster
// (the filter coefficients, RPM filter dt/Nyquist ceiling and the dynamic-notch
// gate all derive from gyro.sampleLooptime/targetLooptime).
//
// Only ever called at boot, before the host starts stepping the FC, so the
// chain rebuild cannot disturb a running loop.
void sitlLocalApplyGyroRate(uint32_t hz)
{
#ifdef SITL_LOCAL
    if (hz < 100 || hz > 32000) {
        return;
    }
    if (gyro.sampleRateHz == hz && gyro.sampleLooptime == (uint32_t)(1e6f / hz)) {
        return;     // already consistent
    }
    gyro.sampleRateHz = hz;
    gyro.sampleLooptime = 1e6f / hz;
    // The accelerometer lives in the same virtual device and is sampled at the
    // same rate; keep its task consistent as well.
    gyro.accSampleRateHz = hz;
    acc.sampleRateHz = hz;
    if (gyro.rawSensorDev != NULL) {
        gyro.rawSensorDev->gyroSampleRateHz = hz;
        gyro.rawSensorDev->accSampleRateHz = hz;
    }
    // The loop-time validation can raise pid_process_denom (e.g. a motor
    // protocol that cannot be updated that fast); run it before deriving the
    // looptime, exactly like initPhase3 does.
    validateAndFixGyroConfig();
    gyroSetTargetLooptime(pidConfig()->pid_process_denom);
    gyroInitFilters();
    pidInit(currentPidProfile);
#if defined(USE_DSHOT_TELEMETRY) || defined(USE_ESC_SENSOR)
    initDshotTelemetry(gyro.targetLooptime);
#endif
    sitlAuditLog("gyro rate: applied %u Hz (sampleUs=%u targetUs=%u denom=%u) "
                 "and re-derived the filter chains",
                 (unsigned)hz, (unsigned)gyro.sampleLooptime,
                 (unsigned)gyro.targetLooptime,
                 (unsigned)pidConfig()->pid_process_denom);
#else
    UNUSED(hz);
#endif
}

// The rates the gyro/loop actually run at (for the boot rate audit).
void sitlLocalGetGyroRates(uint16_t *sampleRateHz, uint32_t *sampleLooptime,
                           uint32_t *targetLooptime)
{
#ifdef SITL_LOCAL
    if (sampleRateHz)   { *sampleRateHz = gyro.sampleRateHz; }
    if (sampleLooptime) { *sampleLooptime = gyro.sampleLooptime; }
    if (targetLooptime) { *targetLooptime = gyro.targetLooptime; }
#else
    UNUSED(sampleRateHz);
    UNUSED(sampleLooptime);
    UNUSED(targetLooptime);
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

extern void sitlLocalGetRpmNotchHz(float notchHz[3]);

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

// msp_box.c's initActiveBoxIds() only adds BOXCRASHFLIP ("FLIP OVER AFTER
// CRASH") to the active box list when the configured motor protocol is DShot
// (msp_box.c: `checkMotorProtocolEnabled(...); if (configuredMotorProtocolDshot)
// BME(BOXCRASHFLIP);`). The LOCAL link has to run the virtual PWM backend, so
// without this the mode logic exists and MSP_SET_MODE_RANGE accepts the box, but
// the Configurator never lists it (it only shows the boxes MSP_BOXIDS returns).
//
// msp_box.c's symbol is renamed to sitlInitActiveBoxIdsReal() by CMakeLists.txt;
// this wrapper reports DShot for the duration of that pure config function and
// restores the real protocol before returning. Nothing else reads the value in
// between, so motorDev, the RPM bridge and the output path stay on PWM.
#ifdef SITL_LOCAL
void initActiveBoxIds(void)
{
    extern void sitlInitActiveBoxIdsReal(void);

    const uint8_t savedProtocol = motorConfig()->dev.motorProtocol;
    motorConfigMutable()->dev.motorProtocol = MOTOR_PROTOCOL_DSHOT600;
    sitlInitActiveBoxIdsReal();
    motorConfigMutable()->dev.motorProtocol = savedProtocol;
}

// Callers of isCrashFlipModeActive() (mixer.c, the OSD files, ledstrip.c) are
// renamed to this by CMakeLists.txt in the LOCAL build. The firmware's own latch
// is only set when the configured protocol is DShot, which the PWM-only LOCAL
// backend never is; sitl_local.c reproduces that latch with the stock semantics
// (arm edge + box active, cleared on disarm / switch off).
bool sitlIsCrashFlipModeActive(void)
{
    extern bool sitlLocalCrashFlipActive(void);
    return isCrashFlipModeActive() || sitlLocalCrashFlipActive();
}
#endif

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

// Substitute for isBlackboxDeviceWorking(), used by osd.c only
// (CMakeLists.txt renames the call in that file).
//
// The post-flight statistics page draws a "BLACKBOX" row through
// osdGetBlackboxStatusString(), which computes
//
//     storageUsedPercent = (storageUsed * 100) / storageTotal;
//
// with storageTotal still 0 whenever neither USE_SDCARD nor USE_FLASHFS is
// compiled - which is the case for SITL - so GCC constant-folds the division
// into a ud2 trap. The row is drawn while the aircraft is disarmed, so every
// arm/disarm killed the process with 0xC000001D inside osdUpdate(). The
// file-based virtual blackbox has no storage statistics to report, so the honest
// answer for the OSD is "no working storage device"; SERIAL keeps its real value
// (that is the one case the firmware can answer without storage counters).
bool sitlOsdIsBlackboxDeviceWorking(void)
{
    extern bool isBlackboxDeviceWorking(void);
    extern blackboxConfig_t *blackboxConfigMutable(void);
    if (blackboxConfigMutable()->device != BLACKBOX_DEVICE_SERIAL) {
        return false;
    }
    return isBlackboxDeviceWorking();
}

// ---------------------------------------------------------------------------
// Crash locator.
//
// A crash inside the firmware (executed ud2, null pointer, stack overflow) tells
// the host nothing on its own. This installs a vectored exception handler that
// logs the exception code and the faulting address together with its offset
// inside the module that contains it - the RVA that
//
//     addr2line -f -C -e betaflight_SITL.dll <rva>
//
// turns into file:line (the LOCAL DLL is built with -g for exactly this). The
// handler never swallows the exception: it logs and returns
// EXCEPTION_CONTINUE_SEARCH, so the process still dies the way Windows and a
// debugger expect.
//
// The handler deliberately does not call sitlAuditLog(): the crash may be inside
// that function with its spin lock held by this very thread, so it uses its own
// one-shot write (and stderr, which a console host shows).
//
// What it must NOT do is what it used to: a vectored handler runs for *every*
// exception, first-chance ones included, and the interesting-looking ones are
// frequently not crashes at all - 0x40010006 (OutputDebugString), 0x406D1388
// (thread naming by a debugger), 0xE06D7363 (a C++ exception being thrown and
// caught), 0x80000003 (a breakpoint). Logging those buried the log in noise.
// Worse, the handler also did CRT file I/O (fopen/fprintf/fclose) with ~600
// bytes of stack locals: when the real fault is a stack overflow - or any fault
// with the stack nearly gone - its own prologue/`_chkstk` faults, that nested
// exception re-enters the handler, and the recursive work keeps the process
// from handing a clean exception to the host's own crash reporter.
//
// So: fatal codes only, no CRT, no heap, no locks, fixed static buffers, one
// pre-opened handle (opened at install time, never closed), one WriteFile, and a
// per-thread re-entrancy guard so a fault *inside* the handler is just ignored.
// ---------------------------------------------------------------------------

static HANDLE gCrashLogHandle = INVALID_HANDLE_VALUE;
static HANDLE gCrashErrHandle = INVALID_HANDLE_VALUE;
static DWORD gCrashTlsIndex = TLS_OUT_OF_INDEXES;
// Static, not stack: the handler must not need the (possibly exhausted) stack.
static char gCrashLine[256];
static MEMORY_BASIC_INFORMATION gCrashMbi;

// Exception codes that mean "the process is going down". Everything else -
// debugger/debug-print pseudo-exceptions, thrown-and-caught language exceptions,
// breakpoints, Ctrl+C - is deliberately ignored.
static bool sitlCrashIsFatal(DWORD code)
{
    switch (code) {
    case 0xC0000005:    // access violation
    case 0xC000001D:    // illegal instruction (the executed-ud2 case)
    case 0xC0000025:    // noncontinuable: DCALL
    case 0xC0000026:    // noncontinuable: invalid dispatch
    case 0xC000008C:    // array bounds exceeded
    case 0xC000008D:    // float denormal operand
    case 0xC000008E:    // float divide by zero
    case 0xC000008F:    // float inexact result
    case 0xC0000090:    // float invalid operation
    case 0xC0000091:    // float overflow
    case 0xC0000092:    // float stack check
    case 0xC0000093:    // float underflow
    case 0xC0000094:    // integer divide by zero
    case 0xC0000095:    // integer overflow
    case 0xC00000FD:    // stack overflow
    case 0xC0000374:    // heap corruption detected
    case 0xC0000409:    // fastfail / buffer overrun / __stack_chk_fail
    case 0xC0000602:    // fail-fast exception
        return true;
    default:
        return false;
    }
}

static char *sitlCrashAppendStr(char *p, const char *s)
{
    while (*s != '\0' && p < &gCrashLine[sizeof(gCrashLine) - 2]) {
        *p++ = *s++;
    }
    return p;
}

// Unsigned hex, exactly `digits` digits, no leading zeros beyond that.
static char *sitlCrashAppendHex(char *p, unsigned long long value, int digits)
{
    for (int shift = (digits - 1) * 4; shift >= 0; shift -= 4) {
        if (p >= &gCrashLine[sizeof(gCrashLine) - 2]) {
            break;
        }
        const unsigned nibble = (unsigned)((value >> shift) & 0xF);
        *p++ = (char)(nibble < 10 ? ('0' + nibble) : ('A' + (nibble - 10)));
    }
    return p;
}

// Unsigned hex with no leading zeros at all (for the RVA, which is small).
static char *sitlCrashAppendHexVar(char *p, unsigned long long value)
{
    if (value == 0) {
        if (p < &gCrashLine[sizeof(gCrashLine) - 2]) {
            *p++ = '0';
        }
        return p;
    }
    int shift = 60;
    while (shift > 0 && ((value >> shift) & 0xF) == 0) {
        shift -= 4;
    }
    return sitlCrashAppendHex(p, value, (shift / 4) + 1);
}

static void sitlCrashWriteLine(const char *prefix, char *end)
{
    *end++ = '\r';
    *end++ = '\n';
    const DWORD len = (DWORD)(end - gCrashLine);
    if (gCrashLogHandle != INVALID_HANDLE_VALUE) {
        DWORD written = 0;
        SetFilePointer(gCrashLogHandle, 0, NULL, FILE_END);
        WriteFile(gCrashLogHandle, gCrashLine, len, &written, NULL);
    }
    if (gCrashErrHandle != INVALID_HANDLE_VALUE) {
        DWORD written = 0;
        WriteFile(gCrashErrHandle, prefix, (DWORD)strlen(prefix), &written, NULL);
        WriteFile(gCrashErrHandle, gCrashLine, len, &written, NULL);
    }
}

static LONG WINAPI sitlCrashLogger(EXCEPTION_POINTERS *info)
{
    if (info == NULL || info->ExceptionRecord == NULL) {
        return EXCEPTION_CONTINUE_SEARCH;
    }
    const DWORD code = info->ExceptionRecord->ExceptionCode;
    if (!sitlCrashIsFatal(code)) {
        return EXCEPTION_CONTINUE_SEARCH;
    }

    // Per-thread re-entrancy guard (TlsAlloc, not a static: two threads can fault
    // at the same time, and a fault inside this handler must not recurse).
    if (gCrashTlsIndex != TLS_OUT_OF_INDEXES && TlsGetValue(gCrashTlsIndex) != NULL) {
        return EXCEPTION_CONTINUE_SEARCH;
    }
    if (gCrashTlsIndex != TLS_OUT_OF_INDEXES) {
        TlsSetValue(gCrashTlsIndex, (void *)1);
    }

    const void *addr = info->ExceptionRecord->ExceptionAddress;
    unsigned long long base = 0;
    // VirtualQuery, not GetModuleHandleEx: no loader lock, no allocation, results
    // in a static buffer.
    if (VirtualQuery(addr, &gCrashMbi, sizeof(gCrashMbi)) == sizeof(gCrashMbi)) {
        base = (unsigned long long)(uintptr_t)gCrashMbi.AllocationBase;
    }
    const unsigned long long rva = base != 0 ? (unsigned long long)(uintptr_t)addr - base : 0;

    char *p = gCrashLine;
    p = sitlCrashAppendStr(p, "code=0x");
    p = sitlCrashAppendHex(p, code, 8);
    p = sitlCrashAppendStr(p, " addr=0x");
    p = sitlCrashAppendHex(p, (unsigned long long)(uintptr_t)addr, 12);
    p = sitlCrashAppendStr(p, " module=0x");
    p = sitlCrashAppendHex(p, base, 12);
    p = sitlCrashAppendStr(p, " rva=0x");
    p = sitlCrashAppendHexVar(p, rva);
    p = sitlCrashAppendStr(p, " (addr2line -f -C -e betaflight_SITL.dll 0x");
    p = sitlCrashAppendHexVar(p, rva);
    p = sitlCrashAppendStr(p, ")");
    sitlCrashWriteLine("[SITL-CRASH] ", p);

    if (gCrashTlsIndex != TLS_OUT_OF_INDEXES) {
        TlsSetValue(gCrashTlsIndex, NULL);
    }
    // Never swallow it: the host's crash reporter / the debugger must still see it.
    return EXCEPTION_CONTINUE_SEARCH;
}

void sitlInstallCrashLogger(void)
{
    static LONG installed = 0;
    if (InterlockedCompareExchange(&installed, 1, 0) == 0) {
        char path[MAX_PATH];
        gCrashTlsIndex = TlsAlloc();
        gCrashErrHandle = GetStdHandle(STD_ERROR_HANDLE);

        // Opened once, kept open: a crash must not have to open a file. The audit
        // log path is honoured so a test harness gets the report in its own file.
        if (GetEnvironmentVariableA("BF_SITL_AUDIT_LOG", path, sizeof(path)) <= 0) {
            if (GetEnvironmentVariableA("LOCALAPPDATA", path, sizeof(path)) > 0) {
                strncat(path, "\\Betaflight-SITL\\sitl-crash.log",
                        sizeof(path) - strlen(path) - 1);
            } else {
                path[0] = '\0';
            }
        }
        if (path[0] != '\0') {
            gCrashLogHandle = CreateFileA(path, FILE_APPEND_DATA,
                                          FILE_SHARE_READ | FILE_SHARE_WRITE, NULL,
                                          OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
        }
        AddVectoredExceptionHandler(1, sitlCrashLogger);
    }
}

// ---------------------------------------------------------------------------
// Arming path trace (LOCAL diagnostics).
//
// tryArm() is the only function that arms, and it calls updateArmingStatus()
// itself, so "armFlags == 0 yet never armed" can only mean tryArm() was never
// reached. Its only RC-path caller is processRcStickPositions(), and processRx()
// skips that call entirely while the CMS menu is open. Both are renamed through
// CMakeLists.txt (the pinned submodule is untouched); the trace logs only when
// the inputs change, so a steady throttle/switch costs nothing.
// ---------------------------------------------------------------------------
void sitlProcessRcStickPositionsTraced(void)
{
    extern void sitlProcessRcStickPositionsReal(void);
    extern bool cmsInMenu;

    const uint32_t flags = (uint32_t)getArmingDisableFlags();
    const int armBoxActive = IS_RC_MODE_ACTIVE(BOXARM) ? 1 : 0;
    const int armBoxConfigured = isModeActivationConditionPresent(BOXARM) ? 1 : 0;
    const int cms = cmsInMenu ? 1 : 0;
    const int throttle = (int)rcData[3];      // AETR: 3 = throttle

    static uint32_t lastFlags = 0xFFFFFFFFu;
    static int lastActive = -1, lastConfigured = -1, lastCms = -1;
    if (flags != lastFlags || armBoxActive != lastActive
        || armBoxConfigured != lastConfigured || cms != lastCms) {
        lastFlags = flags;
        lastActive = armBoxActive;
        lastConfigured = armBoxConfigured;
        lastCms = cms;
        sitlAuditLog("arm path: stick positions processed (armFlags=0x%08X throttle=%d "
                     "armBox active/configured=%d/%d cmsInMenu=%d)",
                     (unsigned)flags, throttle, armBoxActive, armBoxConfigured, cms);
    }
    sitlProcessRcStickPositionsReal();
}

void sitlTryArmTraced(void)
{
    extern void sitlTryArmReal(void);
    static uint32_t calls = 0;
    extern timeUs_t sitlLastDshotBeaconCommandTimeUs(void);
    sitlTryArmReal();
    const uint32_t n = ++calls;
    if (n <= 3 || (n % 500) == 0) {
        const timeDelta_t beaconAge = cmpTimeUs(micros(), getLastDshotBeaconCommandTimeUs());
        sitlAuditLog("arm path: tryArm() #%u -> armed=%u (armFlags=0x%08X, beaconAge=%d us, "
                     "guard %s)",
                     (unsigned)n, (unsigned)(ARMING_FLAG(ARMED) ? 1 : 0),
                     (unsigned)getArmingDisableFlags(), (int)beaconAge,
                     (beaconAge >= 0 && beaconAge < 1200000) ? "ACTIVE" : "clear");
    }
}

// Substitute for core.c's DShot beacon guard
// (DSHOT_BEACON_GUARD_DELAY_US = 1.2 s, checked before every arming attempt).
// A real FC plays beacon tones through the ESCs, so it refuses to arm for 1.2 s
// after the last beacon command; SITL's motors are the virtual PWM backend and
// never play one. Left to the real beeper state, a leftover/ongoing beep can
// delay - or, while the beeper keeps refreshing the timestamp, block arming
// indefinitely, with no arming-disable flag set to explain it.
timeUs_t sitlLastDshotBeaconCommandTimeUs(void)
{
    // A DShot beacon can never have been played here (the motors are the virtual
    // PWM backend), so report a timestamp old enough that core.c's guard is always
    // clear. The firmware's own value is unusable in LOCAL mode: beeper.c clamps
    // it to "now - 60 s", and the subtraction is done in uint32, so with a virtual
    // clock younger than 60 s (always, in a stepped sim) the clamp wraps and the
    // guard computes a ~1 s age - i.e. it stays ACTIVE for ever and blocks
    // arming with no arming-disable flag set to explain it.
    extern uint64_t micros64(void);
    return (timeUs_t)(micros64() - 2000000);   // "2 s ago" in the same uint32 domain
}

// The RC switch-off disarm (rc_controls.c) only fires while the firmware
// considers the RX link valid, which is why a probe that never sees a validated
// link stays armed. Trace it so "the switch does not disarm" is visible.
void sitlDisarmTraced(flightLogDisarmReason_e reason)
{
    sitlAuditLog("arm path: disarm() from the ARM switch, reason=%d", (int)reason);
    disarm(reason);
}

// The flight thread's writeEEPROM() calls (fc/stats.c on disarm, cli.c and
// gyro_init.c from their own paths - see CMakeLists.txt) land here instead of
// doing a synchronous 32 KB file write inside a step. The real write is picked
// up by the MSP thread within its next 1 ms poll, exactly like a configurator
// save; the only visible difference is that the EEPROM lags the RAM config by a
// millisecond or two.
void sitlWriteEEPROMDeferred(void)
{
    extern void sitlLocalRequestEepromWrite(void);
    sitlAuditLog("deferred writeEEPROM (flight thread) queued");
    sitlLocalRequestEepromWrite();
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
// %LOCALAPPDATA%\Betaflight-SITL\blackbox (a dedicated folder next to the
// virtual EEPROM, so logs never mix with eeprom.bin / the audit logs), and the
// host can override it per aircraft at runtime with
// sitl_local_set_blackbox_dir() or BF_SITL_BLACKBOX_DIR.
//
// The folder is kept bounded: only the newest BF_SITL_BLACKBOX_MAX_LOGS logs
// (default 10, 0 = unlimited, also settable through
// sitl_local_set_blackbox_max_logs()) are kept, oldest first deleted, both when
// a new log is opened and when the blackbox directory is (re)scanned at boot.
static char gBlackboxDir[MAX_PATH] = "";
static int gBlackboxMaxLogs = 10;
// Upper bound of the folder scan (LOG00000..LOG99999 fits in 5 digits).
#define SITL_BLACKBOX_MAX_TRACKED 1024

// mkdir -p for the blackbox/Eeprom folders (a single CreateDirectoryA would
// fail for "%LOCALAPPDATA%\Betaflight-SITL\blackbox" before the parent exists).
static bool sitlCreateDirChain(char *path)
{
    if (path[0] == '\0' || (path[1] == ':' && path[2] == '\0')) {
        return true;
    }
    const DWORD attrs = GetFileAttributesA(path);
    if (attrs != INVALID_FILE_ATTRIBUTES) {
        return (attrs & FILE_ATTRIBUTE_DIRECTORY) != 0;
    }
    char *slash = strrchr(path, '\\');
    char *fslash = strrchr(path, '/');
    if (fslash > slash) {
        slash = fslash;
    }
    if (slash != NULL && slash != path && slash[1] != '\0') {
        const char saved = *slash;
        *slash = '\0';
        const bool parentOk = sitlCreateDirChain(path);
        *slash = saved;
        if (!parentOk) {
            return false;
        }
    }
    return CreateDirectoryA(path, NULL) != 0 || GetLastError() == ERROR_ALREADY_EXISTS;
}

static bool sitlIsBlackboxLogName(const char *name)
{
    return strlen(name) == 12
        && strncmp(name, "LOG", 3) == 0
        && name[8] == '.'
        && strncmp(name + 9, "BFL", 3) == 0;
}

typedef struct {
    int number;
    char name[MAX_PATH];
} sitlBlackboxLog_t;

// List the LOG*.BFL files in `dir`, sorted by their log number (= their age
// order, because every log is created with a number above all existing ones).
static int sitlBlackboxCollect(const char *dir, sitlBlackboxLog_t *logs, int maxLogs)
{
    int count = 0;
    DIR *d = opendir(dir);
    if (d == NULL) {
        return 0;
    }
    struct dirent *entry;
    while ((entry = readdir(d)) != NULL && count < maxLogs) {
        if (sitlIsBlackboxLogName(entry->d_name)) {
            logs[count].number = atoi(entry->d_name + 3);
            strncpy(logs[count].name, entry->d_name, sizeof(logs[count].name) - 1);
            logs[count].name[sizeof(logs[count].name) - 1] = '\0';
            count++;
        }
    }
    closedir(d);

    for (int i = 1; i < count; i++) {
        const sitlBlackboxLog_t key = logs[i];
        int j = i - 1;
        while (j >= 0 && logs[j].number > key.number) {
            logs[j + 1] = logs[j];
            j--;
        }
        logs[j + 1] = key;
    }
    return count;
}

// Drop the oldest logs until at most `keep` remain (keep < 0 = no limit) and
// renumber the survivors so the folder is a rolling window LOG00001..LOG0000n
// with the newest log always at the highest number. Without the renumbering the
// firmware's own counter (blackbox_virtual.c always uses "largest in folder + 1")
// grows for ever, and a fresh folder inherits the previous folder's numbering.
//
// The renumbering only ever moves a closed log to a *lower* number, processing
// them in ascending number order, so every target is either already correct or
// was freed by the previous step - it can never collide with a file that is
// still to move. The log being written at this moment is untouched (it is
// created with its final name by sitlBlackboxFopen()).
// The directory scan's working buffer (SITL_BLACKBOX_MAX_TRACKED * ~264 B, i.e.
// ~270 KB). It must not live on the caller's stack: both callers sit deep in
// somebody else's stack (the boot/seed path runs under the host's step call, the
// prune runs on the MSP thread), and 270 KB twice over overflows a 1 MB stack and
// takes the whole process down. One shared buffer plus a lock instead.
static sitlBlackboxLog_t gBlackboxScanBuf[SITL_BLACKBOX_MAX_TRACKED];
static volatile LONG gBlackboxScanLock = 0;

static void sitlBlackboxScanLockAcquire(void)
{
    while (InterlockedExchange(&gBlackboxScanLock, 1) != 0) {
        Sleep(0);
    }
}

static void sitlBlackboxScanLockRelease(void)
{
    InterlockedExchange(&gBlackboxScanLock, 0);
}

static int sitlBlackboxEnforce(const char *dir, int keep, const char *openLogName)
{
    sitlBlackboxScanLockAcquire();
    sitlBlackboxLog_t *logs = gBlackboxScanBuf;
    const int count = sitlBlackboxCollect(dir, logs, SITL_BLACKBOX_MAX_TRACKED);
    int first = 0;

    if (keep >= 0 && count > keep) {
        first = count - keep;
        for (int i = 0; i < first; i++) {
            char path[MAX_PATH];
            if (openLogName != NULL && strcmp(logs[i].name, openLogName) == 0) {
                continue;   // never delete the log being written
            }
            _snprintf(path, sizeof(path), "%s\\%s", dir, logs[i].name);
            if (remove(path) == 0) {
                sitlAuditLog("blackbox: pruned %s (keeping the newest %d logs in %s)",
                             logs[i].name, keep, dir);
            }
        }
    }

    for (int i = first; i < count; i++) {
        if (openLogName != NULL && strcmp(logs[i].name, openLogName) == 0) {
            continue;   // an open file cannot be renamed on Windows, and it is
                        // the newest log, so leaving its number alone is fine
        }
        const int target = i - first + 1;
        if (logs[i].number == target) {
            continue;
        }
        char from[MAX_PATH];
        char to[MAX_PATH];
        _snprintf(from, sizeof(from), "%s\\%s", dir, logs[i].name);
        _snprintf(to, sizeof(to), "%s\\LOG%05d.BFL", dir, target);
        if (rename(from, to) != 0) {
            sitlAuditLog("blackbox: could not renumber %s -> LOG%05d.BFL (file in use?)",
                         logs[i].name, target);
        }
    }
    sitlBlackboxScanLockRelease();
    return count - first;
}

// The number of the next log this layer creates. Seeded from the biggest number
// in the folder (the boot path and the prune below may both walk the directory)
// and then incremented per log - that is what lets the flight thread name a new
// log without scanning anything.
static volatile LONG gBlackboxNextNumberValue = 0;
// Path of the log currently being written, so the prune never deletes or
// renames it (an open file cannot be renamed on Windows).
static char gBlackboxOpenLogName[MAX_PATH];
// Raised by the flight thread when it creates a log, cleared and executed by the
// MSP thread.
static volatile LONG gBlackboxPruneRequest = 0;

static const char *sitlBlackboxDir(char *buf, size_t size);

void sitlBlackboxSeedFromDisk(void)
{
    char dir[MAX_PATH];
    int highest = 0;
    if (sitlBlackboxDir(dir, sizeof(dir)) != NULL) {
        sitlBlackboxScanLockAcquire();
        const sitlBlackboxLog_t *logs = gBlackboxScanBuf;
        const int count = sitlBlackboxCollect(dir, logs, SITL_BLACKBOX_MAX_TRACKED);
        for (int i = 0; i < count; i++) {
            if (logs[i].number > highest) {
                highest = logs[i].number;
            }
        }
        sitlBlackboxScanLockRelease();
    }
    InterlockedExchange(&gBlackboxNextNumberValue, highest);
}

static int sitlBlackboxNextNumber(void)
{
    if (gBlackboxNextNumberValue <= 0) {
        sitlBlackboxSeedFromDisk();
    }
    return (int)InterlockedIncrement(&gBlackboxNextNumberValue);
}

// Called by the MSP thread: the directory walk the flight thread refuses to do.
void sitlBlackboxRunPendingPrune(void)
{
    char dir[MAX_PATH];
    if (sitlBlackboxDir(dir, sizeof(dir)) == NULL) {
        return;
    }
    const int keep = (gBlackboxMaxLogs > 0) ? gBlackboxMaxLogs : -1;
    const int remaining = sitlBlackboxEnforce(dir, keep,
                                              gBlackboxOpenLogName[0] ? gBlackboxOpenLogName : NULL);
    sitlAuditLog("blackbox: housekeeping done (%d logs kept, cap %d)", remaining, keep);
}

int sitlBlackboxTakePendingPrune(void)
{
    return InterlockedExchange(&gBlackboxPruneRequest, 0) != 0;
}

static void sitlBlackboxReadMaxLogsSetting(void)
{
    const char *env = getenv("BF_SITL_BLACKBOX_MAX_LOGS");
    if (env != NULL && env[0] != '\0') {
        const long v = strtol(env, NULL, 10);
        if (v >= 0 && v <= 100000) {
            gBlackboxMaxLogs = (int)v;
        }
    }
}

static const char *sitlBlackboxDir(char *buf, size_t size)
{
    if (gBlackboxDir[0] != '\0') {
        if (strlen(gBlackboxDir) + 1 > size) {
            return NULL;
        }
        memcpy(buf, gBlackboxDir, strlen(gBlackboxDir) + 1);
        sitlCreateDirChain(buf);
        return buf;
    }
    const char *env = getenv("BF_SITL_BLACKBOX_DIR");
    if (env != NULL && env[0] != '\0') {
        if (strlen(env) + 1 > size) {
            return NULL;
        }
        memcpy(buf, env, strlen(env) + 1);
        sitlCreateDirChain(buf);
        return buf;
    }
    if (GetEnvironmentVariableA("LOCALAPPDATA", buf, (DWORD)size) > 0) {
        _snprintf(buf + strlen(buf), size - strlen(buf), "\\Betaflight-SITL\\blackbox");
        sitlCreateDirChain(buf);
        return buf;
    }
    return NULL;
}

FILE *sitlBlackboxFopen(const char *filename, const char *mode)
{
    // The firmware writes blackbox frames one byte at a time (fputc), and a
    // FILE* opened without an explicit buffer uses the CRT default (512 bytes
    // on MinGW/ucrt). Give it a large buffer of our own: the per-byte writes
    // then never reach the OS, and the throttled flush below is the only thing
    // that touches the disk.
    static char blackboxFileBuffer[64 * 1024];
    FILE *fp = NULL;
    char dir[MAX_PATH];
    if (sitlBlackboxDir(dir, sizeof(dir)) != NULL) {
        char path[MAX_PATH];
        const bool newLog = sitlIsBlackboxLogName(filename)
            && (mode[0] == 'w') && (mode[1] == '\0' || mode[1] == 'b');
        if (newLog) {
            // No directory walk and no prune here: sitlBlackboxCollect() and
            // sitlBlackboxEnforce() are opendir + readdir + remove/rename walks,
            // and this runs on the flight thread (blackboxVirtualBeginLog() is
            // called from the blackbox task, i.e. inside a step). The name comes
            // from a counter that the boot/prune path keeps in sync with the
            // folder, and the housekeeping is requested instead - the MSP thread
            // runs it a millisecond or two later.
            // BF_SITL_BLACKBOX_LEGACY_OPEN=1 restores the old behaviour (scan
            // the folder and prune inline, on the flight thread) so the A/B for
            // "is the scan-free open responsible" can be run without a rebuild.
            static int legacyOpen = -1;
            if (legacyOpen < 0) {
                const char *env = getenv("BF_SITL_BLACKBOX_LEGACY_OPEN");
                legacyOpen = (env != NULL && env[0] != '\0' && env[0] != '0') ? 1 : 0;
            }
            const int number = legacyOpen
                ? sitlBlackboxEnforce(dir, (gBlackboxMaxLogs > 0) ? gBlackboxMaxLogs - 1 : -1,
                                      NULL) + 1
                : sitlBlackboxNextNumber();
            _snprintf(path, sizeof(path), "%s\\LOG%05d.BFL", dir, number % 100000);
            fp = fopen(path, mode);
            if (fp != NULL) {
                strncpy(gBlackboxOpenLogName, path, sizeof(gBlackboxOpenLogName) - 1);
                gBlackboxOpenLogName[sizeof(gBlackboxOpenLogName) - 1] = '\0';
                sitlAuditLog("blackbox: new log LOG%05d.BFL in %s (max %d)",
                             number % 100000, dir, gBlackboxMaxLogs);
                if (!legacyOpen) {
                    gBlackboxPruneRequest = 1;
                }
            }
        } else {
            _snprintf(path, sizeof(path), "%s\\%s", dir, filename);
            fp = fopen(path, mode);
        }
    } else {
        fp = fopen(filename, mode);
    }
    if (fp != NULL) {
        setvbuf(fp, blackboxFileBuffer, _IOFBF, sizeof(blackboxFileBuffer));
    }
    return fp;
}

// The per-iteration flush blackbox.c performs (renamed to this symbol for the
// LOCAL build, see CMakeLists.txt). It exists so that a decode-to-date log file
// is on disk at all times, but a 4 kHz loop logs ~1000 frames/s and every
// flush() is a WriteFile syscall from the flight loop. The log is only read
// after it is closed, so fuse that to a few Hz: nothing is lost - the forced
// flushes at log start/stop and the shutdown path still write everything out -
// and the flight loop stops paying for the file system.
void sitlBlackboxDeviceFlushThrottled(void)
{
    extern void blackboxDeviceFlush(void);
    static uint32_t flushIntervalMs = 0;
    static uint64_t lastFlushUs = 0;
    if (flushIntervalMs == 0) {
        const char *env = getenv("BF_SITL_BLACKBOX_FLUSH_MS");
        const long v = (env != NULL && env[0] != '\0') ? strtol(env, NULL, 10) : 100;
        flushIntervalMs = (v >= 1 && v <= 5000) ? (uint32_t)v : 100;
    }
    extern uint64_t sitlWallUs(void);
    const uint64_t nowUs = sitlWallUs();
    if (nowUs - lastFlushUs < (uint64_t)flushIntervalMs * 1000ULL) {
        return;
    }
    lastFlushUs = nowUs;
    blackboxDeviceFlush();
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
    sitlBlackboxReadMaxLogsSetting();
    char dir[MAX_PATH];
    if (sitlBlackboxDir(dir, sizeof(dir)) != NULL) {
        sitlAuditLog("blackbox: directory %s (keeping at most %d logs)",
                     dir, gBlackboxMaxLogs);
        // Trim to the cap and renumber the survivors to LOG00001.. so a folder
        // that was left with drifting numbers (or came from an older build) is
        // tidied up at every boot / directory switch.
        const int kept = sitlBlackboxEnforce(dir, (gBlackboxMaxLogs > 0) ? gBlackboxMaxLogs : -1, NULL);
        // The survivors are LOG00001..LOG000kept, so the next log this layer
        // creates is kept+1. Seeding the counter here is what lets the flight
        // thread open a log without walking the folder (see sitlBlackboxFopen).
        InterlockedExchange(&gBlackboxNextNumberValue, kept);
    }
    return sitlBlackboxVirtualOpenReal();
}

int sitl_local_set_blackbox_dir(const char *path)
{
    if (path == NULL || strlen(path) >= MAX_PATH) {
        return -1;
    }
    if (path[0] == '\0') {
        // Empty path: back to the default (BF_SITL_BLACKBOX_DIR, else
        // %LOCALAPPDATA%\Betaflight-SITL\blackbox).
        gBlackboxDir[0] = '\0';
        blackboxVirtualOpen();
        return 0;
    }
    strncpy(gBlackboxDir, path, sizeof(gBlackboxDir) - 1);
    gBlackboxDir[sizeof(gBlackboxDir) - 1] = '\0';
    sitlCreateDirChain(gBlackboxDir);
    blackboxVirtualOpen(); // re-scan for correct numbering in the new folder
    return 0;
}

// Keep at most `maxLogs` logs in the blackbox folder (0 = unlimited). The cap is
// applied immediately (the oldest logs are deleted) and again whenever a new log
// is opened, so a long session can never fill the disk. Returns 0, or -1 for an
// out-of-range value.
int sitl_local_set_blackbox_max_logs(int maxLogs)
{
    if (maxLogs < 0 || maxLogs > 100000) {
        return -1;
    }
    gBlackboxMaxLogs = maxLogs;
    char dir[MAX_PATH];
    if (sitlBlackboxDir(dir, sizeof(dir)) != NULL) {
        const int kept = sitlBlackboxEnforce(dir, (gBlackboxMaxLogs > 0) ? gBlackboxMaxLogs : -1,
                                             gBlackboxOpenLogName[0] ? gBlackboxOpenLogName : NULL);
        InterlockedExchange(&gBlackboxNextNumberValue, kept);
    }
    return 0;
}

// Current blackbox directory ("" when the platform default is used), for a host
// that wants to show or log where the logs land. Returns 0 on success, -1 when
// the buffer is too small.
int sitl_local_get_blackbox_dir(char *out, int size)
{
    if (out == NULL || size <= 0) {
        return -1;
    }
    char dir[MAX_PATH];
    if (sitlBlackboxDir(dir, sizeof(dir)) == NULL || (int)strlen(dir) + 1 > size) {
        return -1;
    }
    memcpy(out, dir, strlen(dir) + 1);
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
    // No-op: the value comes from the simulator (sitlBatteryVoltageRead).
    //
    // This hook runs *inside* batteryUpdateVoltage(), before voltageMeterADCRead()
    // has produced this tick's value - so presence detection belongs in
    // batteryUpdateVoltage() below, after the complete stock update.
}

// battery.c's batteryUpdateVoltage() is renamed to sitlBatteryUpdateVoltageReal()
// by CMakeLists.txt; this is the firmware-facing entry point (TASK_BATTERY_VOLTAGE,
// 50 Hz).
void batteryUpdateVoltage(timeUs_t currentTimeUs)
{
    extern void sitlBatteryUpdateVoltageReal(timeUs_t currentTimeUs);

    sitlBatteryUpdateVoltageReal(currentTimeUs);

    // voltageMeter now holds this tick's filtered value.
    //
    // Stock firmware probes for the battery only from taskBatteryAlerts() and
    // only while DISARMED. A host that arms before the voltage filter reports a
    // stable pack (e.g. right after a reboot, when batteryInit() has just reset
    // batteryCellCount to 0) would then never get a cell count - and every
    // consumer of it (MSP/OSD cell voltage, current thresholds, mAh) stays at 0
    // until a disarm/rearm cycle. The SITL battery is physically always
    // connected, so keep probing from the 50 Hz voltage task until the count
    // exists; batteryUpdatePresence() is idempotent once it does.
    if (getBatteryCellCount() == 0) {
        batteryUpdatePresence();
    }
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
