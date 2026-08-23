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
#include "flight/pid.h"
#include "flight/pid_init.h"
#include "sensors/gyro_init.h"

#ifdef _WIN32
#include <windows.h>
#include <pthread.h>
#include <dirent.h>

#include "common/time.h"
#include "sensors/voltage.h"
#include "sensors/current.h"
#include "fc/runtime_config.h"
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

// Real firmware re-runs init on every reboot, which applies boot-time config
// (filters, debug mode, ...) to runtime state. The LOCAL reboot keeps the
// process alive, so re-apply the same config here: the gyro/dterm filter
// chains are rebuilt from the saved settings (Filter tab) and debugMode is
// re-synced (CHIRP blackbox). Filter re-init is skipped while armed so it
// never races the flight loop; a later save/reboot while disarmed applies it.
void sitlLocalReapplyBootConfig(void)
{
#ifdef SITL_LOCAL
    sitlLocalSyncDebugMode();
    if (!ARMING_FLAG(ARMED)) {
        gyroInitFilters();
        pidInit(currentPidProfile);
    }
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
}

#ifdef SITL_LOCAL
// Simulated motor RPM bridge. dshot.c's getDshotRpm/getDshotRpmAverage/
// getDshotErpm/getMotorFrequencyHz/getMinMotorFrequencyHz are renamed to the
// sitl*Real symbols below; these wrappers return the simulator-provided RPM
// (sitl_local_input_t.motor_rpm / the UDP extended tail) whenever it is
// nonzero, and fall back to the real (always-zero in SITL) telemetry state.
#include "pg/motor.h"

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
// in-process FC, so treat it like the firmware reboot: persist and jump back
// to the MSP thread loop instead (mspRebootFn's bootloader branch spins in a
// `while (true);` after this returns, exactly like the firmware case).
void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);
    writeEEPROM();
    unsetArmingDisabled(ARMING_DISABLED_CLI);
    sitlLocalReapplyBootConfig();
    sitlLocalRebootJump();
}

#endif // SITL_LOCAL

// sitl.c's fopen() calls are renamed to sitlFopen() by CMakeLists.txt so the
// virtual EEPROM file can be redirected without touching the Betaflight
// submodule. Set BF_SITL_EEPROM to a file path to keep separate configs
// (e.g. "E:\sim\unreal.bin"); the default stays eeprom.bin in the working
// directory.
#define SITL_EEPROM_FILENAME "eeprom.bin"

FILE *sitlFopen(const char *filename, const char *mode)
{
    if (strcmp(filename, SITL_EEPROM_FILENAME) == 0) {
        const char *eeprom = getenv("BF_SITL_EEPROM");
        if (eeprom != NULL && eeprom[0] != '\0') {
            return fopen(eeprom, mode);
        }
    }
    return fopen(filename, mode);
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
    blackboxFinish();
    unsetArmingDisabled(ARMING_DISABLED_CLI);
    sitlLocalReapplyBootConfig();
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
#ifdef USE_BLACKBOX
    // Close any in-progress blackbox log cleanly (writes the end-of-log event)
    // before relaunching, so a reboot never leaves a truncated .BFL file.
    blackboxFinish();
#endif
#ifdef SITL_LOCAL
    // In-process library mode: there is no standalone process to relaunch and
    // exiting would kill the host engine. Defer the (potentially slow)
    // EEPROM persist to the background thread so systemReset() never blocks
    // the UE thread that may be executing it via the scheduler's TASK_SERIAL
    // (the MSP caller already persisted via sitlSystemReset when applicable).
    extern void sitlLocalRequestReset(void);
    sitlLocalRequestReset();
    // cliEnter() sets ARMING_DISABLED_CLI and nothing ever clears it (real
    // FCs clear it on reboot, which LOCAL mode does not do). Clear it so the
    // craft can arm again after the CLI panel is closed.
    unsetArmingDisabled(ARMING_DISABLED_CLI);
    sitlLocalReapplyBootConfig();
#else
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
