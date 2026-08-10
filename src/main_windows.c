/*
 * Windows entry point for the Betaflight SITL UDP server.
 *
 * This mirrors the official src/main/main.c but first makes sure the current
 * working directory is writable, so eeprom.bin can always be created even
 * when the executable is launched from a read-only directory (for example
 * directly from an archive or a protected folder).
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include <mmsystem.h>
#endif

#include "platform.h"

#include "drivers/light_led.h"
#include "drivers/time.h"

#ifdef USE_VCP
#include "drivers/serial_usb_vcp.h"
#endif

#include "drivers/system.h"

#if ENABLE_LCD_CONSOLE && ENABLE_LCD_PRINTF_REDIRECT
#include "common/printf_serial.h"
#include "drivers/serial_lcd_console.h"
#endif

#include "fc/init.h"

#if defined(ENABLE_MULTICORE_INIT) && !defined(USE_MULTICORE)
#error "ENABLE_MULTICORE_INIT requires USE_MULTICORE"
#endif

#ifdef USE_MULTICORE
#include "platform/multicore.h"
#endif

#ifdef USE_USB_MSC
#include "drivers/usb_msc.h"
#endif

#include "scheduler/scheduler.h"
#include "fc/tasks.h"
#include "msp/msp.h"

#ifdef CONFIG_IN_FILE
#include "cli/cli.h"
#endif

void run(void);

#ifdef _WIN32
#ifdef SITL_UDP_TIME
extern void sitlUdpFdmInit(void);

// msp_serial.h pulls in io/serial.h, which collides with the MinGW windows.h
// include chain; declare just what the idle MSP keep-alive needs.
typedef enum {
    MSP_EVALUATE_NON_MSP_DATA = 0,
    MSP_SKIP_NON_MSP_DATA
} mspEvaluateNonMspData_e;

void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData,
                      mspProcessCommandFnPtr mspProcessCommandFn,
                      mspProcessReplyFnPtr mspProcessReplyFn);
#endif

static uint32_t sitlGyroHz(void)
{
    const char *env = getenv("BF_SITL_GYRO_HZ");
    if (env != NULL && env[0] != '\0') {
        const long v = strtol(env, NULL, 10);
        if (v >= 100 && v <= 10000) {
            return (uint32_t)v;
        }
    }
#ifdef SITL_GYRO_HZ
    return SITL_GYRO_HZ;
#else
    return 1000;
#endif
}

static bool directoryExistsOrCreate(const char *path)
{
    const DWORD attrs = GetFileAttributesA(path);
    if (attrs != INVALID_FILE_ATTRIBUTES && (attrs & FILE_ATTRIBUTE_DIRECTORY)) {
        return true;
    }
    return CreateDirectoryA(path, NULL) != 0 || GetLastError() == ERROR_ALREADY_EXISTS;
}

static bool trySetWorkingDirectory(const char *path)
{
    if (path == NULL || path[0] == '\0') {
        return false;
    }
    if (!directoryExistsOrCreate(path)) {
        return false;
    }
    return SetCurrentDirectoryA(path) != 0;
}

// Create every missing component of a directory path (mkdir -p equivalent).
static bool createDirectoryChain(char *path)
{
    if (path[0] == '\0' || (path[1] == ':' && path[2] == '\0')) {
        return true; // empty or drive root
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
        const bool parentOk = createDirectoryChain(path);
        *slash = saved;
        if (!parentOk) {
            return false;
        }
    }
    return CreateDirectoryA(path, NULL) != 0 || GetLastError() == ERROR_ALREADY_EXISTS;
}

// If BF_SITL_EEPROM points into a directory that does not exist yet, create
// it before init so sitl.c's virtual EEPROM can create the file there.
static void ensureEepromDirectory(void)
{
    const char *eeprom = getenv("BF_SITL_EEPROM");
    if (eeprom == NULL || eeprom[0] == '\0') {
        return;
    }
    char path[MAX_PATH];
    strncpy(path, eeprom, sizeof(path) - 1);
    path[sizeof(path) - 1] = '\0';
    char *slash = strrchr(path, '\\');
    char *fslash = strrchr(path, '/');
    if (fslash > slash) {
        slash = fslash;
    }
    if (slash != NULL && slash != path) {
        *slash = '\0';
        createDirectoryChain(path);
    }
}

static void ensureWritableWorkingDirectory(void)
{
    // If the current directory already lets us create the EEPROM file, keep
    // it. With BF_SITL_EEPROM set, probe that file (its directory is created
    // by ensureEepromDirectory) so no stray eeprom.bin appears in the CWD.
    const char *eepromOverride = getenv("BF_SITL_EEPROM");
    const char *probeName = (eepromOverride != NULL && eepromOverride[0] != '\0')
        ? eepromOverride
        : "eeprom.bin";
    FILE *probe = fopen(probeName, "a");
    if (probe != NULL) {
        fclose(probe);
        return;
    }

    // Fall back to the executable's directory.
    char exePath[MAX_PATH];
    if (GetModuleFileNameA(NULL, exePath, MAX_PATH) > 0) {
        char *slash = strrchr(exePath, '\\');
        if (slash != NULL) {
            *slash = '\0';
            if (trySetWorkingDirectory(exePath)) {
                return;
            }
        }
    }

    // Last resort: a per-user writable directory.
    char appDataPath[MAX_PATH];
    if (GetEnvironmentVariableA("LOCALAPPDATA", appDataPath, MAX_PATH) > 0) {
        char sitlPath[MAX_PATH];
        _snprintf(sitlPath, sizeof(sitlPath), "%s\\Betaflight-SITL", appDataPath);
        if (trySetWorkingDirectory(sitlPath)) {
            return;
        }
    }

    if (GetTempPathA(MAX_PATH, appDataPath) > 0) {
        trySetWorkingDirectory(appDataPath);
    }
}
#else
static void ensureWritableWorkingDirectory(void)
{
}
#endif

int main(int argc, char *argv[])
{
#ifdef _WIN32
    // Keep SITL progress visible when stderr is redirected to a file/pipe.
    // stdout is fully buffered: the virtual EEPROM writer prints one line
    // per config word on every save, and unbuffered per-line file writes made
    // saves take seconds and time out the configurator (which drops the
    // connection). A large buffer coalesces those writes into a few flushes.
    setvbuf(stdout, NULL, _IOFBF, 64 * 1024);
    setvbuf(stderr, NULL, _IONBF, 0);

    // eeprom.bin is binary data; without this, fopen() uses text mode on
    // Windows and 0x0A bytes are mangled by CRLF translation.
    _fmode = _O_BINARY;

    // 1 ms system timer resolution so short sleeps in the helper threads
    // (UDP links, WebSocket proxy) do not round up to the ~15.6 ms tick.
    timeBeginPeriod(1);
#endif

    ensureWritableWorkingDirectory();
    ensureEepromDirectory();
#ifdef SITL_UDP_TIME
    sitlUdpFdmInit();
#endif

#ifdef USE_MAIN_ARGS
    targetParseArgs(argc, argv);
#else
    UNUSED(argc);
    UNUSED(argv);
#endif

#if SERIAL_PORT_COUNT > 0
    printfSerialInit();
#endif

    systemInit();

#ifdef ENABLE_MULTICORE_INIT
    multicoreExecuteBlocking(initPhase1);
    multicoreExecuteBlocking(initPhase2);
#else
    initPhase1();
    initPhase2();
#endif

#ifdef USE_USB_MSC
    mscButtonInit();
    if (checkMsc()) {
        initMsc();
        return 0;
    }
#endif

#ifdef USE_VCP
    usbVcpInit();
#endif

#ifdef ENABLE_MULTICORE_INIT
    multicoreExecuteBlocking(initPhase3);
#else
    initPhase3();
#endif

#ifdef _WIN32
    // Gyro/filter/PID run at the configured frequency (default 1 kHz,
    // override with BF_SITL_GYRO_HZ or the SITL_GYRO_HZ CMake option). The
    // official real-time scheduler busy-waits to the exact deadline, so the
    // period is the locked loop time.
    const uint32_t gyroPeriodUs = 1000000u / sitlGyroHz();
    rescheduleTask(TASK_GYRO, gyroPeriodUs);
    rescheduleTask(TASK_FILTER, gyroPeriodUs);
    rescheduleTask(TASK_PID, gyroPeriodUs);
#endif

#if ENABLE_LCD_CONSOLE && ENABLE_LCD_PRINTF_REDIRECT
    {
        struct serialPort_s *lcdPort = lcdConsoleSerialOpen();
        if (lcdPort) {
            setPrintfSerialPort(lcdPort);
        }
    }
#endif

#ifdef CONFIG_IN_FILE
    {
        const char *configFile = targetGetConfigFile();
        if (configFile) {
            cliProcessConfigFile(configFile);
            return 0;
        }
    }
#endif

    run();

    return 0;
}

void FAST_CODE run(void)
{
#ifdef SITL_UDP_TIME
    extern void sitlStepTime(uint64_t stepUs);
    extern HANDLE sitlUdpFdmEvent(void);
    extern uint64_t sitlUdpFdmTakePendingUs(void);
    extern bool sitlUdpFdmHasData(void);

    // Virtual clock quantum. Stepping on a grid that divides the gyro period
    // keeps the scheduler's busy-wait aligned: after each step the time left
    // to the gyro deadline is a multiple of the quantum, so the poll never
    // spins on a sub-step remainder.
    const uint64_t stepUs = 100;
    const uint32_t gyroPeriodUs = 1000000u / sitlGyroHz();
    if ((gyroPeriodUs % stepUs) != 0) {
        fprintf(stderr, "[SITL] gyro period %u us not divisible by %llu us\n",
                (unsigned)gyroPeriodUs, (unsigned long long)stepUs);
    }

    uint64_t pendingUs = 0;
    HANDLE fdmEvent = sitlUdpFdmEvent();
    bool fdmLogged = false;

    // The virtual clock starts at 0 and schedulerInit anchored the gyro grid
    // at 0, so fast-forward to the first deadline in fixed quanta. Each pass
    // keeps the scheduler busy-wait aligned (remaining is always a multiple
    // of the quantum) and never triggers its catch-up recovery.
    while (getTask(TASK_GYRO)->lastExecutedAtUs == 0) {
        sitlStepTime(stepUs);
        scheduler();
    }
    fprintf(stderr, "[SITL] UDP time mode: waiting for FDM packets on 127.0.0.1:9003\n");

    while (true) {
        // Consume the pending packet time in fixed quanta: one scheduler pass
        // per quantum, so gyro/filter/PID fire on the FDM timestamp grid.
        while (pendingUs >= stepUs) {
            sitlStepTime(stepUs);
            pendingUs -= stepUs;
            scheduler();
        }
        pendingUs += sitlUdpFdmTakePendingUs();
        if (pendingUs >= stepUs) {
            continue;
        }
        // No packet time left: idle until the next FDM packet. The virtual
        // clock stays frozen, so the flight loop does not run; the brief
        // periodic wake processes serial/MSP directly (the time-driven
        // SERIAL task would never become due on a frozen clock), keeping the
        // ground station connected while waiting.
        if (!fdmLogged && sitlUdpFdmHasData()) {
            fdmLogged = true;
            fprintf(stderr, "[SITL] UDP FDM packets active\n");
        }
        if (fdmEvent == NULL || WaitForSingleObject(fdmEvent, 10) == WAIT_TIMEOUT) {
            mspSerialProcess(MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand, mspFcProcessReply);
        }
    }
#else
    // Official Betaflight run loop: call the scheduler in a tight loop. The
    // scheduler itself busy-waits to the exact gyro deadline, which locks the
    // gyro/filter/PID rate precisely at the cost of one CPU core.
    while (true) {
        scheduler();
#if defined(RUN_LOOP_DELAY_US) && RUN_LOOP_DELAY_US > 0
        delayMicroseconds_real(RUN_LOOP_DELAY_US);
#endif
    }
#endif
}
