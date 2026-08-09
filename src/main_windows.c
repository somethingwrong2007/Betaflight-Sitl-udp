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

#ifdef CONFIG_IN_FILE
#include "cli/cli.h"
#endif

void run(void);

#ifdef _WIN32
extern void sitlUdpFdmInit(void);

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

static void ensureWritableWorkingDirectory(void)
{
    // If the current directory already lets us create eeprom.bin, keep it.
    FILE *probe = fopen("eeprom.bin", "a");
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
    // Keep SITL progress visible when stdout is redirected to a file/pipe.
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    // eeprom.bin is binary data; without this, fopen() uses text mode on
    // Windows and 0x0A bytes are mangled by CRLF translation.
    _fmode = _O_BINARY;

    // 1 ms system timer resolution so short sleeps in the helper threads
    // (UDP links, WebSocket proxy) do not round up to the ~15.6 ms tick.
    timeBeginPeriod(1);
#endif

    ensureWritableWorkingDirectory();
    sitlUdpFdmInit();

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
    extern void sitlStepTime(uint64_t stepUs);
    extern void sitlSetTimeModeUdp(uint64_t syncUs);
    extern void sitlSetTimeModeRealtime(void);
    extern bool sitlTimeIsUdpDriven(void);
    extern uint64_t micros64_real(void);
    extern HANDLE sitlUdpFdmEvent(void);
    extern uint64_t sitlUdpFdmTakePendingUs(void);
    extern bool sitlUdpFdmHasData(void);
    extern void sitlUdpFdmReset(void);

    // Virtual clock quantum. Stepping on a grid that divides the gyro period
    // keeps the scheduler's busy-wait aligned: after each step the time left
    // to the gyro deadline is a multiple of the quantum, so the poll never
    // spins on a sub-step remainder.
    const uint64_t stepUs = 100;
    const uint32_t gyroPeriodUs = 1000000u / sitlGyroHz();
    const bool udpTimeCompatible = (gyroPeriodUs % stepUs) == 0;
    if (!udpTimeCompatible) {
        fprintf(stderr, "[SITL] gyro period %u us not divisible by %llu us, Unreal UDP time disabled\n",
                (unsigned)gyroPeriodUs, (unsigned long long)stepUs);
    }

    bool udpMode = false;
    uint64_t pendingUs = 0;
    HANDLE fdmEvent = sitlUdpFdmEvent();

    while (true) {
        if (!udpMode) {
            // Official Betaflight run loop: call the scheduler in a tight
            // loop. The scheduler itself busy-waits to the exact gyro
            // deadline, which locks the gyro/filter/PID rate precisely at the
            // cost of one CPU core.
            scheduler();
            // Auto-switch: once Unreal FDM packets arrive, hand the clock to
            // the packet stream.
            if (udpTimeCompatible && sitlUdpFdmHasData()) {
                // Align the virtual clock to the scheduler's gyro deadline
                // grid (fixed residue mod the quantum) so stepping never
                // leaves the busy-wait spinning on a sub-step remainder.
                const timeUs_t gyroExec = getTask(TASK_GYRO)->lastExecutedAtUs;
                const uint64_t nowUs = micros64();
                const uint32_t align = (uint32_t)((gyroExec % stepUs + stepUs - (uint32_t)(nowUs % stepUs)) % stepUs);
                sitlSetTimeModeUdp(nowUs + align);
                udpMode = true;
                fprintf(stderr, "[SITL] Unreal FDM detected, switched to UDP-driven virtual time\n");
            }
#if defined(RUN_LOOP_DELAY_US) && RUN_LOOP_DELAY_US > 0
            else {
                delayMicroseconds_real(RUN_LOOP_DELAY_US);
            }
#endif
        } else {
            // UDP-driven virtual time: consume the pending packet time in
            // fixed quanta (one scheduler pass per quantum), then block on
            // the FDM socket event until the next packet. The scheduler's
            // gyro fires on its 1000 us grid exactly once per Unreal ms and
            // the thread sleeps at near-zero CPU between packets.
            while (pendingUs >= stepUs) {
                sitlStepTime(stepUs);
                pendingUs -= stepUs;
                scheduler();
            }
            pendingUs += sitlUdpFdmTakePendingUs();
            if (pendingUs >= stepUs) {
                continue;
            }
            const DWORD waitResult = (fdmEvent == NULL) ? WAIT_TIMEOUT : WaitForSingleObject(fdmEvent, 500);
            if (waitResult == WAIT_TIMEOUT) {
                // No packets for 500 ms: hand the clock back to real time.
                // First walk the virtual clock forward to real time so the
                // scheduler's gyro grid (anchored to the last virtual
                // deadline) stays within one period of the real clock; its
                // catch-up recovery cannot resume from a larger gap.
                pendingUs = 0;
                while (sitlTimeIsUdpDriven() && micros64() < micros64_real()) {
                    sitlStepTime(stepUs);
                    if ((micros64() % gyroPeriodUs) == 0) {
                        scheduler();
                    }
                }
                sitlSetTimeModeRealtime();
                sitlUdpFdmReset();
                udpMode = false;
                fprintf(stderr, "[SITL] Unreal FDM lost, back to real-time mode\n");
            }
        }
    }
}
