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

#ifdef CONFIG_IN_FILE
#include "cli/cli.h"
#endif

void run(void);

#ifdef _WIN32
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

    // 1 ms system timer resolution so the low-CPU scheduler poll actually
    // sleeps ~1 ms instead of the default ~15.6 ms timer tick.
    timeBeginPeriod(1);
#endif

    ensureWritableWorkingDirectory();

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
    // Gyro/filter/PID always run at 1 kHz, matching the external physics
    // simulator's rate. In the default official real-time mode the scheduler
    // busy-waits to the exact deadline, so 1000 us is the locked loop period.
    // In the opt-in low-CPU mode (BF_SITL_LOW_CPU=1, see wincompat.c) the
    // stepped virtual time paces the scheduler with ~1 ms sleeps; the serial
    // task then needs a shorter period than its default 10 ms or it loses the
    // priority selection to medium-priority tasks at the reduced cadence.
    const char *lowCpu = getenv("BF_SITL_LOW_CPU");
    const bool lowCpuMode = (lowCpu != NULL && strcmp(lowCpu, "1") == 0);
    rescheduleTask(TASK_GYRO, 1000);
    rescheduleTask(TASK_FILTER, 1000);
    rescheduleTask(TASK_PID, 1000);
    if (lowCpuMode) {
        // The reduced scheduler cadence makes the default 10 ms serial task
        // (low priority) lose the priority selection to medium-priority
        // tasks, which starves MSP and makes the configurator time out during
        // its initial request burst. A 100 us period makes the serial task
        // due on every scheduler pass so MSP stays responsive.
        rescheduleTask(TASK_SERIAL, 100);
    }
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
    const char *lowCpu = getenv("BF_SITL_LOW_CPU");
    const bool lowCpuMode = (lowCpu != NULL && strcmp(lowCpu, "1") == 0);
    uint32_t iter = 0;
    while (true) {
        if (lowCpuMode) {
            // Virtual time advances 500 us per scheduler pass (2 kHz virtual
            // clock). With the gyro/PID period rescheduled to 1000 us below,
            // the scheduler runs gyro+filter+PID every second pass and gives
            // the serial/MSP task the other passes, so nothing starves.
            sitlStepTime(500);
        }
        scheduler();

        if (lowCpuMode) {
            // Pace the real-time loop: sleep ~1 ms every four passes, which
            // puts the scheduler cadence near 2 kHz so gyro/PID runs at
            // ~1 kHz while the serial/MSP task gets the alternate passes.
            if ((++iter % 4) == 0) {
                Sleep(1);
            }
        }
#if defined(RUN_LOOP_DELAY_US) && RUN_LOOP_DELAY_US > 0
        else {
            delayMicroseconds_real(RUN_LOOP_DELAY_US);
        }
#endif
    }
}
