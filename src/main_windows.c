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
    // Official Betaflight run loop: call the scheduler in a tight loop. The
    // scheduler itself busy-waits to the exact gyro deadline, which locks the
    // gyro/filter/PID rate precisely at the cost of one CPU core.
    while (true) {
        scheduler();
#if defined(RUN_LOOP_DELAY_US) && RUN_LOOP_DELAY_US > 0
        delayMicroseconds_real(RUN_LOOP_DELAY_US);
#endif
    }
}
