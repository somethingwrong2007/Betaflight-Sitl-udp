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
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#include <fcntl.h>
#include <io.h>
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
    while (true) {
        scheduler();

#if defined(RUN_LOOP_DELAY_US) && RUN_LOOP_DELAY_US > 0
        delayMicroseconds_real(RUN_LOOP_DELAY_US);
#endif
    }
}
