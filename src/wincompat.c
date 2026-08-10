/**
 * Small POSIX compatibility shims for the MinGW build of Betaflight SITL.
 */

#include <string.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "platform.h"
#include "drivers/io.h"
#include "drivers/serial.h"

#ifdef _WIN32
#include <windows.h>
#include <pthread.h>

extern uint64_t micros64_real(void);
extern void sitlDelayMicroseconds(uint32_t us);
extern void writeEEPROM(void);

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
    return 0;
}

// msp.c's systemReset() calls are renamed to sitlSystemReset() by
// CMakeLists.txt. A firmware reboot terminates the SITL process, so persist
// the current RAM config first - this makes the configurator's "Save and
// Reboot" always save, even if the MSP_EEPROM_WRITE step was skipped or lost.
void sitlSystemReset(void)
{
    writeEEPROM();
    systemReset();
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
