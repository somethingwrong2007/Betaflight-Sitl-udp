/**
 * Small POSIX compatibility shims for the MinGW build of Betaflight SITL.
 */

#include <string.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "platform.h"
#include "drivers/io.h"
#include "drivers/serial.h"

#ifdef _WIN32
#include <windows.h>

extern uint64_t micros64_real(void);
extern void sitlDelayMicroseconds(uint32_t us);

void dyad_update(void)
{
    // The dyad TCP bridge is replaced by serial_tcp_win.c on Windows, so the
    // SITL tcpWorker thread only spins here. With no streams, select() returns
    // immediately on Windows, so dyad_update() would burn a full CPU core.
    // Sleep instead; 10 ms matches the update timeout the SITL would use.
    Sleep(10);
}

// Time-base mode: the default official real-time clock, or the Unreal
// UDP-driven virtual clock (stepped from FDM packet timestamps by the run
// loop). In UDP mode micros/millis/getCycleCounter follow the external sim
// clock so gyro/PID run once per 1000 us of Unreal time while the main loop
// blocks on the UDP socket at near-zero CPU.
typedef enum {
    SITL_TIME_REALTIME = 0,
    SITL_TIME_UDP
} sitlTimeMode_e;

static sitlTimeMode_e sitlTimeMode = SITL_TIME_REALTIME;
static uint64_t sitlVirtualTimeUs = 0;

void sitlSetTimeModeUdp(uint64_t syncUs)
{
    // Keep the virtual clock continuous with the real-time clock and aligned
    // to the caller's chosen 100 us grid (the scheduler's gyro deadline grid
    // is anchored to a fixed residue mod 100, so stepping on that grid never
    // leaves the scheduler busy-wait spinning on a sub-step remainder).
    sitlVirtualTimeUs = syncUs;
    sitlTimeMode = SITL_TIME_UDP;
}

void sitlSetTimeModeRealtime(void)
{
    sitlTimeMode = SITL_TIME_REALTIME;
}

bool sitlTimeIsUdpDriven(void)
{
    return sitlTimeMode == SITL_TIME_UDP;
}

void sitlStepTime(uint64_t stepUs)
{
    sitlVirtualTimeUs += stepUs;
}

// Official real-time time base. sitl.c's time functions are renamed to
// sitl* by CMakeLists.txt; these wrappers keep the original symbols available
// to the rest of the firmware.
uint64_t micros64(void)
{
    // Realtime mode uses the raw monotonic clock directly. The official
    // sitlMicros64() is scaled by the external simulator's simRate, which is
    // left at a stale value after an Unreal session ends and would otherwise
    // make the "official" 1 kHz lock drift from wall time.
    return sitlTimeMode == SITL_TIME_UDP ? sitlVirtualTimeUs : micros64_real();
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
    // In UDP mode the sim clock is owned by the incoming packet stream;
    // firmware delayMicroseconds() calls must not advance it or sleep the
    // scheduler thread.
    if (sitlTimeMode == SITL_TIME_REALTIME) {
        sitlDelayMicroseconds(us);
    }
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
