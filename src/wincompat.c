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
extern uint64_t sitlMicros64(void);
extern uint64_t sitlMillis64(void);
extern uint32_t sitlMicros(void);
extern uint32_t sitlMillis(void);
extern void sitlDelayMicroseconds(uint32_t us);
extern uint32_t sitlGetCycleCounter(void);

void dyad_update(void)
{
    // The dyad TCP bridge is replaced by serial_tcp_win.c on Windows, so the
    // SITL tcpWorker thread only spins here. With no streams, select() returns
    // immediately on Windows, so dyad_update() would burn a full CPU core.
    // Sleep instead; 10 ms matches the update timeout the SITL would use.
    Sleep(10);
}

// Stepped virtual time, mirroring AJ92/SimITL: time advances by a fixed step
// per scheduler pass instead of tracking the real-time clock. The scheduler's
// gyro busy-wait then exits within a few steps (no spin, no starvation), and
// the gyro/PID loop rate is set by the step size and the run-loop cadence.
// Set BF_SITL_LOW_CPU=0 to use the official real-time time base instead.
static uint64_t sitlVirtualTimeUs = 0;

static bool sitlLowCpuMode(void)
{
    static int cached = -1;
    if (cached < 0) {
        const char *env = getenv("BF_SITL_LOW_CPU");
        cached = (env == NULL || strcmp(env, "0") != 0) ? 1 : 0;
    }
    return cached != 0;
}

void sitlStepTime(uint64_t stepUs)
{
    sitlVirtualTimeUs += stepUs;
}

uint64_t micros64(void)
{
    return sitlLowCpuMode() ? sitlVirtualTimeUs : sitlMicros64();
}

uint32_t micros(void)
{
    return sitlLowCpuMode() ? (uint32_t)(sitlVirtualTimeUs & 0xFFFFFFFF) : sitlMicros();
}

uint64_t millis64(void)
{
    return sitlLowCpuMode() ? sitlVirtualTimeUs / 1000 : sitlMillis64();
}

uint32_t millis(void)
{
    return sitlLowCpuMode() ? (uint32_t)((sitlVirtualTimeUs / 1000) & 0xFFFFFFFF) : sitlMillis();
}

uint32_t getCycleCounter(void)
{
    return sitlLowCpuMode() ? (uint32_t)(sitlVirtualTimeUs & 0xFFFFFFFF) : sitlGetCycleCounter();
}

void delayMicroseconds(uint32_t us)
{
    if (sitlLowCpuMode()) {
        sitlStepTime(us);
    } else {
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
