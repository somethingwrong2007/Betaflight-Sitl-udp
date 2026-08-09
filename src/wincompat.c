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
extern uint32_t sitlGetCycleCounter(void);

void dyad_update(void)
{
    // The dyad TCP bridge is replaced by serial_tcp_win.c on Windows, so the
    // SITL tcpWorker thread only spins here. With no streams, select() returns
    // immediately on Windows, so dyad_update() would burn a full CPU core.
    // Sleep instead; 10 ms matches the update timeout the SITL would use.
    Sleep(10);
}

uint32_t getCycleCounter(void)
{
    // The SITL scheduler spin-waits on the cycle counter for the next gyro
    // tick, which burns a full CPU core even when no flight simulator is
    // attached. Sleeping on every third call turns that busy loop (and the
    // scheduler's own bookkeeping) into a ~1 ms poll without starving the
    // low-priority serial/MSP task. The gyro/PID period is widened to 5 ms in
    // main_windows.c so the widened schedule stays stable. Set
    // BF_SITL_LOW_CPU=0 for the official behavior.
    const char *lowCpu = getenv("BF_SITL_LOW_CPU");
    if (lowCpu == NULL || strcmp(lowCpu, "0") != 0) {
        static uint32_t callCount = 0;
        if ((++callCount % 3) == 0) {
            Sleep(1);
        }
    }
    return sitlGetCycleCounter();
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
