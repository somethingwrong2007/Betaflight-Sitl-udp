/**
 * Small POSIX compatibility shims for the MinGW build of Betaflight SITL.
 */

#include <string.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#include "platform.h"
#include "drivers/io.h"
#include "drivers/serial.h"

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
