/**
 * Checks whether enabling the OSD at boot mutates the stored EEPROM.
 *
 * sitlOsdEnableFeature() calls featureEnableImmediate(FEATURE_OSD), which writes
 * featureConfigMutable()->enabledFeatures and marks the config dirty. If the FC
 * then persists that, simply booting the game would silently switch FEATURE_OSD
 * on in the user's aircraft config.
 *
 * This harness boots the FC against a COPY of a given EEPROM and reports whether
 * that copy changed, so the behaviour is a measured fact rather than an
 * assumption.
 *
 * Usage: sitl_osd_eeprom_effect <path-to-eeprom-copy>
 */

#include "sitl_local.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#endif

#define OSDOUT(...) fprintf(stderr, __VA_ARGS__)

static unsigned char *readFile(const char *path, long *sizeOut)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char *data = (unsigned char *)malloc((size_t)size);
    if (data && fread(data, 1, (size_t)size, f) != (size_t)size) {
        free(data);
        data = NULL;
    }
    fclose(f);
    *sizeOut = size;
    return data;
}

static unsigned long hashOf(const unsigned char *data, long size)
{
    // FNV-1a: enough to detect any change.
    unsigned long h = 2166136261u;
    for (long i = 0; i < size; i++) {
        h ^= data[i];
        h *= 16777619u;
    }
    return h;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        OSDOUT("usage: %s <eeprom-copy>\n", argv[0]);
        return 2;
    }
    const char *path = argv[1];

    long sizeBefore = 0;
    unsigned char *before = readFile(path, &sizeBefore);
    if (!before) {
        OSDOUT("cannot read %s\n", path);
        return 2;
    }
    const unsigned long hashBefore = hashOf(before, sizeBefore);
    OSDOUT("eeprom before boot: %ld bytes, hash=%08lX\n", sizeBefore, hashBefore);

    // Point the FC at this file *before* booting.
    if (sitl_local_set_eeprom_path(path) != 0) {
        OSDOUT("set_eeprom_path failed\n");
        return 3;
    }

    if (sitl_local_init() != 0) {
        OSDOUT("init failed\n");
        return 4;
    }

    // Step long enough for the OSD to run and for any deferred EEPROM flush.
    sitl_local_input_t in;
    memset(&in, 0, sizeof(in));
    in.orientation_quat[0] = 1.0;
    in.position_xyz[2] = 1.0;
    in.battery_voltage = 16.8;
    for (int i = 0; i < SITL_LOCAL_MAX_RC_CHANNELS; i++) {
        in.rc_channels[i] = 1500;
    }
    in.rc_channels[4] = 1000;

    sitl_local_output_t out;
    for (int i = 0; i < 3000; i++) {
        sitl_local_step(&in, 1000, &out);
    }

    // Give the background thread a moment to flush a deferred save.
    Sleep(1500);

    long sizeAfter = 0;
    unsigned char *after = readFile(path, &sizeAfter);
    if (!after) {
        OSDOUT("cannot re-read %s\n", path);
        return 5;
    }
    const unsigned long hashAfter = hashOf(after, sizeAfter);
    OSDOUT("eeprom after boot : %ld bytes, hash=%08lX\n", sizeAfter, hashAfter);

    // Report the OSD state the FC is running with.
    sitl_local_osd_diag_t diag;
    memset(&diag, 0, sizeof(diag));
    sitl_local_get_osd_diag(&diag);
    OSDOUT("osd: feature=%u wrapped=%u canvas=%ux%u frames=%u\n",
           diag.osdFeatureEnabled, diag.wrapped, diag.cols, diag.rows, diag.frameCount);

    if (hashBefore == hashAfter && sizeBefore == sizeAfter) {
        OSDOUT("RESULT: EEPROM UNCHANGED (enabling the OSD is not persisted)\n");
        return 0;
    }

    OSDOUT("RESULT: EEPROM CHANGED\n");
    if (sizeBefore == sizeAfter) {
        int diffs = 0;
        for (long i = 0; i < sizeBefore; i++) {
            if (before[i] != after[i]) {
                if (diffs < 20) {
                    OSDOUT("  0x%04lX: before=0x%02X after=0x%02X\n",
                           i, before[i], after[i]);
                }
                diffs++;
            }
        }
        OSDOUT("  %d differing byte(s)\n", diffs);
    }
    return 1;
}
