/**
 * OSD capture verification for the LOCAL-mode DLL.
 *
 * Boots the flight controller, steps it like a host would, and dumps the OSD
 * character grid the firmware rendered. This is the end-to-end proof that the
 * OSD is enabled, that the displayport wrapper is installed, and that a host can
 * read the exact canvas Betaflight would show.
 *
 * NOTE: everything goes to stderr. sitlBoot() redirects stdout to NUL in LOCAL
 * mode (the DLL runs inside a GUI host), so printf() output would be lost.
 *
 * Usage: sitl_osd_check [steps] [--raw] [--reinit]
 *
 * --reinit repeats the LOCAL-mode host restart (sitl_local_shutdown() +
 * sitl_local_init(), which is what the game's FullResetSitl does inside the
 * same process) and checks that the FC comes back looking like it was just
 * powered up: the boot splash (BF logo + version + "MENU:THR MID" hint) is
 * drawn again from OSD_STATE_INIT. Without that, a restarted FC keeps the
 * previous session's OSD state machine and never redraws the splash.
 */

#include "sitl_local.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#endif

// windows.h already defines a bare OUT macro, so use a distinct name.
#define OSDOUT(...) fprintf(stderr, __VA_ARGS__)

static void dumpFrame(const sitl_local_osd_frame_t *frame)
{
    OSDOUT("canvas %ux%u  video_system=%u  device=%u  active=%u  frames=%u  revision=%u\n",
        frame->cols, frame->rows, frame->videoSystem, frame->displayPortDevice,
        frame->active, frame->frames, frame->revision);

    for (unsigned y = 0; y < frame->rows; y++) {
        OSDOUT("|");
        for (unsigned x = 0; x < frame->cols; x++) {
            const unsigned index = y * SITL_LOCAL_OSD_MAX_COLS + x;
            const unsigned char ch = frame->character[index];
            const unsigned char kind = frame->kind[index];
            if (kind == SITL_LOCAL_OSD_CELL_BLANK) {
                OSDOUT(" ");
            } else if (kind == SITL_LOCAL_OSD_CELL_SYS) {
                OSDOUT("#"); // goggle-drawn system element
            } else if (ch >= 32 && ch < 127) {
                OSDOUT("%c", (int)ch);
            } else {
                OSDOUT(".");
            }
        }
        OSDOUT("|\n");
    }
}

static int countDrawn(const sitl_local_osd_frame_t *frame)
{
    int drawn = 0;
    for (unsigned y = 0; y < frame->rows; y++) {
        for (unsigned x = 0; x < frame->cols; x++) {
            if (frame->kind[y * SITL_LOCAL_OSD_MAX_COLS + x] != SITL_LOCAL_OSD_CELL_BLANK) {
                drawn++;
            }
        }
    }
    return drawn;
}

/** True when `needle` appears as a run of glyph cells inside one canvas row.
 *  This looks for the splash's static text (cms.h's CMS_STARTUP_HELP_TEXT1
 *  "MENU:THR MID"), which no other screen draws. */
static bool frameRowContains(const sitl_local_osd_frame_t *frame, const char *needle)
{
    const size_t len = strlen(needle);
    if (len == 0) {
        return false;
    }
    for (unsigned y = 0; y < frame->rows; y++) {
        for (unsigned x = 0; x + len <= frame->cols; x++) {
            bool match = true;
            for (size_t i = 0; i < len; i++) {
                const unsigned index = y * SITL_LOCAL_OSD_MAX_COLS + (x + i);
                if (frame->kind[index] != SITL_LOCAL_OSD_CELL_CHAR ||
                    frame->character[index] != (uint8_t)needle[i]) {
                    match = false;
                    break;
                }
            }
            if (match) {
                return true;
            }
        }
    }
    return false;
}

/** Glyphs from the logo font (osdDrawLogo() writes font indices 160 upwards).
 *  A splash leaves a solid block of them; a live HUD uses the symbol range a
 *  few dozen times at most. */
static int countLogoCells(const sitl_local_osd_frame_t *frame)
{
    int cells = 0;
    for (unsigned i = 0; i < (unsigned)frame->cols * frame->rows; i++) {
        if (frame->kind[i] == SITL_LOCAL_OSD_CELL_CHAR && frame->character[i] >= 160) {
            cells++;
        }
    }
    return cells;
}

/** Restart the FC inside this process and look for the boot splash in the
 *  seconds that follow. Returns 0 when the splash came back. */
static int checkSplashAfterRestart(sitl_local_input_t *in)
{
    sitl_local_output_t out;

    // Step past this boot's own splash first. Without this the check could pass
    // on a canvas that was simply never touched again: the evidence has to be a
    // frame that lost the splash and then got it back.
    bool splashStillUp = true;
    for (int i = 0; i < 8000; i++) {
        in->rc_channels[4] = 1000;      // ARM switch low: disarmed
        sitl_local_step(in, 1000, &out);
        if ((i % 100) != 0) {
            continue;
        }
        sitl_local_osd_frame_t frame;
        splashStillUp = (sitl_local_get_osd_frame(&frame) == 0) &&
                        frameRowContains(&frame, "MENU:THR MID");
        if (!splashStillUp && i >= 4000) {
            break;                      // the 4 s splash window has expired
        }
    }
    if (splashStillUp) {
        OSDOUT("\nPRE-RESTART: the splash is still on the canvas - restart skipped, "
               "this run cannot tell a redraw from an untouched canvas\n");
        return 2;
    }
    OSDOUT("\nPRE-RESTART: canvas is past the splash (no 'MENU:THR MID')\n");

    OSDOUT("\n=== in-process restart: sitl_local_shutdown() + sitl_local_init() ===\n");
    sitl_local_shutdown();
    if (sitl_local_init() != 0) {
        OSDOUT("sitl_local_init() failed after the restart\n");
        return 1;
    }

    // The splash is drawn once, as soon as the displayport reports ready, and
    // stays up for 4 s. Sample every 20 ms for 2 s of stepping.
    sitl_local_osd_frame_t splashFrame;
    memset(&splashFrame, 0, sizeof(splashFrame));
    bool splashSeen = false;
    uint32_t splashAtMs = 0;
    int splashLogo = 0;
    int splashCells = 0;

    for (int i = 0; i < 2000; i++) {
        in->rc_channels[4] = 1000;      // ARM switch low: disarmed
        sitl_local_step(in, 1000, &out);

        if ((i % 20) != 0) {
            continue;
        }
        sitl_local_osd_frame_t frame;
        if (sitl_local_get_osd_frame(&frame) != 0) {
            continue;
        }
        if (frameRowContains(&frame, "MENU:THR MID")) {
            splashSeen = true;
            splashAtMs = (uint32_t)i;
            splashLogo = countLogoCells(&frame);
            splashCells = countDrawn(&frame);
            memcpy(&splashFrame, &frame, sizeof(splashFrame));
            break;
        }
        // Keep the fullest frame as a fallback for the report.
        if (countDrawn(&frame) > splashCells) {
            splashCells = countDrawn(&frame);
            splashLogo = countLogoCells(&frame);
            memcpy(&splashFrame, &frame, sizeof(splashFrame));
        }
    }

    if (splashSeen) {
        OSDOUT("SPLASH: yes - 'MENU:THR MID' at %u ms after the restart "
               "(%d cells drawn, %d logo glyphs)\n", splashAtMs, splashCells, splashLogo);
        dumpFrame(&splashFrame);
        return 0;
    }

    OSDOUT("SPLASH: NO - no 'MENU:THR MID' in 2 s of stepping. "
           "Fullest frame: %d cells, %d logo glyphs:\n", splashCells, splashLogo);
    dumpFrame(&splashFrame);
    return 1;
}

int main(int argc, char **argv)
{
    int steps = 4000;
    bool raw = false;
    bool reinit = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--raw") == 0) {
            raw = true;
        } else if (strcmp(argv[i], "--reinit") == 0) {
            reinit = true;
        } else {
            steps = atoi(argv[i]);
            if (steps <= 0) {
                steps = 4000;
            }
        }
    }

    OSDOUT("sitl_local_init()...\n");
    if (sitl_local_init() != 0) {
        OSDOUT("sitl_local_init() failed\n");
        return 1;
    }

    sitl_local_input_t in;
    memset(&in, 0, sizeof(in));
    in.orientation_quat[0] = 1.0;
    in.position_xyz[2] = 1.0;
    in.battery_voltage = 16.8;
    in.battery_current = 12.0;
    in.motor_rpm[0] = 12000.0;
    in.motor_rpm[1] = 11000.0;
    in.motor_rpm[2] = 10000.0;
    in.motor_rpm[3] = 9000.0;
    for (int i = 0; i < SITL_LOCAL_MAX_RC_CHANNELS; i++) {
        in.rc_channels[i] = 1500;
    }

    sitl_local_output_t out;
    OSDOUT("stepping %d x 1 ms...\n", steps);

    // Sample the canvas while stepping and keep the frame with the most content.
    // The FC's default config enables only the warnings element, so the steady
    // state is legitimately near-empty; the boot logo is what proves the capture
    // path carries real glyphs end to end.
    sitl_local_osd_frame_t frame;
    sitl_local_osd_frame_t best;
    memset(&frame, 0, sizeof(frame));
    memset(&best, 0, sizeof(best));
    int bestCells = -1;
    uint32_t bestAtStep = 0;

    for (int i = 0; i < steps; i++) {
        // Hold the ARM switch low so the disarmed OSD (the "DISARMED" element and
        // the pre-arm warnings) is what gets rendered.
        in.rc_channels[4] = 1000;
        sitl_local_step(&in, 1000, &out);

        if ((i % 50) == 0) {
            if (sitl_local_get_osd_frame(&frame) == 0) {
                const int drawn = countDrawn(&frame);
                if (drawn > bestCells) {
                    bestCells = drawn;
                    bestAtStep = (uint32_t)i;
                    memcpy(&best, &frame, sizeof(best));
                }
            }
        }
    }
    OSDOUT("peak content: %d cells at step %u\n", bestCells, bestAtStep);

    sitl_local_osd_diag_t diag;
    memset(&diag, 0, sizeof(diag));
    sitl_local_get_osd_diag(&diag);
    OSDOUT("diag: feature=%u dpReady=%u wrapped=%u video=%u device=%u canvas=%ux%u frames=%u rev=%u\n",
        diag.osdFeatureEnabled, diag.displayPortReady, diag.wrapped, diag.videoSystem,
        diag.displayPortDevice, diag.cols, diag.rows, diag.frameCount, diag.revision);
    OSDOUT("diag calls: clear=%u writeString=%u writeChar=%u writeSys=%u acceptedPuts=%u maxCellsAtPublish=%u\n",
        diag.clearCalls, diag.writeStringCalls, diag.writeCharCalls, diag.writeSysCalls,
        diag.acceptedPuts, diag.maxCellsAtPublish);
    OSDOUT("diag sample: x=%u y=%u char=%u\n", diag.sampleX, diag.sampleY, diag.sampleChar);
    OSDOUT("diag gate: storedFeatureMask=0x%08X OSDbit=%u configurationState=%u\n",
        diag.featureMask, diag.osdBitStored, diag.configurationState);
    OSDOUT("diag hook: maskBeforeHook=0x%08X hookEnabledOsd=%u\n",
        diag.storedFeatureMaskBeforeHook, diag.hookEnabledOsd);

    OSDOUT("armed=%d  osd_frames=%u  osd_revision=%u\n",
        (int)sitl_local_get_armed(),
        sitl_local_get_osd_frame_count(),
        sitl_local_get_osd_revision());

    // Dump the frame with the most content, not the final one: with the shipped
    // config the steady-state OSD is nearly empty, so the peak frame is what
    // demonstrates the glyphs actually travelled through the capture path.
    memcpy(&frame, &best, sizeof(frame));
    if (sitl_local_get_osd_frame(&frame) != 0) {
        OSDOUT("sitl_local_get_osd_frame() failed - OSD capture not active\n");
        return 2;
    }
    if (bestCells > 0) {
        memcpy(&frame, &best, sizeof(frame));
    }

    const int drawn = countDrawn(&frame);
    OSDOUT("drawn cells: %d\n", drawn);
    dumpFrame(&frame);

    if (raw) {
        OSDOUT("\n--- raw cell dump (row,col,kind,char,attr) ---\n");
        for (unsigned y = 0; y < frame.rows; y++) {
            for (unsigned x = 0; x < frame.cols; x++) {
                const unsigned index = y * SITL_LOCAL_OSD_MAX_COLS + x;
                if (frame.kind[index] != SITL_LOCAL_OSD_CELL_BLANK) {
                    OSDOUT("r%02u c%02u kind=%u char=%u attr=0x%02X\n",
                        y, x, frame.kind[index], frame.character[index],
                        frame.attribute[index]);
                }
            }
        }
    }

    OSDOUT("\nRESULT: %s\n", drawn > 0 ? "OSD ACTIVE (grid populated)" : "OSD EMPTY");
    if (drawn <= 0) {
        return 3;
    }
    if (reinit) {
        const int splashResult = checkSplashAfterRestart(&in);
        OSDOUT("\nRESULT (restart): %s\n",
            splashResult == 0 ? "SPLASH REDRAWN - the FC looks freshly powered up"
                              : "NO SPLASH - the restarted FC kept its OSD state");
        return splashResult;
    }
    return 0;
}
