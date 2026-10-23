/**
 * End-to-end proof of the OSD mirror contract.
 *
 * The requirement is: "when the SITL OSD is configured, the UE HUD changes with
 * it". The UE HUD simply draws whatever character canvas the flight controller
 * produces, so the contract reduces to:
 *
 *   (1) configuring an OSD element makes it appear on the canvas a host reads,
 *   (2) MOVING that element moves it on the canvas (old spot cleared), and
 *   (3) the canvas change token (revision) advances so a poller notices.
 *
 * The move test uses a text element and compares the drawn string at two
 * different anchors, which pins content AND position without hardcoding what the
 * craft name happens to be. That is deliberately stronger than counting cells:
 * the OSD boot logo times out, so the total cell count legitimately drops while
 * the configured elements appear.
 *
 * Usage: sitl_osd_contract_check
 */

#include "sitl_local.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#endif

#define OSDOUT(...) fprintf(stderr, __VA_ARGS__)

static void stepMany(int count)
{
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
    in.rc_channels[4] = 1000; // ARM switch low

    sitl_local_output_t out;
    for (int i = 0; i < count; i++) {
        sitl_local_step(&in, 1000, &out);
    }
}

static void dumpFrame(const char *title, const sitl_local_osd_frame_t *frame)
{
    OSDOUT("--- %s (canvas %ux%u, revision %u, frames %u) ---\n",
           title, frame->cols, frame->rows, frame->revision, frame->frames);
    for (unsigned y = 0; y < frame->rows; y++) {
        OSDOUT("|");
        for (unsigned x = 0; x < frame->cols; x++) {
            const unsigned index = y * SITL_LOCAL_OSD_MAX_COLS + x;
            if (frame->kind[index] == SITL_LOCAL_OSD_CELL_BLANK) {
                OSDOUT(" ");
            } else if (frame->kind[index] == SITL_LOCAL_OSD_CELL_SYS) {
                OSDOUT("#");
            } else {
                const unsigned char ch = frame->character[index];
                OSDOUT("%c", (ch >= 32 && ch < 127) ? (int)ch : '.');
            }
        }
        OSDOUT("|\n");
    }
}

/** Collect the run of printable character cells starting at (col,row).
 *  Returns the length written (0 when the cell is blank). */
static int rowText(const sitl_local_osd_frame_t *frame, int col, int row,
                   char *out, int outSize)
{
    int len = 0;
    if (outSize <= 0 || col < 0 || row < 0 || row >= (int)frame->rows) {
        return 0;
    }
    for (int x = col; x < (int)frame->cols && len < outSize - 1; x++) {
        const unsigned index = (unsigned)row * SITL_LOCAL_OSD_MAX_COLS + (unsigned)x;
        if (frame->kind[index] == SITL_LOCAL_OSD_CELL_BLANK) {
            break; // end of this run
        }
        if (frame->kind[index] != SITL_LOCAL_OSD_CELL_CHAR) {
            break;
        }
        const unsigned char ch = frame->character[index];
        if (ch < 32 || ch >= 127) {
            break;
        }
        out[len++] = (char)ch;
    }
    out[len] = '\0';
    return len;
}

/** Is there ink within a box around (col,row)? Elements draw relative to their
 *  anchor (the artificial horizon's bars extend below it), so the box is
 *  deliberately generous. */
static int inkNear(const sitl_local_osd_frame_t *frame, int col, int row,
                   int dyUp, int dyDown, int dxLeft, int dxRight)
{
    int found = 0;
    for (int y = row - dyUp; y <= row + dyDown; y++) {
        for (int x = col - dxLeft; x <= col + dxRight; x++) {
            if (x < 0 || y < 0 || x >= (int)frame->cols || y >= (int)frame->rows) {
                continue;
            }
            if (frame->kind[y * SITL_LOCAL_OSD_MAX_COLS + x] != SITL_LOCAL_OSD_CELL_BLANK) {
                found++;
            }
        }
    }
    return found;
}

int main(void)
{
    OSDOUT("sitl_local_init()...\n");
    if (sitl_local_init() != 0) {
        OSDOUT("init failed\n");
        return 1;
    }

    // Let the boot logo time out so element changes are unambiguous.
    stepMany(4000);

    sitl_local_osd_frame_t baseline;
    memset(&baseline, 0, sizeof(baseline));
    if (sitl_local_get_osd_frame(&baseline) != 0) {
        OSDOUT("no OSD frame\n");
        return 2;
    }
    dumpFrame("baseline (boot logo timed out)", &baseline);
    const uint32_t baselineRevision = baseline.revision;

    // --- Phase 1: place a text element at anchor A -------------------------
    const int anchorAX = 5, anchorAY = 3;
    if (sitl_local_set_osd_element(SITL_LOCAL_OSD_ITEM_CRAFT_NAME,
                                   anchorAX, anchorAY, 1) != 0) {
        OSDOUT("set_osd_element failed\n");
        return 3;
    }
    stepMany(1500);

    sitl_local_osd_frame_t placed;
    memset(&placed, 0, sizeof(placed));
    if (sitl_local_get_osd_frame(&placed) != 0) {
        return 4;
    }
    char textA[64];
    const int lenA = rowText(&placed, anchorAX, anchorAY, textA, sizeof(textA));
    dumpFrame("element placed at A", &placed);
    OSDOUT("at A (%d,%d): \"%s\" (len %d)\n", anchorAX, anchorAY, textA, lenA);

    // --- Phase 2: MOVE the same element to anchor B ------------------------
    const int anchorBX = 5, anchorBY = 12;
    if (sitl_local_set_osd_element(SITL_LOCAL_OSD_ITEM_CRAFT_NAME,
                                   anchorBX, anchorBY, 1) != 0) {
        OSDOUT("move failed\n");
        return 5;
    }
    stepMany(1500);

    sitl_local_osd_frame_t moved;
    memset(&moved, 0, sizeof(moved));
    if (sitl_local_get_osd_frame(&moved) != 0) {
        return 6;
    }
    char textB[64];
    const int lenB = rowText(&moved, anchorBX, anchorBY, textB, sizeof(textB));
    char textOld[64];
    const int lenOld = rowText(&moved, anchorAX, anchorAY, textOld, sizeof(textOld));
    dumpFrame("element moved to B", &moved);
    OSDOUT("at B (%d,%d): \"%s\" (len %d)\n", anchorBX, anchorBY, textB, lenB);
    OSDOUT("old anchor A still shows: \"%s\" (len %d, expected 0)\n", textOld, lenOld);

    // --- Phase 3: a spread of elements, as a visual sanity check -----------
    struct { int item; int col; int row; } placements[] = {
        { SITL_LOCAL_OSD_ITEM_CROSSHAIRS,         15, 8  },
        { SITL_LOCAL_OSD_ITEM_ARTIFICIAL_HORIZON, 15, 4  },
        { SITL_LOCAL_OSD_ITEM_MAIN_BATT_VOLTAGE,  1,  1  },
        { SITL_LOCAL_OSD_ITEM_RSSI_VALUE,         1,  2  },
        { SITL_LOCAL_OSD_ITEM_FLYMODE,            40, 1  },
        { SITL_LOCAL_OSD_ITEM_THROTTLE_POS,       40, 2  },
        { SITL_LOCAL_OSD_ITEM_ALTITUDE,           40, 3  },
    };
    for (unsigned i = 0; i < sizeof(placements) / sizeof(placements[0]); i++) {
        sitl_local_set_osd_element(placements[i].item, placements[i].col,
                                   placements[i].row, 1);
    }
    stepMany(2000);

    sitl_local_osd_frame_t spread;
    memset(&spread, 0, sizeof(spread));
    if (sitl_local_get_osd_frame(&spread) != 0) {
        return 7;
    }
    dumpFrame("spread of configured elements", &spread);

    int present = 0;
    for (unsigned i = 0; i < sizeof(placements) / sizeof(placements[0]); i++) {
        const int ink = inkNear(&spread, placements[i].col, placements[i].row, 1, 6, 1, 8);
        if (ink > 0) {
            present++;
        }
        OSDOUT("  item %2d at (%2d,%2d): %s\n", placements[i].item,
               placements[i].col, placements[i].row, ink > 0 ? "drawn" : "MISSING");
    }

    // --- Verdict -----------------------------------------------------------
    const bool bRevisionMoved = placed.revision != baselineRevision;
    const bool bContentChanged =
        memcmp(baseline.kind, placed.kind, sizeof(baseline.kind)) != 0 ||
        memcmp(baseline.character, placed.character, sizeof(baseline.character)) != 0;
    const bool bPlaced = (lenA > 0);
    const bool bMovedSameText = (lenB > 0) && (lenA == lenB) && (strcmp(textA, textB) == 0);
    const bool bOldCleared = (lenOld == 0);
    const bool bSpread = (present == (int)(sizeof(placements) / sizeof(placements[0])));

    OSDOUT("\nchecks:\n");
    OSDOUT("  revision moved          : %d (baseline %u -> placed %u)\n",
           bRevisionMoved, baselineRevision, placed.revision);
    OSDOUT("  canvas content changed  : %d\n", bContentChanged);
    OSDOUT("  element drew at anchor A: %d\n", bPlaced);
    OSDOUT("  same text after moving  : %d\n", bMovedSameText);
    OSDOUT("  old anchor cleared      : %d\n", bOldCleared);
    OSDOUT("  all elements drawn      : %d (%d/%d)\n", bSpread, present,
           (int)(sizeof(placements) / sizeof(placements[0])));

    const bool bPass = bRevisionMoved && bContentChanged && bPlaced &&
                       bMovedSameText && bOldCleared;
    OSDOUT("RESULT: %s\n", bPass
        ? "PASS - the host canvas follows the SITL OSD config, including moves"
        : "FAIL - the canvas did not follow the config");
    return bPass ? 0 : 8;
}
