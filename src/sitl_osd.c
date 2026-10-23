/**
 * sitl_osd.c - in-process capture of the flight controller's OSD canvas.
 *
 * WHY
 * ---
 * The Unreal HUD has to mirror the Betaflight OSD exactly: same element
 * positions, same warnings, same font, same canvas size. The only way to get
 * that for free is to let the *firmware itself* render the OSD and read the
 * resulting character grid back, instead of re-deriving the layout on the host.
 *
 * HOW
 * ---
 * Betaflight renders the OSD into a character canvas and pushes it through a
 * "displayport" (drivers/display.h's displayPortVTable_t). With the OSD enabled
 * and vcd_video_system == HD, fc/init.c picks OSD_DISPLAYPORT_DEVICE_MSP and
 * calls displayPortMspInit() from io/displayport_msp.c.
 *
 * This file wraps that displayport: the real MSP vtable is kept (so the
 * configurator / goggle stream is untouched) and a thin wrapper vtable is
 * installed in its place. The wrapper mirrors every clear/write/draw call into
 * a character grid and publishes that grid on MSP_DP_DRAW_SCREEN - i.e. exactly
 * the frame a digital VTX would display.
 *
 * This is the same layering SimITL uses (see .simitl-ref's
 * betaflightext/src/main/io/displayport_fake.c, which replaces
 * max7456DisplayPortInit with a buffer-backed displayport). Wrapping rather than
 * replacing is deliberate here: the pinned extern/betaflight submodule stays
 * untouched, and the MSP stream keeps flowing so the Configurator still sees a
 * live OSD.
 *
 * The canvas size is taken from the displayport the firmware built, so the host
 * follows whatever vcd_video_system / osd_canvas_* the FC is configured with
 * (HD 53x20, PAL 30x16, ...) rather than assuming one layout.
 */

#ifdef SITL_LOCAL

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

#ifdef USE_OSD

#include "config/feature.h"
#include "config/config.h"
#include "fc/controlrate_profile.h"

#include "drivers/display.h"
// OSD_CHAR_VISIBLE_BYTES and the full osdCharacter_t (display.h only forward
// declares the struct).
#include "drivers/osd.h"

#include "io/displayport_msp.h"

#include "osd/osd.h"

#include "pg/vcd.h"

#include "sitl_local.h"

#include <windows.h>

// The real io/displayport_msp.c entry point, renamed by CMake so this file can
// provide the displayPortMspInit() the firmware calls.
extern displayPort_t *sitlRealDisplayPortMspInit(void);

extern void sitlAuditLog(const char *fmt, ...);

// ---------------------------------------------------------------------------
// Captured canvas
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t kind;       // sitl_local_osd_cell_kind_e
    uint8_t character;  // MAX7456 font index, or displayPortSystemElement_e
    uint8_t attribute;  // displayPortSeverity_e | DISPLAYPORT_BLINK
} osdCell_t;

// What the firmware is drawing right now (the "foreground" layer).
static osdCell_t gLive[SITL_LOCAL_OSD_MAX_ROWS][SITL_LOCAL_OSD_MAX_COLS];
// The last committed frame, i.e. what the host reads.
static sitl_local_osd_frame_t gPublished;

static uint8_t gCols = 0;
static uint8_t gRows = 0;
static uint32_t gRevision = 0;
static uint32_t gFrameCount = 0;
// The feature mask as the EEPROM left it, captured before sitlOsdEnableFeature()
// touches anything. Without this the diag would report the post-hook mask and
// could not tell "the config already had the OSD" from "this hook enabled it".
static uint32_t gStoredFeatureMaskBeforeHook = 0;
static uint8_t gHookEnabledOsd = 0;
// Diagnostics: how often the firmware actually drove the displayport.
static uint32_t gClearCalls = 0;
static uint32_t gWriteStringCalls = 0;
static uint32_t gWriteCharCalls = 0;
static uint32_t gWriteSysCalls = 0;
// How many puts were accepted (in-bounds) and how many cells were non-blank at
// the moment a frame was published. Distinguishes "the firmware never drew
// anything" from "content was drawn but cleared before the commit".
static uint32_t gAcceptedPuts = 0;
static uint32_t gMaxCellsAtPublish = 0;
static uint8_t gSampleX = 0, gSampleY = 0, gSampleC = 0;
static uint32_t gSampleCount = 0;

static uint32_t osdCountLive(void)
{
    uint32_t count = 0;
    for (uint8_t y = 0; y < gRows; y++) {
        for (uint8_t x = 0; x < gCols; x++) {
            if (gLive[y][x].kind != SITL_LOCAL_OSD_CELL_BLANK) {
                count++;
            }
        }
    }
    return count;
}

static const displayPortVTable_t *gRealVTable = NULL;
static bool gActive = false;

// The grid is written by the thread stepping the FC (scheduler TASK_OSD inside
// sitl_local_step) and read by the host's game thread, so publishing and reading
// are serialized. A CRITICAL_SECTION is enough: the host reads at most once per
// rendered frame.
static CRITICAL_SECTION gOsdCrit;
static volatile LONG gOsdCritReady = 0;

static void osdEnsureLock(void)
{
    if (InterlockedCompareExchange(&gOsdCritReady, 1, 0) == 0) {
        InitializeCriticalSection(&gOsdCrit);
    }
}

// ---------------------------------------------------------------------------
// Captured font
//
// When the configurator flashes a font it sends one MSP_OSD_CHAR_WRITE per
// character (256 of them, payload = 1-byte character address + the 54 visible
// bytes) and displayWriteFontCharacter() forwards each one to the displayport.
// The last complete font is cached here so a host can render the OSD with the
// font the user actually flashed instead of its own .mcm fallback.
//
// Only OSD_CHAR_VISIBLE_BYTES (54) per character are kept: the firmware hands
// the driver an OSD_CHAR_BYTES (64) buffer whose last 10 bytes come from an
// uninitialized local in the MSP handler, and the configurator never sends
// them - caching 64 would make "did the font change" depend on stack garbage.
// ---------------------------------------------------------------------------

#define OSD_FONT_CHAR_COUNT 256
#define OSD_FONT_CHAR_BYTES OSD_CHAR_VISIBLE_BYTES

// The host API advertises 54 bytes per character (see
// sitl_local_get_osd_font()), so a firmware that changes the character format
// must be reflected there as well.
STATIC_ASSERT(OSD_FONT_CHAR_BYTES == 54, osdFontCharBytes);

static uint8_t gFontBytes[OSD_FONT_CHAR_COUNT * OSD_FONT_CHAR_BYTES];
static uint32_t gFontSeenBitmap[OSD_FONT_CHAR_COUNT / 32];
static unsigned gFontSeenCount;
static uint32_t gFontRevision;

// ---------------------------------------------------------------------------
// Captured font, on disk: <EEPROM directory>\osd_font.bin
//
// The configurator hands the glyphs over live (MSP_OSD_CHAR_WRITE, one
// character per frame), so without a file the font dies with the process -
// restarting the game or switching aircraft would mean flashing it again.
// Keeping it next to the virtual EEPROM makes it per-aircraft for free: the
// host points BF_SITL_EEPROM at Content/Drone/<craft>/eeprom.bin, so the font
// travels with that aircraft's config (and with a copy of its directory).
//
// Why a sidecar file and not a trailer inside eeprom.bin: the EEPROM is a
// fixed-size structure plus a version byte, so 13.8 KB in there would either
// need a version bump (which resets every stored config) or a trailer that the
// whole save/restore path (FullResetSitl templates, save-and-reboot, the
// EEPROM tools) would have to carry along. A separate file has none of that
// coupling.
// ---------------------------------------------------------------------------

#define OSD_FONT_FILE "osd_font.bin"

// Path the currently cached font belongs to ("" = none yet).
static char gFontPath[MAX_PATH];
// Set when the live font was just completed and needs writing out; the file I/O
// is done by the host-side getters, never inside the flight loop.
static int gFontPendingSave;
// Scratch for the save, so the copy can happen under the lock and the write
// without it.
static uint8_t gFontSaveScratch[sizeof(gFontBytes)];

STATIC_ASSERT(sizeof(gFontBytes) == 13824, osdFontBytes);   // 256 * 54

// Walk up from the virtual EEPROM path and append OSD_FONT_FILE. Empty result
// means "no EEPROM path set yet", which disables persistence.
static void osdFontPath(char *out, size_t size)
{
    const char *eeprom = getenv("BF_SITL_EEPROM");
    const char *slash = NULL;
    const char *p;

    out[0] = '\0';
    if (eeprom == NULL || eeprom[0] == '\0' || size < 2) {
        return;
    }
    for (p = eeprom; *p != '\0'; p++) {
        if (*p == '\\' || *p == '/') {
            slash = p;
        }
    }
    if (slash == NULL) {
        return;                            // no directory part: nothing to do
    }
    const size_t dirLen = (size_t)(slash - eeprom) + 1;
    if (dirLen + sizeof(OSD_FONT_FILE) > size) {
        return;
    }
    memcpy(out, eeprom, dirLen);
    memcpy(out + dirLen, OSD_FONT_FILE, sizeof(OSD_FONT_FILE));
}

static void osdFontWriteFile(const char *path, const uint8_t *bytes)
{
    FILE *fp = fopen(path, "wb");
    if (fp == NULL) {
        sitlAuditLog("osd: cannot write font %s", path);
        return;
    }
    const size_t written = fwrite(bytes, 1, sizeof(gFontBytes), fp);
    fclose(fp);
    if (written != sizeof(gFontBytes)) {
        sitlAuditLog("osd: short font write to %s (%u of %u bytes)",
                     path, (unsigned)written, (unsigned)sizeof(gFontBytes));
        return;
    }
    sitlAuditLog("osd: font saved to %s (%u bytes)", path, (unsigned)sizeof(gFontBytes));
}

// Adopt a stored font as the live one; the revision bump is what tells a host
// "there is a font now, use it".
static void osdFontLoadFile(const char *path)
{
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        return;                            // no font for this aircraft (yet)
    }
    const size_t got = fread(gFontSaveScratch, 1, sizeof(gFontSaveScratch), fp);
    fclose(fp);
    if (got != sizeof(gFontSaveScratch)) {
        sitlAuditLog("osd: stored font %s is not %u bytes (%u), ignored",
                     path, (unsigned)sizeof(gFontSaveScratch), (unsigned)got);
        return;
    }
    EnterCriticalSection(&gOsdCrit);
    memcpy(gFontBytes, gFontSaveScratch, sizeof(gFontBytes));
    memset(gFontSeenBitmap, 0xFF, sizeof(gFontSeenBitmap));
    gFontSeenCount = OSD_FONT_CHAR_COUNT;
    gFontRevision++;
    LeaveCriticalSection(&gOsdCrit);
    sitlAuditLog("osd: font loaded from %s (revision %u)", path, (unsigned)gFontRevision);
}

// Keep the cached font in sync with the aircraft the EEPROM points at: flush a
// font that was just flashed, and swap in the stored one when the path moved.
// Called from the host-facing getters - it does file I/O, so it must never run
// inside sitl_local_step().
static void osdFontSyncWithPath(void)
{
    char newPath[MAX_PATH];
    char oldPath[MAX_PATH];
    bool savePending = false;
    bool pathChanged = false;
    bool loadNew = false;

    osdEnsureLock();
    EnterCriticalSection(&gOsdCrit);

    if (gFontPendingSave) {
        gFontPendingSave = 0;
        memcpy(gFontSaveScratch, gFontBytes, sizeof(gFontSaveScratch));
        memcpy(oldPath, gFontPath, sizeof(oldPath));
        savePending = true;
    }
    osdFontPath(newPath, sizeof(newPath));
    if (strcmp(newPath, gFontPath) != 0) {
        pathChanged = true;
        memcpy(gFontPath, newPath, sizeof(gFontPath));
        // A font belongs to one aircraft: drop the live one, then try to load
        // whatever the new directory has.
        memset(gFontSeenBitmap, 0, sizeof(gFontSeenBitmap));
        gFontSeenCount = 0;
        loadNew = (newPath[0] != '\0');
    }
    LeaveCriticalSection(&gOsdCrit);

    if (savePending && oldPath[0] != '\0') {
        // Save where the font was flashed *from*: if the path changed in the
        // same call, those glyphs still belong to the previous aircraft.
        osdFontWriteFile(oldPath, gFontSaveScratch);
    }
    if (pathChanged) {
        if (loadNew) {
            osdFontLoadFile(newPath);
        } else {
            sitlAuditLog("osd: no EEPROM path, font persistence off (live font dropped)");
        }
    }
}

static void osdClearLive(void)
{
    memset(gLive, 0, sizeof(gLive));
}

static void osdPut(uint8_t x, uint8_t y, uint8_t kind, uint8_t character, uint8_t attribute)
{
    if (x >= gCols || y >= gRows) {
        return;
    }
    osdCell_t *cell = &gLive[y][x];
    cell->kind = kind;
    cell->character = character;
    cell->attribute = attribute;
    gAcceptedPuts++;
    if (gSampleCount < 8) {
        gSampleX = x;
        gSampleY = y;
        gSampleC = character;
        gSampleCount++;
    }
}

// Commit the live grid into the published frame. Called from drawScreen, i.e.
// once per OSD refresh the firmware decides to show.
static void osdPublish(void)
{
    osdEnsureLock();
    EnterCriticalSection(&gOsdCrit);

    gPublished.cols = gCols;
    gPublished.rows = gRows;
    gPublished.videoSystem = (uint8_t)vcdProfile()->video_system;
    gPublished.active = 1;
    gPublished.frames = gFrameCount;
    gPublished.revision = gRevision;

    const uint32_t liveCells = osdCountLive();
    if (liveCells > gMaxCellsAtPublish) {
        gMaxCellsAtPublish = liveCells;
    }

    for (uint8_t y = 0; y < gRows; y++) {
        for (uint8_t x = 0; x < gCols; x++) {
            const uint32_t index = (uint32_t)y * SITL_LOCAL_OSD_MAX_COLS + x;
            gPublished.kind[index] = gLive[y][x].kind;
            gPublished.character[index] = gLive[y][x].character;
            gPublished.attribute[index] = gLive[y][x].attribute;
        }
    }

    gRevision++;
    gPublished.revision = gRevision;

    LeaveCriticalSection(&gOsdCrit);
}

// ---------------------------------------------------------------------------
// Wrapper vtable: mirror into the grid, then delegate to the real MSP driver.
// ---------------------------------------------------------------------------

static int wrapGrab(displayPort_t *dp)
{
    return gRealVTable->grab ? gRealVTable->grab(dp) : 0;
}

static int wrapRelease(displayPort_t *dp)
{
    return gRealVTable->release ? gRealVTable->release(dp) : 0;
}

static int wrapClearScreen(displayPort_t *dp, displayClearOption_e options)
{
    gClearCalls++;
    osdClearLive();
    return gRealVTable->clearScreen ? gRealVTable->clearScreen(dp, options) : 0;
}

static bool wrapDrawScreen(displayPort_t *dp)
{
    // Publish before delegating: the real driver only pushes bytes to the MSP
    // port, it does not change the canvas.
    osdPublish();
    gFrameCount++;
    return gRealVTable->drawScreen ? gRealVTable->drawScreen(dp) : false;
}

static int wrapScreenSize(const displayPort_t *dp)
{
    return gRealVTable->screenSize ? gRealVTable->screenSize(dp) : (int)gCols * (int)gRows;
}

static int wrapWriteSys(displayPort_t *dp, uint8_t x, uint8_t y, displayPortSystemElement_e systemElement)
{
    gWriteSysCalls++;
    // System elements (goggle voltage, bitrate, ...) are drawn by the VTX, not
    // sent as characters. Record them so the host can show the same slot.
    osdPut(x, y, SITL_LOCAL_OSD_CELL_SYS, (uint8_t)systemElement, DISPLAYPORT_SEVERITY_NORMAL);
    return gRealVTable->writeSys ? gRealVTable->writeSys(dp, x, y, systemElement) : 0;
}

static int wrapWriteString(displayPort_t *dp, uint8_t x, uint8_t y, uint8_t attr, const char *text)
{
    gWriteStringCalls++;
    if (text) {
        for (int i = 0; text[i] != '\0'; i++) {
            const uint8_t col = (uint8_t)(x + i);
            if (col >= gCols) {
                break; // matches the real driver, which clamps to the canvas
            }
            osdPut(col, y, SITL_LOCAL_OSD_CELL_CHAR, (uint8_t)text[i], attr);
        }
    }
    return gRealVTable->writeString ? gRealVTable->writeString(dp, x, y, attr, text) : 0;
}

static int wrapWriteChar(displayPort_t *dp, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    gWriteCharCalls++;
    osdPut(x, y, SITL_LOCAL_OSD_CELL_CHAR, c, attr);
    return gRealVTable->writeChar ? gRealVTable->writeChar(dp, x, y, attr, c) : 0;
}

static bool wrapIsTransferInProgress(const displayPort_t *dp)
{
    return gRealVTable->isTransferInProgress ? gRealVTable->isTransferInProgress(dp) : false;
}

static int wrapHeartbeat(displayPort_t *dp)
{
    return gRealVTable->heartbeat ? gRealVTable->heartbeat(dp) : 0;
}

static void wrapRedraw(displayPort_t *dp)
{
    if (gRealVTable->redraw) {
        gRealVTable->redraw(dp);
    }
}

static bool wrapIsSynced(const displayPort_t *dp)
{
    return gRealVTable->isSynced ? gRealVTable->isSynced(dp) : true;
}

static uint32_t wrapTxBytesFree(const displayPort_t *dp)
{
    return gRealVTable->txBytesFree ? gRealVTable->txBytesFree(dp) : UINT32_MAX;
}

static bool wrapLayerSupported(displayPort_t *dp, displayPortLayer_e layer)
{
    return gRealVTable->layerSupported ? gRealVTable->layerSupported(dp, layer) : false;
}

static bool wrapLayerSelect(displayPort_t *dp, displayPortLayer_e layer)
{
    return gRealVTable->layerSelect ? gRealVTable->layerSelect(dp, layer) : false;
}

static bool wrapLayerCopy(displayPort_t *dp, displayPortLayer_e dest, displayPortLayer_e src)
{
    return gRealVTable->layerCopy ? gRealVTable->layerCopy(dp, dest, src) : false;
}

static bool wrapWriteFontCharacter(displayPort_t *dp, uint16_t addr, const struct osdCharacter_s *chr)
{
    const bool cached = (chr != NULL && addr < OSD_FONT_CHAR_COUNT);
    if (cached) {
        osdEnsureLock();
        EnterCriticalSection(&gOsdCrit);
        // The configurator's upload walks the characters from 0 upwards, so
        // address 0 marks the start of a new font: drop the previous one there
        // instead of mixing a half-uploaded font with the old one. (While the
        // upload is running sitl_local_get_osd_font() reports "no font".)
        if (addr == 0) {
            memset(gFontSeenBitmap, 0, sizeof(gFontSeenBitmap));
            gFontSeenCount = 0;
            sitlAuditLog("osd: font upload started (address 0)");
        }
        memcpy(&gFontBytes[(unsigned)addr * OSD_FONT_CHAR_BYTES], chr->data, OSD_FONT_CHAR_BYTES);
        const uint32_t bit = 1u << (addr & 31u);
        if ((gFontSeenBitmap[addr >> 5] & bit) == 0) {
            gFontSeenBitmap[addr >> 5] |= bit;
            gFontSeenCount++;
        }
        gFontRevision++;
        if (gFontSeenCount == OSD_FONT_CHAR_COUNT) {
            if (gFontPendingSave == 0) {
                // Persist it next to the aircraft's EEPROM. The write itself
                // happens in the host-facing getters: a file write here would
                // be the MSP thread blocking on disk.
                gFontPendingSave = 1;
                sitlAuditLog("osd: font upload complete (%u characters, revision %u)",
                             (unsigned)gFontSeenCount, gFontRevision);
            }
        }
        LeaveCriticalSection(&gOsdCrit);
    }

    // Offer it to the real driver too, but never let that decide whether the
    // configurator's upload succeeds.
    //
    // In this build the MSP displayport's font path is not compiled
    // (displayport_msp.c guards it with USE_MSP_DISPLAYPORT_FONT, which the
    // SITL target does not define), so the real entry is NULL - and `output()`
    // would have no VTX serial port to push MSP_DP_FONTCHAR_WRITE to anyway.
    // Reporting that as a failure would answer MSP_RESULT_ERROR for every
    // character, and the configurator aborts the flash right there ("font
    // upload failed"). The SITL has no goggles to forward the font to: the host
    // renders the OSD itself, from the copy cached above.
    if (gRealVTable != NULL && gRealVTable->writeFontCharacter != NULL) {
        (void)gRealVTable->writeFontCharacter(dp, addr, chr);
    }
    return cached;
}

static bool wrapCheckReady(displayPort_t *dp, bool rescan)
{
    return gRealVTable->checkReady ? gRealVTable->checkReady(dp, rescan) : true;
}

static void wrapBeginTransaction(displayPort_t *dp, displayTransactionOption_e opts)
{
    if (gRealVTable->beginTransaction) {
        gRealVTable->beginTransaction(dp, opts);
    }
}

static void wrapCommitTransaction(displayPort_t *dp)
{
    if (gRealVTable->commitTransaction) {
        gRealVTable->commitTransaction(dp);
    }
}

static bool wrapGetCanvas(struct displayCanvas_s *canvas, const displayPort_t *dp)
{
    return gRealVTable->getCanvas ? gRealVTable->getCanvas(canvas, dp) : false;
}

static void wrapSetBackgroundType(displayPort_t *dp, displayPortBackground_e backgroundType)
{
    if (gRealVTable->setBackgroundType) {
        gRealVTable->setBackgroundType(dp, backgroundType);
    }
}

static bool wrapDrawOsdItem(displayPort_t *dp, uint8_t x, uint8_t y, uint8_t item, bool isBackground)
{
    return gRealVTable->drawOsdItem ? gRealVTable->drawOsdItem(dp, x, y, item, isBackground) : false;
}

static void wrapRedrawBackground(displayPort_t *dp)
{
    if (gRealVTable->redrawBackground) {
        gRealVTable->redrawBackground(dp);
    }
}

static void wrapFontUpdateCompletion(displayPort_t *dp)
{
    if (gRealVTable->fontUpdateCompletion) {
        gRealVTable->fontUpdateCompletion(dp);
    }
}

static const displayPortVTable_t gOsdVTable = {
    .grab = wrapGrab,
    .release = wrapRelease,
    .clearScreen = wrapClearScreen,
    .drawScreen = wrapDrawScreen,
    .screenSize = wrapScreenSize,
    .writeSys = wrapWriteSys,
    .writeString = wrapWriteString,
    .writeChar = wrapWriteChar,
    .isTransferInProgress = wrapIsTransferInProgress,
    .heartbeat = wrapHeartbeat,
    .redraw = wrapRedraw,
    .isSynced = wrapIsSynced,
    .txBytesFree = wrapTxBytesFree,
    .layerSupported = wrapLayerSupported,
    .layerSelect = wrapLayerSelect,
    .layerCopy = wrapLayerCopy,
    .writeFontCharacter = wrapWriteFontCharacter,
    .checkReady = wrapCheckReady,
    .beginTransaction = wrapBeginTransaction,
    .commitTransaction = wrapCommitTransaction,
    .getCanvas = wrapGetCanvas,
    .setBackgroundType = wrapSetBackgroundType,
    .drawOsdItem = wrapDrawOsdItem,
    .redrawBackground = wrapRedrawBackground,
    .fontUpdateCompletion = wrapFontUpdateCompletion,
};

// ---------------------------------------------------------------------------
// Firmware entry point (replaces io/displayport_msp.c's displayPortMspInit)
// ---------------------------------------------------------------------------

displayPort_t *displayPortMspInit(void)
{
    displayPort_t *dp = sitlRealDisplayPortMspInit();
    if (dp == NULL) {
        return NULL;
    }

    // The firmware may re-init the displayport on a config reload / cold restart.
    // displayInit() restores the real vtable first, so only wrap when we are not
    // already installed - otherwise gRealVTable would point at our own table and
    // every call would recurse.
    if (dp->vTable != &gOsdVTable) {
        gRealVTable = dp->vTable;
        dp->vTable = &gOsdVTable;
    }

    gCols = dp->cols;
    gRows = dp->rows;
    if (gCols == 0 || gCols > SITL_LOCAL_OSD_MAX_COLS) {
        gCols = (uint8_t)OSD_SD_COLS;
    }
    if (gRows == 0 || gRows > SITL_LOCAL_OSD_MAX_ROWS) {
        gRows = (uint8_t)OSD_SD_ROWS;
    }

    osdClearLive();
    osdEnsureLock();

    EnterCriticalSection(&gOsdCrit);
    memset(&gPublished, 0, sizeof(gPublished));
    gPublished.cols = gCols;
    gPublished.rows = gRows;
    gPublished.videoSystem = (uint8_t)vcdProfile()->video_system;
    gPublished.displayPortDevice = (uint8_t)OSD_DISPLAYPORT_DEVICE_MSP;
    gPublished.active = 1;
    gPublished.revision = ++gRevision;
    LeaveCriticalSection(&gOsdCrit);

    gActive = true;
    sitlAuditLog("osd: displayport wrapped, canvas %ux%u video_system=%u",
                 (unsigned)gCols, (unsigned)gRows,
                 (unsigned)vcdProfile()->video_system);
    return dp;
}

// ---------------------------------------------------------------------------
// Host API
// ---------------------------------------------------------------------------

int sitl_local_get_osd_frame(sitl_local_osd_frame_t *out)
{
    if (out == NULL || !gActive) {
        return -1;
    }
    osdEnsureLock();
    EnterCriticalSection(&gOsdCrit);
    memcpy(out, &gPublished, sizeof(*out));
    LeaveCriticalSection(&gOsdCrit);
    return 0;
}

uint32_t sitl_local_get_osd_revision(void)
{
    if (!gActive) {
        return 0;
    }
    osdEnsureLock();
    EnterCriticalSection(&gOsdCrit);
    const uint32_t revision = gPublished.revision;
    LeaveCriticalSection(&gOsdCrit);
    return revision;
}

uint32_t sitl_local_get_osd_frame_count(void)
{
    return gFrameCount;
}

// Revision of the captured font the configurator flashed, or 0 while no
// complete font has been uploaded (i.e. during an upload, and until the first
// one ever). A host polls this and re-reads the font when it changes.
uint32_t sitl_local_get_osd_font_revision(void)
{
    osdEnsureLock();
    osdFontSyncWithPath();
    EnterCriticalSection(&gOsdCrit);
    const uint32_t revision = (gFontSeenCount == OSD_FONT_CHAR_COUNT) ? gFontRevision : 0;
    LeaveCriticalSection(&gOsdCrit);
    return revision;
}

// Copy the captured font: OSD_FONT_CHAR_COUNT (256) characters of
// OSD_FONT_CHAR_BYTES (54) bytes, character-major. Returns the number of bytes
// written, or 0 when there is no complete font yet / the buffer is too small.
int sitl_local_get_osd_font(char *out, int size)
{
    if (out == NULL || size < (int)sizeof(gFontBytes)) {
        return 0;
    }
    osdEnsureLock();
    osdFontSyncWithPath();
    EnterCriticalSection(&gOsdCrit);
    // Checked under the lock: a new upload (which clears the bitmap at address
    // 0) cannot start while the copy runs, so `out` is never a half-old,
    // half-new mixture.
    const bool complete = (gFontSeenCount == OSD_FONT_CHAR_COUNT);
    if (complete) {
        memcpy(out, gFontBytes, sizeof(gFontBytes));
    }
    LeaveCriticalSection(&gOsdCrit);
    return complete ? (int)sizeof(gFontBytes) : 0;
}

// Diagnostic: why is (or isn't) the OSD running? Used by tools/sitl_osd_check.
void sitl_local_get_osd_diag(sitl_local_osd_diag_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->osdFeatureEnabled = featureIsEnabled(FEATURE_OSD) ? 1 : 0;
    out->displayPortReady = (osdGetDisplayPort(NULL) != NULL) ? 1 : 0;
    out->wrapped = gActive ? 1 : 0;
    out->cols = gCols;
    out->rows = gRows;
    out->videoSystem = (uint8_t)vcdProfile()->video_system;
    out->displayPortDevice = (uint8_t)osdConfig()->displayPortDevice;
    out->frameCount = gFrameCount;
    out->revision = gRevision;
    out->osdTaskEnabled = 0; // filled by the caller from the scheduler, if needed
    out->clearCalls = gClearCalls;
    out->writeStringCalls = gWriteStringCalls;
    out->writeCharCalls = gWriteCharCalls;
    out->writeSysCalls = gWriteSysCalls;
    out->acceptedPuts = gAcceptedPuts;
    out->maxCellsAtPublish = gMaxCellsAtPublish;
    out->sampleX = gSampleX;
    out->sampleY = gSampleY;
    out->sampleChar = gSampleC;
    // Ground truth for the feature gate: the live mask, the stored mask and the
    // board state, so a host can tell "already enabled in the config" from
    // "enabled by sitlOsdEnableFeature()".
    out->featureMask = featureConfig()->enabledFeatures;
    out->configurationState = systemConfig()->configurationState;
    out->osdBitStored = (featureConfig()->enabledFeatures & FEATURE_OSD) ? 1 : 0;
    // The mask the EEPROM itself carried, before any host-side enable.
    out->storedFeatureMaskBeforeHook = gStoredFeatureMaskBeforeHook;
    out->hookEnabledOsd = gHookEnabledOsd;
}

// Called from the LOCAL boot path (main_windows.c) after initPhase1 and before
// initPhase3, which is where fc/init.c checks featureIsEnabled(FEATURE_OSD) and
// fc/tasks.c enables TASK_OSD. The stored EEPROM's feature mask wins over
// DEFAULT_FEATURES, so a config saved before the OSD existed would otherwise
// leave the OSD compiled in but switched off at runtime.
// The spec pre-arm screen (osd_elements.c osdDrawSpec(), compiled because this
// build enables USE_SPEC_PREARM_SCREEN) formats
//
//     tfp_sprintf(buff, "THR LIMIT %s",
//                 lookupTableThrottleLimitType[currentControlRateProfile->throttle_limit_type]);
//
// and lookupTableThrottleLimitType[] has exactly THROTTLE_LIMIT_TYPE_COUNT (3)
// entries. Nothing range-checks the stored uint8_t - the CLI's parameter dump and
// the CMS index the same table - so a config file written by another build (or
// hand-edited) can hold a value >= 3. That turns the "%s" into a wild-pointer
// read, and because tfp_sprintf() does not bound its output, into a stack
// overwrite - drawn while the aircraft sits disarmed, i.e. exactly when the
// pilot is trying to arm. Clamp every stored profile once, at boot, before
// anything can read it.
static void sitlOsdSanitizeThrottleLimitType(void)
{
    for (int i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        const uint8_t type = controlRateProfiles(i)->throttle_limit_type;
        if (type >= THROTTLE_LIMIT_TYPE_COUNT) {
            controlRateProfilesMutable(i)->throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF;
            sitlAuditLog("osd: rate profile %d had throttle_limit_type=%u (only %u exist), "
                         "clamped to OFF", i, (unsigned)type,
                         (unsigned)THROTTLE_LIMIT_TYPE_COUNT);
        }
    }
}

void sitlOsdEnableFeature(void)
{
    sitlOsdSanitizeThrottleLimitType();
    gStoredFeatureMaskBeforeHook = featureConfig()->enabledFeatures;

    // Diagnostic escape hatch: BF_SITL_OSD=0 boots with the OSD compiled in but
    // switched off, which is how the "no OSD without this hook" behaviour is
    // verified (tools/sitl_osd_check reports feature/dpReady/wrapped).
    const char *env = getenv("BF_SITL_OSD");
    if (env != NULL && env[0] == '0') {
        sitlAuditLog("osd: FEATURE_OSD left disabled (BF_SITL_OSD=0), stored mask 0x%08X",
                     (unsigned)gStoredFeatureMaskBeforeHook);
        return;
    }

    if (!featureIsEnabled(FEATURE_OSD)) {
        featureEnableImmediate(FEATURE_OSD);
        gHookEnabledOsd = 1;
        sitlAuditLog("osd: FEATURE_OSD enabled at boot (stored mask 0x%08X lacked the OSD bit)",
                     (unsigned)gStoredFeatureMaskBeforeHook);
    } else {
        sitlAuditLog("osd: FEATURE_OSD already set in the stored config (mask 0x%08X)",
                     (unsigned)gStoredFeatureMaskBeforeHook);
    }
}

// --- OSD element configuration ---------------------------------------------
//
// These write the same osdElementConfig record MSP_SET_OSD_CONFIG writes, so a
// host can configure the OSD with the same result as the Configurator's OSD tab.
// The change is applied to the running OSD immediately: osdAnalyzeActiveElements()
// rebuilds the list of elements to draw, which is what makes the next rendered
// canvas reflect the new layout.

// The public item indices must match the firmware's osd_items_e exactly,
// otherwise a host would silently configure the wrong element. These checks fail
// the build if a Betaflight update ever renumbers them. (osd_items_e has no
// typedef name, so the constants are compared directly.)
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_RSSI_VALUE == OSD_RSSI_VALUE, osdItemRssi);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_MAIN_BATT_VOLTAGE == OSD_MAIN_BATT_VOLTAGE, osdItemVbat);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_CROSSHAIRS == OSD_CROSSHAIRS, osdItemCrosshairs);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_ARTIFICIAL_HORIZON == OSD_ARTIFICIAL_HORIZON, osdItemAh);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_HORIZON_SIDEBARS == OSD_HORIZON_SIDEBARS, osdItemAhSbar);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_TIMER_1 == OSD_ITEM_TIMER_1, osdItemTim1);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_TIMER_2 == OSD_ITEM_TIMER_2, osdItemTim2);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_FLYMODE == OSD_FLYMODE, osdItemFlymode);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_CRAFT_NAME == OSD_CRAFT_NAME, osdItemCraftName);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_THROTTLE_POS == OSD_THROTTLE_POS, osdItemThrottle);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_VTX_CHANNEL == OSD_VTX_CHANNEL, osdItemVtx);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_CURRENT_DRAW == OSD_CURRENT_DRAW, osdItemCurrent);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_MAH_DRAWN == OSD_MAH_DRAWN, osdItemMah);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_GPS_SPEED == OSD_GPS_SPEED, osdItemGpsSpeed);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_GPS_SATS == OSD_GPS_SATS, osdItemGpsSats);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_ALTITUDE == OSD_ALTITUDE, osdItemAltitude);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_WARNINGS == OSD_WARNINGS, osdItemWarnings);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_AVG_CELL_VOLTAGE == OSD_AVG_CELL_VOLTAGE, osdItemCellV);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_GPS_LON == OSD_GPS_LON, osdItemGpsLon);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_GPS_LAT == OSD_GPS_LAT, osdItemGpsLat);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_PITCH_ANGLE == OSD_PITCH_ANGLE, osdItemPitchAngle);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_ROLL_ANGLE == OSD_ROLL_ANGLE, osdItemRollAngle);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_DISARMED == OSD_DISARMED, osdItemDisarmed);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_HOME_DIR == OSD_HOME_DIR, osdItemHomeDir);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_HOME_DIST == OSD_HOME_DIST, osdItemHomeDist);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_NUMERICAL_HEADING == OSD_NUMERICAL_HEADING, osdItemHeading);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_NUMERICAL_VARIO == OSD_NUMERICAL_VARIO, osdItemVario);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_COMPASS_BAR == OSD_COMPASS_BAR, osdItemCompassBar);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_ESC_TMP == OSD_ESC_TMP, osdItemEscTmp);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_ESC_RPM == OSD_ESC_RPM, osdItemEscRpm);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_REMAINING_TIME_ESTIMATE == OSD_REMAINING_TIME_ESTIMATE, osdItemRemTime);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_RTC_DATETIME == OSD_RTC_DATETIME, osdItemRtc);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_ADJUSTMENT_RANGE == OSD_ADJUSTMENT_RANGE, osdItemAdjRange);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_CORE_TEMPERATURE == OSD_CORE_TEMPERATURE, osdItemCoreTemp);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_ANTI_GRAVITY == OSD_ANTI_GRAVITY, osdItemAntiGrav);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_G_FORCE == OSD_G_FORCE, osdItemGForce);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_MOTOR_DIAG == OSD_MOTOR_DIAG, osdItemMotorDiag);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_LOG_STATUS == OSD_LOG_STATUS, osdItemLogStatus);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_FLIP_ARROW == OSD_FLIP_ARROW, osdItemFlipArrow);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_LINK_QUALITY == OSD_LINK_QUALITY, osdItemLq);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_FLIGHT_DIST == OSD_FLIGHT_DIST, osdItemFlightDist);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_PILOT_NAME == OSD_PILOT_NAME, osdItemPilotName);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_RATE_PROFILE_NAME == OSD_RATE_PROFILE_NAME, osdItemRateName);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_PID_PROFILE_NAME == OSD_PID_PROFILE_NAME, osdItemPidName);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_PROFILE_NAME == OSD_PROFILE_NAME, osdItemProfileName);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_RSSI_DBM_VALUE == OSD_RSSI_DBM_VALUE, osdItemRssiDbm);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_RC_CHANNELS == OSD_RC_CHANNELS, osdItemRcChannels);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_CAMERA_FRAME == OSD_CAMERA_FRAME, osdItemCamFrame);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_EFFICIENCY == OSD_EFFICIENCY, osdItemEfficiency);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_TOTAL_FLIGHTS == OSD_TOTAL_FLIGHTS, osdItemTotalFlights);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_UP_DOWN_REFERENCE == OSD_UP_DOWN_REFERENCE, osdItemUpDown);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_TX_UPLINK_POWER == OSD_TX_UPLINK_POWER, osdItemTxPower);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_WATT_HOURS_DRAWN == OSD_WATT_HOURS_DRAWN, osdItemWh);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_AUX_VALUE == OSD_AUX_VALUE, osdItemAux);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_READY_MODE == OSD_READY_MODE, osdItemReadyMode);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_RSNR_VALUE == OSD_RSNR_VALUE, osdItemRsnr);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_SYS_GOGGLE_VOLTAGE == OSD_SYS_GOGGLE_VOLTAGE, osdItemSysGogV);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_SYS_VTX_VOLTAGE == OSD_SYS_VTX_VOLTAGE, osdItemSysVtxV);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_SYS_BITRATE == OSD_SYS_BITRATE, osdItemSysBitrate);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_SYS_DELAY == OSD_SYS_DELAY, osdItemSysDelay);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_SYS_DISTANCE == OSD_SYS_DISTANCE, osdItemSysDist);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_SYS_LQ == OSD_SYS_LQ, osdItemSysLq);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_SYS_GOGGLE_DVR == OSD_SYS_GOGGLE_DVR, osdItemSysGogDvr);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_SYS_VTX_DVR == OSD_SYS_VTX_DVR, osdItemSysVtxDvr);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_SYS_WARNINGS == OSD_SYS_WARNINGS, osdItemSysWarn);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_SYS_VTX_TEMP == OSD_SYS_VTX_TEMP, osdItemSysVtxTemp);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_SYS_FAN_SPEED == OSD_SYS_FAN_SPEED, osdItemSysFan);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_GPS_LAP_TIME_CURRENT == OSD_GPS_LAP_TIME_CURRENT, osdItemLapCur);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_GPS_LAP_TIME_PREVIOUS == OSD_GPS_LAP_TIME_PREVIOUS, osdItemLapPrev);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_GPS_LAP_TIME_BEST3 == OSD_GPS_LAP_TIME_BEST3, osdItemLapBest3);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_DEBUG2 == OSD_DEBUG2, osdItemDebug2);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_CUSTOM_MSG0 == OSD_CUSTOM_MSG0, osdItemMsg0);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_CUSTOM_MSG1 == OSD_CUSTOM_MSG1, osdItemMsg1);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_CUSTOM_MSG2 == OSD_CUSTOM_MSG2, osdItemMsg2);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_CUSTOM_MSG3 == OSD_CUSTOM_MSG3, osdItemMsg3);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_LIDAR_DIST == OSD_LIDAR_DIST, osdItemLidar);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_CUSTOM_SERIAL_TEXT == OSD_CUSTOM_SERIAL_TEXT, osdItemSerialText);
STATIC_ASSERT(SITL_LOCAL_OSD_ITEM_BATTERY_PROFILE_NAME == OSD_BATTERY_PROFILE_NAME, osdItemBattProfileName);

int sitl_local_set_osd_element(int item, int col, int row, int visible)
{
    if (!gActive || item < 0 || item >= OSD_ITEM_COUNT) {
        return -1;
    }

    // OSD_POS() packs x/y with the profile/type bits; out-of-range coordinates
    // are clamped the same way osdInit() clamps them.
    uint8_t x = (uint8_t)(col < 0 ? 0 : (col >= gCols ? (gCols > 0 ? gCols - 1 : 0) : col));
    uint8_t y = (uint8_t)(row < 0 ? 0 : (row >= gRows ? (gRows > 0 ? gRows - 1 : 0) : row));

    uint16_t value = OSD_POS(x, y);
    if (visible) {
        // Mark the element visible in every OSD profile.
        for (unsigned profile = 1; profile <= OSD_PROFILE_COUNT; profile++) {
            value |= OSD_PROFILE_FLAG(profile);
        }
    }

    osdElementConfigMutable()->item_pos[item] = value;
    osdAnalyzeActiveElements();
    return 0;
}

int sitl_local_get_osd_element(int item, int *col, int *row, int *visible)
{
    if (!gActive || item < 0 || item >= OSD_ITEM_COUNT) {
        return -1;
    }

    const uint16_t value = osdElementConfig()->item_pos[item];
    if (col) { *col = OSD_X(value); }
    if (row) { *row = OSD_Y(value); }
    if (visible) { *visible = VISIBLE(value) ? 1 : 0; }
    return 0;
}

// Force the next rendered frame to rebuild from the current configuration.
void sitl_local_osd_refresh(void)
{
    if (gActive) {
        osdAnalyzeActiveElements();
    }
}

#else // !USE_OSD

#include "sitl_local.h"

int sitl_local_get_osd_frame(sitl_local_osd_frame_t *out)
{
    (void)out;
    return -1;
}

uint32_t sitl_local_get_osd_revision(void)
{
    return 0;
}

uint32_t sitl_local_get_osd_frame_count(void)
{
    return 0;
}

void sitl_local_get_osd_diag(sitl_local_osd_diag_t *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
}

void sitlOsdEnableFeature(void)
{
}

#endif // USE_OSD

#endif // SITL_LOCAL
