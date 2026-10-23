/**
 * Proves the UE-side OSD struct mirror is byte-compatible with the DLL's.
 *
 * BfSitlLocalLink.h declares FBfSitlOsdFrame as a hand-written mirror of
 * sitl_local_osd_frame_t and lets the DLL write straight into it. That contract
 * is invisible to the UE build if the two ever drift, and the failure mode is
 * silent corruption, so this test includes BOTH definitions and asserts the
 * layout field by field with offsetof.
 *
 * The UE-side declaration is reproduced verbatim (only the UE typedefs are
 * aliased to standard types), so a mismatch here is a real mismatch in the game
 * module too.
 *
 * Built by CMake as sitl_osd_abi_check; exits non-zero on any mismatch.
 */

#include "sitl_local.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// --- UE typedef aliases (UE: uint8/uint32/int32) ----------------------------
typedef uint8_t uint8;
typedef uint32_t uint32;
typedef int32_t int32;

// --- Verbatim reproduction of the UE-side declaration ----------------------
// Keep in step with Source/FPVSkyline/Public/BfSitl/BfSitlLocalLink.h.
#define BFSITL_OSD_MAX_COLS 64
#define BFSITL_OSD_MAX_ROWS 32
#define BFSITL_OSD_MAX_CELLS (BFSITL_OSD_MAX_COLS * BFSITL_OSD_MAX_ROWS)

// Uses C11 _Static_assert (Betaflight's own STATIC_ASSERT maps to the same
// thing); this TU is compiled as C, so no C++ headers are involved.
#define ABI_ASSERT(cond, name) _Static_assert((cond), #name)

typedef struct
{
	uint8 Cols;
	uint8 Rows;
	uint8 VideoSystem;
	uint8 DisplayPortDevice;
	uint8 bActive;
	uint8 Reserved[3];
	uint32 Revision;
	uint32 Frames;
	uint8 Kind[BFSITL_OSD_MAX_CELLS];
	uint8 Character[BFSITL_OSD_MAX_CELLS];
	uint8 Attribute[BFSITL_OSD_MAX_CELLS];
} FBfSitlOsdFrameUe;

// --- The contract -----------------------------------------------------------
ABI_ASSERT(sizeof(FBfSitlOsdFrameUe) == sizeof(sitl_local_osd_frame_t),
	ueOsdFrameSize);

ABI_ASSERT(offsetof(FBfSitlOsdFrameUe, Cols) == offsetof(sitl_local_osd_frame_t, cols),
	offsCols);
ABI_ASSERT(offsetof(FBfSitlOsdFrameUe, Rows) == offsetof(sitl_local_osd_frame_t, rows),
	offsRows);
ABI_ASSERT(offsetof(FBfSitlOsdFrameUe, VideoSystem) == offsetof(sitl_local_osd_frame_t, videoSystem),
	offsVideoSystem);
ABI_ASSERT(offsetof(FBfSitlOsdFrameUe, DisplayPortDevice) == offsetof(sitl_local_osd_frame_t, displayPortDevice),
	offsDisplayPortDevice);
ABI_ASSERT(offsetof(FBfSitlOsdFrameUe, bActive) == offsetof(sitl_local_osd_frame_t, active),
	offsActive);
ABI_ASSERT(offsetof(FBfSitlOsdFrameUe, Reserved) == offsetof(sitl_local_osd_frame_t, reserved),
	offsReserved);
ABI_ASSERT(offsetof(FBfSitlOsdFrameUe, Revision) == offsetof(sitl_local_osd_frame_t, revision),
	offsRevision);
ABI_ASSERT(offsetof(FBfSitlOsdFrameUe, Frames) == offsetof(sitl_local_osd_frame_t, frames),
	offsFrames);
ABI_ASSERT(offsetof(FBfSitlOsdFrameUe, Kind) == offsetof(sitl_local_osd_frame_t, kind),
	offsKind);
ABI_ASSERT(offsetof(FBfSitlOsdFrameUe, Character) == offsetof(sitl_local_osd_frame_t, character),
	offsCharacter);
ABI_ASSERT(offsetof(FBfSitlOsdFrameUe, Attribute) == offsetof(sitl_local_osd_frame_t, attribute),
	offsAttribute);

// The public canvas bounds must match the firmware's maximum coordinate range
// (OSD X is 6 bits, Y is 5 bits), or a canvas could not be represented.
ABI_ASSERT(BFSITL_OSD_MAX_COLS == SITL_LOCAL_OSD_MAX_COLS, canvasWidth);
ABI_ASSERT(BFSITL_OSD_MAX_ROWS == SITL_LOCAL_OSD_MAX_ROWS, canvasHeight);
ABI_ASSERT(BFSITL_OSD_MAX_CELLS == SITL_LOCAL_OSD_MAX_CELLS, cellCount);

// The cell-kind enum the renderer switches on must match the DLL's.
ABI_ASSERT((int)SITL_LOCAL_OSD_CELL_BLANK == 0, cellKindBlank);
ABI_ASSERT((int)SITL_LOCAL_OSD_CELL_CHAR == 1, cellKindChar);
ABI_ASSERT((int)SITL_LOCAL_OSD_CELL_SYS == 2, cellKindSys);

// Video-system values, used to pick the display aspect ratio.
ABI_ASSERT((int)SITL_LOCAL_OSD_VIDEO_AUTO == 0, videoAuto);
ABI_ASSERT((int)SITL_LOCAL_OSD_VIDEO_PAL == 1, videoPal);
ABI_ASSERT((int)SITL_LOCAL_OSD_VIDEO_NTSC == 2, videoNtsc);
ABI_ASSERT((int)SITL_LOCAL_OSD_VIDEO_HD == 3, videoHd);

int main(void)
{
	printf("UE OSD frame size      : %u\n", (unsigned)sizeof(FBfSitlOsdFrameUe));
	printf("DLL OSD frame size     : %u\n", (unsigned)sizeof(sitl_local_osd_frame_t));
	printf("offset revision        : UE %u / DLL %u\n",
		(unsigned)offsetof(FBfSitlOsdFrameUe, Revision), (unsigned)offsetof(sitl_local_osd_frame_t, revision));
	printf("offset kind plane      : UE %u / DLL %u\n",
		(unsigned)offsetof(FBfSitlOsdFrameUe, Kind), (unsigned)offsetof(sitl_local_osd_frame_t, kind));
	printf("offset character plane : UE %u / DLL %u\n",
		(unsigned)offsetof(FBfSitlOsdFrameUe, Character), (unsigned)offsetof(sitl_local_osd_frame_t, character));
	printf("offset attribute plane : UE %u / DLL %u\n",
		(unsigned)offsetof(FBfSitlOsdFrameUe, Attribute), (unsigned)offsetof(sitl_local_osd_frame_t, attribute));
	printf("RESULT: ABI MATCH\n");
	return 0;
}
