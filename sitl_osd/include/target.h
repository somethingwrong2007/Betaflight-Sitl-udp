/*
 * SITL OSD enablement shim (shadow target.h).
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * The SITL target disables the OSD, but not by an explicit "#undef USE_OSD":
 * in extern/betaflight/src/main/target/common_pre.h the whole classic-build
 * block (lines 104-319, which contains "#define USE_OSD" at line 244 and
 * "#define USE_MAX7456" at line 193) is wrapped in
 *
 *     #if !defined(CLOUD_BUILD) && !defined(SITL)
 *
 * and the SITL build defines SITL. So the OSD macros are never defined in the
 * first place, and target.h's later "#undef USE_OSD" is simply a no-op.
 *
 * common_pre.h runs BEFORE target.h (see src/main/platform.h: line 37 includes
 * target/common_pre.h, line 42 includes "target.h", line 43 includes
 * target/common_post.h), so a shim placed on the include path ahead of the real
 * target directory can re-open the OSD gates here, and common_post.h - which
 * runs afterwards - then sees USE_OSD defined and behaves exactly as it does on
 * a normal OSD-capable target.
 *
 * This mirrors the layering the wrapper repo already uses for wincompat/ (which
 * shadows an ARM-only build/atomic.h). The pinned extern/betaflight submodule is
 * deliberately NOT modified.
 *
 * NOTE ON USE_MAX7456: it is intentionally NOT enabled. The MAX7456 driver needs
 * SPI, and the SITL target does "#undef USE_SPI". The HD/MSP displayport path
 * (USE_MSP_DISPLAYPORT) is the one we want: with USE_OSD_HD defined,
 * pgResetFn_vcdProfile() defaults video_system to VIDEO_SYSTEM_HD, and
 * init.c then selects OSD_DISPLAYPORT_DEVICE_MSP.
 */

#pragma once

/* Pull in the real target.h from the next directory on the include path. */
#include_next "target.h"

#ifndef USE_OSD

/* Core OSD gate. */
#define USE_OSD

/* Video system support: HD (MSP displayport, 53x20 default canvas) and SD
 * (30x16). Both are defined so the canvas follows whatever the FC is set to. */
#define USE_OSD_SD
#define USE_OSD_HD

/* Canvas abstraction used by the OSD element renderer. */
#define USE_CANVAS

/* CMS: init.c only offers the OSD_DISPLAYPORT_DEVICE_MSP case when
 * USE_CMS && USE_MSP_DISPLAYPORT && USE_OSD_OVER_MSP_DISPLAYPORT are all set. */
#define USE_CMS
#define USE_CMS_FAILSAFE_MENU
#define USE_EXTENDED_CMS_MENUS

/* The actual transport: Betaflight's MSP displayport (MSP_DISPLAYPORT / 182),
 * which is what digital goggles and the configurator's OSD preview consume. */
#define USE_MSP_DISPLAYPORT
#define USE_OSD_OVER_MSP_DISPLAYPORT

/* Optional OSD feature packs that a normal OSD target gets. */
#define USE_OSD_ADJUSTMENTS
#define USE_OSD_PROFILES
#define USE_OSD_STICK_OVERLAY
#define USE_OSD_CUSTOM_TEXT
/* osd_custom_text.c's own guard is USE_OSD_CUSTOM_TEXT, but the init/update
 * call sites in fc/init.c and fc/tasks.c are guarded by the ENABLE_ form. Without
 * this the file compiles to dead code. */
#define ENABLE_OSD_CUSTOM_TEXT 1
#define USE_OSD_QUICK_MENU
#define USE_RC_STATS
#define USE_SPEC_PREARM_SCREEN

/* Mirrors common_pre.h's USE_GPS-conditional CMS menu. */
#if defined(USE_GPS) && !defined(USE_CMS_GPS_RESCUE_MENU)
#define USE_CMS_GPS_RESCUE_MENU
#endif

#endif /* USE_OSD */
