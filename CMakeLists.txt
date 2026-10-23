cmake_minimum_required(VERSION 3.10)
project(betaflight-sitl-udp C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Platform flags
# =============================================================================
set(SITL_LINK_MODE "UDP" CACHE STRING "SITL link mode: UDP (network server) or LOCAL (in-process DLL API)")

if(WIN32)
  message(STATUS "Building for Windows (MinGW)")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DWIN32 -D_WINDOWS")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc")
else()
  message(STATUS "Building for Linux")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -Wall -Wextra -Wno-unused-parameter")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections -fno-common")
if(WIN32)
  # Keep LTO for link-time dead-code elimination, but do not LTO-inline dyad:
  # MinGW's LTO miscompiles dyad's vector/listener code and corrupts the heap.
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -flto=auto -fuse-linker-plugin")
  set_source_files_properties("${BF_ROOT}/lib/main/dyad/dyad.c" PROPERTIES COMPILE_OPTIONS "-fno-lto")
else()
  # LTO is required on Linux to let the linker drop unused references that the
  # official build removes at link time.
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -flto=auto -fuse-linker-plugin -ffast-math -fmerge-all-constants")
endif()

if(SITL_LINK_MODE STREQUAL "LOCAL")
  # A MinGW DLL keeps more sections alive than the exe (export/import
  # bookkeeping), which makes LTO + --gc-sections drop definitions while
  # retaining references to target-only helpers (dma, mpu, dshot). Build the
  # library without LTO; per-function sections still let --gc-sections trim it.
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-lto")
endif()

# =============================================================================
# Betaflight SITL source files
# Extracted from official Makefile for TARGET=SITL
# =============================================================================
set(BF_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/extern/betaflight")

set(BF_SOURCES
  # Entry point
  ${BF_ROOT}/src/main/main.c

  # Build / Version
  ${BF_ROOT}/src/main/build/build_config.c
  ${BF_ROOT}/src/main/build/version.c
  ${BF_ROOT}/src/main/build/debug.c

  # Common
  ${BF_ROOT}/src/main/common/maths.c
  ${BF_ROOT}/src/main/common/filter.c
  ${BF_ROOT}/src/main/common/crc.c
  ${BF_ROOT}/src/main/common/streambuf.c
  ${BF_ROOT}/src/main/common/printf.c
  ${BF_ROOT}/src/main/common/printf_serial.c
  ${BF_ROOT}/src/main/common/time.c
  ${BF_ROOT}/src/main/common/typeconversion.c
  ${BF_ROOT}/src/main/common/encoding.c
  ${BF_ROOT}/src/main/common/bitarray.c
  ${BF_ROOT}/src/main/common/vector.c
  ${BF_ROOT}/src/main/common/gps_conversion.c
  ${BF_ROOT}/src/main/common/sensor_alignment.c
  ${BF_ROOT}/src/main/common/string_light.c

  # Drivers
  ${BF_ROOT}/src/main/drivers/accgyro/accgyro_virtual.c
  ${BF_ROOT}/src/main/drivers/accgyro/gyro_sync.c
  ${BF_ROOT}/src/main/drivers/barometer/barometer_virtual.c
  ${BF_ROOT}/src/main/drivers/compass/compass_virtual.c
  ${BF_ROOT}/src/main/drivers/timer_common.c
  ${BF_ROOT}/src/main/drivers/motor.c
  ${BF_ROOT}/src/main/drivers/bus.c
  ${BF_ROOT}/src/main/drivers/dshot.c
  ${BF_ROOT}/src/main/drivers/serial.c
  ${BF_ROOT}/src/main/drivers/serial_tcp.c
  ${BF_ROOT}/src/main/drivers/buf_writer.c
  ${BF_ROOT}/src/main/drivers/stack_check.c
  ${BF_ROOT}/src/main/drivers/io.c
  ${BF_ROOT}/src/main/drivers/display.c
  ${BF_ROOT}/src/main/drivers/light_led.c
  ${BF_ROOT}/src/main/drivers/rx/rx_pwm.c
  ${BF_ROOT}/src/main/drivers/pinio.c
  ${BF_ROOT}/src/main/drivers/resource.c
  ${BF_ROOT}/src/main/drivers/bus_i2c_busdev.c
  ${BF_ROOT}/src/main/drivers/bus_i2c_soft.c
  ${BF_ROOT}/src/main/drivers/vtx_common.c
  ${BF_ROOT}/src/main/drivers/vtx_table.c
  ${BF_ROOT}/src/main/drivers/pwm_output.c

  # Target - SITL (official UDP server, moved to src/platform/SIMULATOR)
  ${BF_ROOT}/src/platform/SIMULATOR/sitl.c
  ${BF_ROOT}/src/platform/SIMULATOR/udplink.c

  # FC
  ${BF_ROOT}/src/main/fc/init.c
  ${BF_ROOT}/src/main/fc/core.c
  ${BF_ROOT}/src/main/fc/runtime_config.c
  ${BF_ROOT}/src/main/fc/rc_controls.c
  ${BF_ROOT}/src/main/fc/tasks.c
  ${BF_ROOT}/src/main/fc/rc_modes.c
  ${BF_ROOT}/src/main/fc/controlrate_profile.c
  ${BF_ROOT}/src/main/fc/rc.c
  ${BF_ROOT}/src/main/fc/rc_adjustments.c
  ${BF_ROOT}/src/main/fc/board_info.c
  ${BF_ROOT}/src/main/fc/stats.c
  ${BF_ROOT}/src/main/fc/dispatch.c

  # Flight
  ${BF_ROOT}/src/main/flight/failsafe.c
  ${BF_ROOT}/src/main/flight/mixer.c
  ${BF_ROOT}/src/main/flight/mixer_init.c
  ${BF_ROOT}/src/main/flight/mixer_tricopter.c
  ${BF_ROOT}/src/main/flight/servos.c
  ${BF_ROOT}/src/main/flight/servos_tricopter.c
  ${BF_ROOT}/src/main/flight/imu.c
  ${BF_ROOT}/src/main/flight/pid.c
  ${BF_ROOT}/src/main/flight/pid_init.c
  ${BF_ROOT}/src/main/flight/position.c
  ${BF_ROOT}/src/main/flight/gps_rescue_multirotor.c
  ${BF_ROOT}/src/main/flight/autopilot_multirotor.c

  # Sensors
  ${BF_ROOT}/src/main/sensors/initialisation.c
  ${BF_ROOT}/src/main/sensors/compass.c
  ${BF_ROOT}/src/main/sensors/acceleration.c
  ${BF_ROOT}/src/main/sensors/acceleration_init.c
  ${BF_ROOT}/src/main/sensors/gyro.c
  ${BF_ROOT}/src/main/sensors/gyro_init.c
  ${BF_ROOT}/src/main/sensors/boardalignment.c
  ${BF_ROOT}/src/main/sensors/barometer.c
  ${BF_ROOT}/src/main/sensors/voltage.c
  ${BF_ROOT}/src/main/sensors/current.c
  ${BF_ROOT}/src/main/sensors/adcinternal.c
  ${BF_ROOT}/src/main/sensors/esc_sensor.c

  # RX
  ${BF_ROOT}/src/main/rx/rx.c
  ${BF_ROOT}/src/main/rx/msp.c
  ${BF_ROOT}/src/main/rx/msp_override.c
  ${BF_ROOT}/src/main/rx/rc_stats.c

  # IO
  ${BF_ROOT}/src/main/io/beeper.c
  ${BF_ROOT}/src/main/io/statusindicator.c
  ${BF_ROOT}/src/main/io/rcdevice_cam.c
  ${BF_ROOT}/src/main/io/rcdevice.c
  ${BF_ROOT}/src/main/io/gps.c
  ${BF_ROOT}/src/main/io/displayport_msp.c
  ${BF_ROOT}/src/main/io/serial_resource.c
  ${BF_ROOT}/src/main/io/gps_virtual.c

  # MSP
  ${BF_ROOT}/src/main/msp/msp.c
  ${BF_ROOT}/src/main/msp/msp_box.c
  ${BF_ROOT}/src/main/msp/msp_serial.c
  ${BF_ROOT}/src/main/msp/msp_build_info.c

  # CLI
  ${BF_ROOT}/src/main/cli/cli.c
  ${BF_ROOT}/src/main/cli/settings.c

  # OSD
  ${BF_ROOT}/src/main/osd/osd.c
  ${BF_ROOT}/src/main/osd/osd_elements.c
  ${BF_ROOT}/src/main/osd/osd_warnings.c

  # CMS
  ${BF_ROOT}/src/main/cms/cms.c
  ${BF_ROOT}/src/main/cms/cms_menu_blackbox.c
  ${BF_ROOT}/src/main/cms/cms_menu_failsafe.c
  ${BF_ROOT}/src/main/cms/cms_menu_firmware.c
  ${BF_ROOT}/src/main/cms/cms_menu_gps_rescue_multirotor.c
  ${BF_ROOT}/src/main/cms/cms_menu_imu.c
  ${BF_ROOT}/src/main/cms/cms_menu_ledstrip.c
  ${BF_ROOT}/src/main/cms/cms_menu_main.c
  ${BF_ROOT}/src/main/cms/cms_menu_misc.c
  ${BF_ROOT}/src/main/cms/cms_menu_osd.c
  ${BF_ROOT}/src/main/cms/cms_menu_persistent_stats.c
  ${BF_ROOT}/src/main/cms/cms_menu_power.c
  ${BF_ROOT}/src/main/cms/cms_menu_saveexit.c
  ${BF_ROOT}/src/main/cms/cms_menu_vtx_common.c
  ${BF_ROOT}/src/main/cms/cms_menu_vtx_msp.c
  ${BF_ROOT}/src/main/cms/cms_menu_vtx_rtc6705.c
  ${BF_ROOT}/src/main/cms/cms_menu_vtx_smartaudio.c
  ${BF_ROOT}/src/main/cms/cms_menu_vtx_tramp.c

  # Blackbox
  ${BF_ROOT}/src/main/blackbox/blackbox.c
  ${BF_ROOT}/src/main/blackbox/blackbox_encoding.c
  ${BF_ROOT}/src/main/blackbox/blackbox_virtual.c

  # Config
  ${BF_ROOT}/src/main/config/config.c
  ${BF_ROOT}/src/main/config/feature.c
  ${BF_ROOT}/src/main/config/config_streamer.c
  ${BF_ROOT}/src/main/config/config_eeprom.c
  ${BF_ROOT}/src/main/config/simplified_tuning.c

  # PG (Parameter Groups)
  ${BF_ROOT}/src/main/pg/pg.c
  ${BF_ROOT}/src/main/pg/motor.c
  ${BF_ROOT}/src/main/pg/scheduler.c
  ${BF_ROOT}/src/main/pg/rx.c
  ${BF_ROOT}/src/main/pg/gyrodev.c
  ${BF_ROOT}/src/main/pg/board.c
  ${BF_ROOT}/src/main/pg/msp.c
  ${BF_ROOT}/src/main/pg/stats.c
  ${BF_ROOT}/src/main/pg/rcdevice.c
  ${BF_ROOT}/src/main/pg/gps.c
  ${BF_ROOT}/src/main/pg/gps_rescue_multirotor.c
  ${BF_ROOT}/src/main/pg/gps_rescue_wing.c
  ${BF_ROOT}/src/main/pg/vcd.c

  # Scheduler
  ${BF_ROOT}/src/main/scheduler/scheduler.c

  # External libs
  ${BF_ROOT}/lib/main/dyad/dyad.c
  ${BF_ROOT}/lib/main/google/olc/olc.c
)

# =============================================================================
# Additional sources required by the official SITL target (make TARGET=SITL)
# =============================================================================
set(BF_SOURCES ${BF_SOURCES}
  # Blackbox I/O
  ${BF_ROOT}/src/main/blackbox/blackbox_io.c

  # CMS
  ${BF_ROOT}/src/main/cms/cms_menu_gps_lap_timer.c
  ${BF_ROOT}/src/main/cms/cms_menu_gps_rescue_wing.c
  ${BF_ROOT}/src/main/cms/cms_menu_quick.c
  ${BF_ROOT}/src/main/cms/cms_menu_rpm_limit.c

  # Common
  ${BF_ROOT}/src/main/common/chirp.c
  ${BF_ROOT}/src/main/common/colorconversion.c
  ${BF_ROOT}/src/main/common/explog_approx.c
  ${BF_ROOT}/src/main/common/huffman.c
  ${BF_ROOT}/src/main/common/huffman_table.c
  ${BF_ROOT}/src/main/common/pwl.c
  ${BF_ROOT}/src/main/common/sdft.c
  ${BF_ROOT}/src/main/common/stopwatch.c
  ${BF_ROOT}/src/main/common/strtol.c
  ${BF_ROOT}/src/main/common/uvarint.c

  # Drivers
  ${BF_ROOT}/src/main/drivers/bus_i2c_utils.c
  ${BF_ROOT}/src/main/drivers/bus_octospi.c
  ${BF_ROOT}/src/main/drivers/bus_quadspi.c
  ${BF_ROOT}/src/main/drivers/buttons.c
  ${BF_ROOT}/src/main/drivers/camera_control.c
  ${BF_ROOT}/src/main/drivers/display_canvas.c
  ${BF_ROOT}/src/main/drivers/dma.c
  ${BF_ROOT}/src/main/drivers/dshot_command.c
  ${BF_ROOT}/src/main/drivers/io_preinit.c
  ${BF_ROOT}/src/main/drivers/lcd_console.c
  ${BF_ROOT}/src/main/drivers/lcd_panel/lcd_panel_font_5x7.c
  ${BF_ROOT}/src/main/drivers/lcd_panel/lcd_panel_stub.c
  ${BF_ROOT}/src/main/drivers/lcd_panel/ssd1306_i2c.c
  ${BF_ROOT}/src/main/drivers/light_ws2811strip.c
  ${BF_ROOT}/src/main/drivers/pin_pull_up_down.c
  ${BF_ROOT}/src/main/drivers/pitot/pitot_ms4525.c
  ${BF_ROOT}/src/main/drivers/rangefinder/rangefinder_hcsr04.c
  ${BF_ROOT}/src/main/drivers/rangefinder/rangefinder_lidartf.c
  ${BF_ROOT}/src/main/drivers/rangefinder/rangefinder_lidarmt.c
  ${BF_ROOT}/src/main/drivers/rangefinder/rangefinder_nooploop.c
  ${BF_ROOT}/src/main/drivers/rangefinder/rangefinder_upt1.c
  ${BF_ROOT}/src/main/drivers/rx/rx_spi.c
  ${BF_ROOT}/src/main/drivers/serial_impl.c
  ${BF_ROOT}/src/main/drivers/serial_lcd_console.c
  ${BF_ROOT}/src/main/drivers/serial_softserial.c
  ${BF_ROOT}/src/main/drivers/sound_beeper.c
  ${BF_ROOT}/src/main/drivers/transponder_ir_arcitimer.c
  ${BF_ROOT}/src/main/drivers/transponder_ir_erlt.c
  ${BF_ROOT}/src/main/drivers/transponder_ir_ilap.c

  # FC
  ${BF_ROOT}/src/main/fc/faults.c
  ${BF_ROOT}/src/main/fc/gps_lap_timer.c

  # Flight
  ${BF_ROOT}/src/main/flight/alt_hold_multirotor.c
  ${BF_ROOT}/src/main/flight/alt_hold_wing.c
  ${BF_ROOT}/src/main/flight/autopilot_wing.c
  ${BF_ROOT}/src/main/flight/dyn_notch_filter.c
  ${BF_ROOT}/src/main/flight/flight_plan_capture.c
  ${BF_ROOT}/src/main/flight/flight_plan_nav.c
  ${BF_ROOT}/src/main/flight/gps_rescue_wing.c
  ${BF_ROOT}/src/main/flight/nav_trail.c
  ${BF_ROOT}/src/main/flight/pos_hold_multirotor.c
  ${BF_ROOT}/src/main/flight/pos_hold_wing.c
  ${BF_ROOT}/src/main/flight/position_estimator.c
  ${BF_ROOT}/src/main/flight/position_filter.c
  ${BF_ROOT}/src/main/flight/position_nav.c
  ${BF_ROOT}/src/main/flight/rpm_filter.c

  # IO
  ${BF_ROOT}/src/main/io/dashboard.c
  ${BF_ROOT}/src/main/io/displayport_crsf.c
  ${BF_ROOT}/src/main/io/displayport_fb_osd.c
  ${BF_ROOT}/src/main/io/displayport_frsky_osd.c
  ${BF_ROOT}/src/main/io/displayport_hott.c
  ${BF_ROOT}/src/main/io/displayport_max7456.c
  ${BF_ROOT}/src/main/io/displayport_srxl.c
  ${BF_ROOT}/src/main/io/frsky_osd.c
  ${BF_ROOT}/src/main/io/gimbal_control.c
  ${BF_ROOT}/src/main/io/ledstrip.c
  ${BF_ROOT}/src/main/io/pidaudio.c
  ${BF_ROOT}/src/main/io/piniobox.c
  ${BF_ROOT}/src/main/io/serial.c
  ${BF_ROOT}/src/main/io/serial_4way.c
  ${BF_ROOT}/src/main/io/serial_4way_avrootloader.c
  ${BF_ROOT}/src/main/io/serial_4way_stk500v2.c
  ${BF_ROOT}/src/main/io/smartaudio_protocol.c
  ${BF_ROOT}/src/main/io/spektrum_rssi.c
  ${BF_ROOT}/src/main/io/spektrum_vtx_control.c
  ${BF_ROOT}/src/main/io/tramp_protocol.c
  ${BF_ROOT}/src/main/io/transponder_ir.c
  ${BF_ROOT}/src/main/io/usb_cdc_hid.c
  ${BF_ROOT}/src/main/io/usb_msc.c
  ${BF_ROOT}/src/main/io/vtx.c
  ${BF_ROOT}/src/main/io/vtx_control.c
  ${BF_ROOT}/src/main/io/vtx_msp.c
  ${BF_ROOT}/src/main/io/vtx_rtc6705.c
  ${BF_ROOT}/src/main/io/vtx_smartaudio.c
  ${BF_ROOT}/src/main/io/vtx_tramp.c

  # OSD
  ${BF_ROOT}/src/main/osd/osd_custom_text.c
  ${BF_ROOT}/src/main/osd/osd_nav_map.c

  # PG
  ${BF_ROOT}/src/main/pg/adc.c
  ${BF_ROOT}/src/main/pg/alt_hold_multirotor.c
  ${BF_ROOT}/src/main/pg/alt_hold_wing.c
  ${BF_ROOT}/src/main/pg/autopilot.c
  ${BF_ROOT}/src/main/pg/beeper.c
  ${BF_ROOT}/src/main/pg/beeper_dev.c
  ${BF_ROOT}/src/main/pg/bus_i2c.c
  ${BF_ROOT}/src/main/pg/bus_quadspi.c
  ${BF_ROOT}/src/main/pg/bus_spi.c
  ${BF_ROOT}/src/main/pg/can.c
  ${BF_ROOT}/src/main/pg/dashboard.c
  ${BF_ROOT}/src/main/pg/displayport_profiles.c
  ${BF_ROOT}/src/main/pg/dronecan.c
  ${BF_ROOT}/src/main/pg/dronecan_dna.c
  ${BF_ROOT}/src/main/pg/dyn_notch.c
  ${BF_ROOT}/src/main/pg/flash.c
  ${BF_ROOT}/src/main/pg/flight_plan.c
  ${BF_ROOT}/src/main/pg/gimbal.c
  ${BF_ROOT}/src/main/pg/gps_lap_timer.c
  ${BF_ROOT}/src/main/pg/max7456.c
  ${BF_ROOT}/src/main/pg/mco.c
  ${BF_ROOT}/src/main/pg/osd_nav_map.c
  ${BF_ROOT}/src/main/pg/pilot.c
  ${BF_ROOT}/src/main/pg/pin_pull_up_down.c
  ${BF_ROOT}/src/main/pg/pinio.c
  ${BF_ROOT}/src/main/pg/piniobox.c
  ${BF_ROOT}/src/main/pg/pos_hold_multirotor.c
  ${BF_ROOT}/src/main/pg/pos_hold_wing.c
  ${BF_ROOT}/src/main/pg/rpm_filter.c
  ${BF_ROOT}/src/main/pg/rx_pwm.c
  ${BF_ROOT}/src/main/pg/rx_spi.c
  ${BF_ROOT}/src/main/pg/rx_spi_cc2500.c
  ${BF_ROOT}/src/main/pg/rx_spi_expresslrs.c
  ${BF_ROOT}/src/main/pg/sdcard.c
  ${BF_ROOT}/src/main/pg/sdio.c
  ${BF_ROOT}/src/main/pg/serial_uart.c
  ${BF_ROOT}/src/main/pg/timerio.c
  ${BF_ROOT}/src/main/pg/timerup.c
  ${BF_ROOT}/src/main/pg/usb.c
  ${BF_ROOT}/src/main/pg/vtx_io.c
  ${BF_ROOT}/src/main/pg/vtx_table.c

  # RX
  ${BF_ROOT}/src/main/rx/crsf.c
  ${BF_ROOT}/src/main/rx/fport.c
  ${BF_ROOT}/src/main/rx/frsky_crc.c
  ${BF_ROOT}/src/main/rx/ghst.c
  ${BF_ROOT}/src/main/rx/ibus.c
  ${BF_ROOT}/src/main/rx/jetiexbus.c
  ${BF_ROOT}/src/main/rx/mavlink.c
  ${BF_ROOT}/src/main/rx/pwm.c
  ${BF_ROOT}/src/main/rx/rx_bind.c
  ${BF_ROOT}/src/main/rx/rx_spi.c
  ${BF_ROOT}/src/main/rx/rx_spi_common.c
  ${BF_ROOT}/src/main/rx/sbus.c
  ${BF_ROOT}/src/main/rx/sbus_channels.c
  ${BF_ROOT}/src/main/rx/spektrum.c
  ${BF_ROOT}/src/main/rx/srxl2.c
  ${BF_ROOT}/src/main/rx/sumd.c
  ${BF_ROOT}/src/main/rx/sumh.c
  ${BF_ROOT}/src/main/rx/xbus.c

  # Sensors
  ${BF_ROOT}/src/main/sensors/battery.c
  ${BF_ROOT}/src/main/sensors/opticalflow.c
  ${BF_ROOT}/src/main/sensors/pitot.c
  ${BF_ROOT}/src/main/sensors/rangefinder.c
  ${BF_ROOT}/src/main/sensors/sensors.c

  # Target / config
  ${BF_ROOT}/src/main/target/config_helper.c

  # Telemetry
  ${BF_ROOT}/src/main/telemetry/frsky_hub.c
  ${BF_ROOT}/src/main/telemetry/hott.c
  ${BF_ROOT}/src/main/telemetry/ibus.c
  ${BF_ROOT}/src/main/telemetry/ibus_shared.c
  ${BF_ROOT}/src/main/telemetry/jetiexbus.c
  ${BF_ROOT}/src/main/telemetry/ltm.c
  ${BF_ROOT}/src/main/telemetry/mavlink.c
  ${BF_ROOT}/src/main/telemetry/mavlink_mission.c
  ${BF_ROOT}/src/main/telemetry/msp_shared.c
  ${BF_ROOT}/src/main/telemetry/smartport.c
  ${BF_ROOT}/src/main/telemetry/telemetry.c
)

if(WIN32)
  # Replace the POSIX udplink.c with a Winsock implementation
  list(REMOVE_ITEM BF_SOURCES "${BF_ROOT}/src/platform/SIMULATOR/udplink.c")
  list(REMOVE_ITEM BF_SOURCES "${BF_ROOT}/src/main/main.c")
  list(REMOVE_ITEM BF_SOURCES "${BF_ROOT}/src/main/drivers/serial_tcp.c")
  # Rename sitl.c's time functions so wincompat.c can provide stepped virtual
  # time (like AJ92/SimITL): the scheduler's gyro busy-wait then terminates in
  # a few fixed steps instead of spinning in real time.
  # systemReset is renamed so wincompat.c can install a custom firmware reboot
  # that relaunches the SITL process automatically instead of just exiting.
  # fopen/fclose are renamed so wincompat.c can redirect the virtual EEPROM to
  # BF_SITL_EEPROM and track whether that file is currently open (the LOCAL
  # link needs to know before re-opening it at another path).
  set(SITL_SITL_C_OPTS "-Dmicros64=sitlMicros64;-Dmillis64=sitlMillis64;-Dmicros=sitlMicros;-Dmillis=sitlMillis;-DdelayMicroseconds=sitlDelayMicroseconds;-DgetCycleCounter=sitlGetCycleCounter;-Dfopen=sitlFopen;-Dfclose=sitlFclose;-Dpthread_mutex_trylock=sitlMutexTrylock;-Dpthread_mutex_unlock=sitlMutexUnlock;-DsystemReset=sitlSystemResetNative")
  if(SITL_LINK_MODE STREQUAL "LOCAL")
    # sitl.c's systemResetToBootloader() calls exit(0), which would kill the
    # host process from a DLL ("Enter DFU/bootloader" in the configurator).
    # wincompat.c provides a LOCAL replacement that persists and jumps back to
    # the MSP thread loop, like the firmware reboot path.
    set(SITL_SITL_C_OPTS "${SITL_SITL_C_OPTS};-DsystemResetToBootloader=sitlSystemResetToBootloaderNative")
  endif()
  set_source_files_properties("${BF_ROOT}/src/platform/SIMULATOR/sitl.c" PROPERTIES COMPILE_OPTIONS "${SITL_SITL_C_OPTS}")
  # msp_serial.c's millis() calls (CLI entry guard, configurator-activity
  # timeout) must track real time even while the UDP-driven virtual clock is
  # frozen during idle, otherwise the CLI can never be entered and the
  # activity window never expires. wincompat.c provides sitlMspMillis().
  set_source_files_properties("${BF_ROOT}/src/main/msp/msp_serial.c" PROPERTIES COMPILE_OPTIONS
    "-Dmillis=sitlMspMillis")
  # A firmware reboot terminates the SITL process; make sure the current RAM
  # config is persisted first (wincompat.c's sitlSystemReset calls
  # writeEEPROM() before the real systemReset), so "Save and Reboot" always
  # saves even if the MSP_EEPROM_WRITE step was skipped or lost.
  if(SITL_LINK_MODE STREQUAL "LOCAL")
    set_source_files_properties("${BF_ROOT}/src/main/msp/msp.c" PROPERTIES COMPILE_OPTIONS
      "-DsystemReset=sitlSystemReset;-DwriteEEPROM=sitlMspWriteEEPROM;-DreadEEPROM=sitlMspReadEEPROM;-DinitRcProcessing=sitlMspInitRcProcessing;-DgyroInitFilters=sitlMspGyroInitFilters;-DpidInitFilters=sitlMspPidInitFilters")
    # Every reboot path (CLI exit, MSP reboot) calls motorShutdown() to stop
    # the ESC outputs before the MCU resets. LOCAL mode never actually resets:
    # systemReset() defers the EEPROM persist and the FC keeps running, so the
    # real shutdown (initialized=false) would make motorEnable() fail forever
    # after the next arming - the FC shows armed but PWM never comes back.
    # wincompat.c provides a no-op motorShutdown for LOCAL mode.
    set_source_files_properties("${BF_ROOT}/src/main/drivers/motor.c" PROPERTIES COMPILE_OPTIONS
      "-DmotorShutdown=sitlMotorShutdownReal")
    # The virtual blackbox writes LOG*.BFL into the process working directory,
    # which for a DLL is the host engine's unpredictable CWD. Redirect the file
    # open and the log-numbering directory scan to the stable
    # %LOCALAPPDATA%\Betaflight-SITL folder (same as the virtual EEPROM).
    set_source_files_properties("${BF_ROOT}/src/main/blackbox/blackbox_virtual.c" PROPERTIES COMPILE_OPTIONS
      "-Dfopen=sitlBlackboxFopen;-Dopendir=sitlBlackboxOpendir;-DblackboxVirtualOpen=sitlBlackboxVirtualOpenReal")
    # blackbox.c flushes the log "every iteration so that our runtime variance is
    # minimized" - at a 4 kHz loop with the default sample_rate of 1/4 that is
    # 1000 flush() syscalls per second, straight out of the flight loop. Rename
    # only that call site: blackboxDeviceFlushForce()/ForceComplete() keep the
    # real flush for the log start/stop and cache-flush paths, and the wrapper in
    # wincompat.c throttles the per-iteration one to 10 Hz.
    set_source_files_properties("${BF_ROOT}/src/main/blackbox/blackbox.c" PROPERTIES COMPILE_OPTIONS
      "-DblackboxDeviceFlush=sitlBlackboxDeviceFlushThrottled")
    # writeEEPROM() is a synchronous 32 KB file write plus fopen/fclose/rename,
    # and the flight thread reaches it directly: fc/stats.c writes the stats from
    # the dispatcher (which runs inside sitl_local_step) the moment the aircraft
    # disarms, and cli.c / gyro_init.c do the same from their own paths. Those
    # call sites become the deferred hook (wincompat.c), which only raises the
    # pending flag; the MSP thread does the real write - exactly what the MSP
    # save path above already does. config.c keeps the real definition, which is
    # what the MSP thread and localFlushEepromWrite() call.
    foreach(BF_EEPROM_WRITER "fc/stats.c" "cli/cli.c" "sensors/gyro_init.c")
      set_source_files_properties("${BF_ROOT}/src/main/${BF_EEPROM_WRITER}" PROPERTIES COMPILE_OPTIONS
        "-DwriteEEPROM=sitlWriteEEPROMDeferred")
    endforeach()
    # Arming path trace (wincompat.c): tryArm() arms and processRcStickPositions()
    # is its only RC caller, so tracing the two call sites names whichever gate is
    # shut when the aircraft will not arm while armFlags reads 0.
    set_source_files_properties("${BF_ROOT}/src/main/fc/core.c" PROPERTIES COMPILE_OPTIONS
      "-DtryArm=sitlTryArmReal;-DprocessRcStickPositions=sitlProcessRcStickPositionsTraced;-DgetLastDshotBeaconCommandTimeUs=sitlLastDshotBeaconCommandTimeUs")
    set_source_files_properties("${BF_ROOT}/src/main/fc/rc_controls.c" PROPERTIES COMPILE_OPTIONS
      "-DtryArm=sitlTryArmTraced;-DprocessRcStickPositions=sitlProcessRcStickPositionsReal;-Ddisarm=sitlDisarmTraced")
    # osd.c's post-flight statistics page prints a blackbox storage line:
    # osdGetBlackboxStatusString() computes used/total where storageTotal stays 0
    # whenever neither USE_SDCARD nor USE_FLASHFS is compiled (both are absent on
    # SITL), and GCC constant-folds that division into a ud2 trap. The row is
    # drawn while disarmed, so the aircraft dies on every arm/disarm with
    # 0xC000001D inside osdUpdate(). Report "no storage device" to the OSD
    # instead: the file-based virtual blackbox has no storage statistics to show.
    set_source_files_properties("${BF_ROOT}/src/main/osd/osd.c" PROPERTIES COMPILE_OPTIONS
      "-DisBlackboxDeviceWorking=sitlOsdIsBlackboxDeviceWorking")
  else()
    set_source_files_properties("${BF_ROOT}/src/main/msp/msp.c" PROPERTIES COMPILE_OPTIONS
      "-DsystemReset=sitlSystemReset")
  endif()
  # UDP-fed battery telemetry: battery.c's ADC meter reads are renamed to
  # wincompat shims that return voltage/current from the extended FDM packet
  # (sim_telemetry.h), and fresh EEPROMs default the meters to that path.
  # batteryUpdateVoltage is renamed so wincompat.c can run the complete update
  # first and then probe for battery presence with this tick's voltage (the
  # refresh hook runs before the meter is read).
  set_source_files_properties("${BF_ROOT}/src/main/sensors/battery.c" PROPERTIES COMPILE_OPTIONS
    "-DvoltageMeterADCRefresh=sitlBatteryVoltageRefresh;-DvoltageMeterADCRead=sitlBatteryVoltageRead;-DcurrentMeterADCRefresh=sitlBatteryCurrentRefresh;-DcurrentMeterADCRead=sitlBatteryCurrentRead;-DbatteryUpdateVoltage=sitlBatteryUpdateVoltageReal;-DDEFAULT_VOLTAGE_METER_SOURCE=VOLTAGE_METER_ADC;-DDEFAULT_CURRENT_METER_SOURCE=CURRENT_METER_ADC")
  # The dyad-based TCP bridge is replaced by serial_tcp_win.c; dyad_update()
  # busy-spins when no streams exist, so wincompat.c provides a sleeping stub.
  set_source_files_properties("${BF_ROOT}/lib/main/dyad/dyad.c" PROPERTIES COMPILE_OPTIONS "-Ddyad_update=dyad_update_real")
  list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/udplink_windows.c")
  list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/wincompat.c")
  list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main_windows.c")
  list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_tcp_win.c")
  list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/ws_proxy_win.c")
  if(SITL_LINK_MODE STREQUAL "LOCAL")
    list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/sitl_local.c")
    # OSD canvas capture: wraps the firmware's MSP displayport and publishes the
    # character grid for the host (see src/sitl_osd.c).
    list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/sitl_osd.c")

    # io/displayport_hott.c is guarded by "USE_HOTT_TEXTMODE && USE_CMS".
    # Enabling the OSD defines USE_CMS, which activates the file, but
    # USE_HOTT_TEXTMODE lives in common_pre.h's non-SITL block (the SITL target
    # disables HOTT telemetry), so the hottTextmode* functions it calls are not
    # compiled and the link fails with undefined references. Nothing on SITL
    # references the HOTT displayport, so drop the source instead.
    list(REMOVE_ITEM BF_SOURCES "${BF_ROOT}/src/main/io/displayport_hott.c")
    # Optional A/B switch: inject the FDM attitude straight into the IMU instead
    # of letting the firmware's estimator (USE_IMU_CALC) derive it from the
    # virtual acc/gyro/mag. Applied to sitl_local.c only, so toggling it rebuilds
    # one file and a comparison build can coexist with the default one.
    option(SITL_ATTITUDE_DIRECT "LOCAL: inject the FDM attitude directly (bypass the estimator)" OFF)
    if(SITL_ATTITUDE_DIRECT)
      set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/src/sitl_local.c"
        PROPERTIES COMPILE_OPTIONS "-DSITL_ATTITUDE_DIRECT")
    endif()

    # msp_box.c only adds BOXCRASHFLIP (FLIP OVER AFTER CRASH) to the active box
    # list when the configured motor protocol is DShot. The LOCAL link has to run
    # the virtual PWM backend, so wincompat.c provides initActiveBoxIds() that
    # reports DShot for the duration of that pure config function and restores
    # PWM immediately after - the Configurator then shows the mode while the
    # motor backend, RPM telemetry and output path stay untouched.
    set_source_files_properties("${BF_ROOT}/src/main/msp/msp_box.c"
      PROPERTIES COMPILE_OPTIONS "-DinitActiveBoxIds=sitlInitActiveBoxIdsReal")

    # Crash flip: the firmware latches crashFlipModeActive only when the motor
    # protocol is DShot (core.c tryArm), so a PWM-only LOCAL backend would never
    # flip. sitl_local.c reproduces the latch; these are the TUs that read it.
    foreach(_cf_source
        "${BF_ROOT}/src/main/flight/mixer.c"
        "${BF_ROOT}/src/main/io/ledstrip.c"
        "${BF_ROOT}/src/main/osd/osd.c"
        "${BF_ROOT}/src/main/osd/osd_elements.c"
        "${BF_ROOT}/src/main/osd/osd_warnings.c")
      set_property(SOURCE "${_cf_source}" APPEND PROPERTY COMPILE_OPTIONS
        "-DisCrashFlipModeActive=sitlIsCrashFlipModeActive")
    endforeach()
  endif()
endif()

# =============================================================================
# Target type: UDP mode produces the standalone server executable; LOCAL mode
# produces an in-process DLL that Unreal's async physics tick calls directly
# (zero UDP latency).
# =============================================================================
if(SITL_LINK_MODE STREQUAL "LOCAL")
  add_library(betaflight_SITL SHARED ${BF_SOURCES})
  set_target_properties(betaflight_SITL PROPERTIES PREFIX "")
  # Line tables for the firmware sources: a crash report carries the faulting
  # instruction's RVA (see sitlCrashLogger() in src/wincompat.c), and without
  # -g addr2line can only answer "osdUpdate" instead of "osd.c:1234". Debug
  # info does not change code generation at the same -O2.
  target_compile_options(betaflight_SITL PRIVATE -g)
  # MinGW auto-exports every global symbol in a DLL by default, which keeps
  # firmware sections alive that the executable build normally garbage
  # collects (and that reference target-only helpers). Export only the API
  # functions marked __declspec(dllexport) in sitl_local.h.
  target_link_options(betaflight_SITL PRIVATE "-Wl,--exclude-all-symbols")
else()
  add_executable(betaflight_SITL ${BF_SOURCES})
endif()

# Gyro/filter/PID loop frequency in Hz (the official scheduler busy-waits to
# this exact deadline). It drives *both* the task periods and the virtual gyro's
# sample rate (VIRTUAL_GYRO_SAMPLE_RATE_HZ below), because the filter chain, the
# RPM filter's dt/Nyquist ceiling and the dynamic-notch gate all derive from
# gyro.sampleLooptime/targetLooptime - if the two disagree the firmware runs the
# filters at a rate they were not built for.
#
# 0 = automatic: 4000 Hz for the LOCAL DLL (the host feeds a physics-rate gyro
# stream, so the loop should match it) and 1000 Hz for the UDP/standalone build
# (the FDM packet rate). Can be overridden at runtime with BF_SITL_GYRO_HZ; the
# boot code then re-derives the gyro sample rate and the filter chains so the two
# stay in sync.
set(SITL_GYRO_HZ 0 CACHE STRING "SITL gyro/filter/PID loop frequency in Hz (0 = auto)")

if(SITL_GYRO_HZ EQUAL 0)
  if(SITL_LINK_MODE STREQUAL "LOCAL")
    set(SITL_GYRO_HZ 4000)
  else()
    set(SITL_GYRO_HZ 1000)
  endif()
endif()

# Virtual brushless motor PWM rate used by Betaflight's config validation.
# Real ESCs cap standard PWM near 480 Hz, which makes the firmware raise
# pid_process_denom (PID drops to ~333 Hz at a 1 kHz gyro) whenever the
# "PWM speed independent of PID" option is disabled. The SITL has no real
# PWM hardware, so raise the cap: synced mode keeps a 1 kHz PID loop and the
# motor UDP packets still go out once per PID iteration.
set(SITL_BRUSHLESS_PWM_RATE 20000 CACHE STRING "Virtual brushless motor PWM rate used by config validation")

# Time base selection: REALTIME is the official Betaflight scheduler
# (busy-waits to the exact gyro deadline, one CPU core). UDP drives the
# virtual clock from Unreal FDM packets on UDP 9003: the flight loop runs
# only while packets arrive, and idles (but keeps serial/MSP alive) between
# packets.
set(SITL_TIME_MODE "REALTIME" CACHE STRING "SITL time base: REALTIME (official) or UDP (FDM packet-driven)")

# =============================================================================
# Compile definitions
# =============================================================================
target_compile_definitions(betaflight_SITL PRIVATE
  SITL
  SIMULATOR
  SIMULATOR_BUILD
  SITL_GYRO_HZ=${SITL_GYRO_HZ}
  VIRTUAL_GYRO_SAMPLE_RATE_HZ=${SITL_GYRO_HZ}
  BRUSHLESS_MOTORS_PWM_RATE=${SITL_BRUSHLESS_PWM_RATE}
  DEFAULT_BLACKBOX_DEVICE=BLACKBOX_DEVICE_VIRTUAL
  USE_CHIRP
  __TARGET__=\"SITL\"
  __REVISION__=\"1\"
  __FORKNAME__=\"betaflight\"
  __MCU_NAME__=\"SIMULATOR\"
  HSE_VALUE=8000000
  FLASH_SIZE=2048
)

# Attitude source: the SITL runs the real IMU estimator (USE_IMU_CALC,
# Mahony) against the virtual acc/gyro/mag feeds. Defining
# SITL_ATTITUDE_DIRECT instead would inject the FDM quaternion directly and
# disable the estimator.

if(SITL_TIME_MODE STREQUAL "UDP")
  target_compile_definitions(betaflight_SITL PRIVATE SITL_UDP_TIME)
endif()

if(SITL_LINK_MODE STREQUAL "LOCAL")
  # The in-process library uses the same packet-free virtual clock stepping as
  # UDP mode, but the time is advanced by sitl_local_step() instead of by
  # incoming FDM datagrams.
  target_compile_definitions(betaflight_SITL PRIVATE
    SITL_LOCAL
    SITL_UDP_TIME
    USE_DSHOT
    USE_RPM_FILTER
  )

  # Feed simulated motor RPM into the firmware's DSHOT-telemetry consumers
  # (RPM filter, dynamic idle, OSD/MSP motor telemetry, blackbox eRPM).
  # dshot.c's definitions are renamed so wincompat.c can provide wrappers
  # that return simTelemetryMotorFrequencyHz() values when present.
  set_source_files_properties("${BF_ROOT}/src/main/drivers/dshot.c" PROPERTIES COMPILE_OPTIONS
    "-DgetDshotRpm=sitlDshotRpmReal;-DgetDshotRpmAverage=sitlDshotRpmAverageReal;-DgetDshotErpm=sitlDshotErpmReal;-DgetMotorFrequencyHz=sitlMotorFrequencyHzReal;-DgetMinMotorFrequencyHz=sitlMinMotorFrequencyHzReal;-DisDshotTelemetryActive=sitlIsDshotTelemetryActiveReal")

  # mspSerialProcess() runs from two threads in LOCAL mode (the scheduler's
  # TASK_SERIAL inside sitl_local_step and the 1 ms background MSP thread).
  # The shared msp_serial parser (and CLI state machine) is not reentrant, so
  # serialize both callers through a mutex wrapper in sitl_local.c - without
  # this, entering the CLI panel and leaving it can leave the port wedged in
  # CLI mode.
  set_source_files_properties("${BF_ROOT}/src/main/msp/msp_serial.c" PROPERTIES COMPILE_OPTIONS
    "-DmspSerialProcess=sitlMspSerialProcessReal")

  # displayport_msp.c's displayPortMspInit() is the displayport the firmware
  # builds when the OSD runs in HD/MSP mode. Rename it so src/sitl_osd.c can
  # provide the displayPortMspInit() the firmware calls, wrap the real vtable
  # (keeping the MSP/goggle stream intact) and mirror every clear/write/draw
  # into a character grid the host reads back. See src/sitl_osd.c.
  set_source_files_properties("${BF_ROOT}/src/main/io/displayport_msp.c" PROPERTIES COMPILE_OPTIONS
    "-DdisplayPortMspInit=sitlRealDisplayPortMspInit")

  # Prepend wincompat so the x86 build/atomic.h shadow wins over the
  # ARM-only submodule header (required once USE_DSHOT is enabled).
  target_include_directories(betaflight_SITL PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/wincompat
  )

  # ---------------------------------------------------------------------------
  # OSD enablement (shadow target.h).
  #
  # The SITL target does not build the OSD: in
  # extern/betaflight/src/main/target/common_pre.h the whole classic-build block
  # (which contains "#define USE_OSD" and "#define USE_MAX7456") sits inside
  #
  #     #if !defined(CLOUD_BUILD) && !defined(SITL)
  #
  # and this build defines SITL, so those gates are never opened. target.h's own
  # "#undef USE_OSD" is therefore a no-op rather than the cause.
  #
  # platform.h includes target/common_pre.h (line 37) BEFORE "target.h"
  # (line 42), so a shim placed ahead of the real target directory can re-open
  # the OSD gates after common_pre.h has run and before common_post.h runs - at
  # which point the firmware behaves exactly like a normal OSD-capable target.
  #
  # The pinned extern/betaflight submodule is deliberately NOT modified; this is
  # the same layering trick already used above for wincompat/.
  #
  # Only the LOCAL in-process build gets the OSD: the standalone UDP server is
  # not a host for the Unreal HUD and its link script (sitl.ld) is not set up
  # for the extra sections.
  # ---------------------------------------------------------------------------
  target_include_directories(betaflight_SITL BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sitl_osd/include
  )
endif()

# =============================================================================
# Include directories
# =============================================================================
target_include_directories(betaflight_SITL PRIVATE
  ${BF_ROOT}/src/main
  ${BF_ROOT}/src/platform/SIMULATOR/include
  ${BF_ROOT}/src/platform/SIMULATOR/target/SITL
  ${BF_ROOT}/lib/main/dyad
  ${BF_ROOT}/lib/main/MAVLink
  ${BF_ROOT}/lib/main/google/olc
)

# Windows compatibility headers for POSIX socket includes in Betaflight
if(WIN32)
  target_include_directories(betaflight_SITL PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/wincompat
  )
  target_compile_options(betaflight_SITL PRIVATE
    "-include${CMAKE_CURRENT_SOURCE_DIR}/wincompat/wincompat.h"
  )
endif()

target_compile_definitions(betaflight_SITL PRIVATE
  _GNU_SOURCE
)

# Official SITL build also relies on the simulator platform include dir
target_include_directories(betaflight_SITL PRIVATE
  ${BF_ROOT}/src/platform/SIMULATOR
)

# PG registry placement required by Betaflight's parameter-group code
if(WIN32)
  target_link_options(betaflight_SITL PRIVATE
    "-Wl,-T,${CMAKE_CURRENT_SOURCE_DIR}/wincompat/sitl_win.ld"
    "-Wl,--gc-sections"
  )
else()
  target_link_options(betaflight_SITL PRIVATE
    "-Wl,-T,${BF_ROOT}/src/platform/SIMULATOR/link/sitl.ld"
    "-Wl,--gc-sections"
  )
endif()

# =============================================================================
# Platform-specific linking
# =============================================================================
if(WIN32)
  # MinGW: link Winsock2 and Windows Multimedia (for timers)
  target_link_libraries(betaflight_SITL PRIVATE
    ws2_32      # Winsock2 (socket, bind, recvfrom, sendto)
    winmm       # Windows Multimedia (timeGetTime, etc.)
    pthread     # POSIX threads (via winpthread)
    crypt32     # CryptoAPI (WebSocket Sec-WebSocket-Accept)
  )
else()
  target_link_libraries(betaflight_SITL PRIVATE
    pthread
    m           # math library
    dl          # dynamic loader
  )
endif()

# =============================================================================
# Install
# =============================================================================
install(TARGETS betaflight_SITL
  RUNTIME DESTINATION bin
)

# =============================================================================
# LOCAL mode: command-line tester that exercises the DLL at 1000 Hz
# =============================================================================
if(SITL_LINK_MODE STREQUAL "LOCAL")
  add_executable(sitl_local_tester tools/sitl_local_tester.c)
  target_link_libraries(sitl_local_tester PRIVATE betaflight_SITL)
  target_include_directories(sitl_local_tester PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  add_executable(sitl_local_host tools/sitl_local_host.c)
  target_link_libraries(sitl_local_host PRIVATE betaflight_SITL)
  target_include_directories(sitl_local_host PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # Save-invariance harness: replays one deterministic control scenario before
  # and after a configurator save and diffs the motor outputs.
  add_executable(sitl_local_save_compare tools/sitl_local_save_compare.c)
  target_link_libraries(sitl_local_save_compare PRIVATE betaflight_SITL ws2_32)
  target_include_directories(sitl_local_save_compare PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # OSD capture check: boots the FC, steps it, and dumps the character grid the
  # firmware rendered - the end-to-end proof that the OSD is enabled and that a
  # host can read the canvas (see src/sitl_osd.c).
  add_executable(sitl_osd_check tools/sitl_osd_check.c)
  target_link_libraries(sitl_osd_check PRIVATE betaflight_SITL)
  target_include_directories(sitl_osd_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # OSD contract check: configures OSD elements the way the Configurator does and
  # asserts the canvas a host reads back changes to match - the "UE HUD follows
  # the SITL OSD" requirement, verified without an editor.
  add_executable(sitl_osd_contract_check tools/sitl_osd_contract_check.c)
  target_link_libraries(sitl_osd_contract_check PRIVATE betaflight_SITL)
  target_include_directories(sitl_osd_contract_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # OSD ABI check: proves the hand-written UE mirror (FBfSitlOsdFrame) is
  # byte-compatible with sitl_local_osd_frame_t, which the DLL writes into
  # directly. A drift otherwise corrupts silently.
  add_executable(sitl_osd_abi_check tools/sitl_osd_abi_check.c)
  target_link_libraries(sitl_osd_abi_check PRIVATE betaflight_SITL)
  target_include_directories(sitl_osd_abi_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # EEPROM side-effect check: booting with the OSD enabled must not persist
  # FEATURE_OSD into the user's aircraft config.
  add_executable(sitl_osd_eeprom_effect tools/sitl_osd_eeprom_effect.c)
  target_link_libraries(sitl_osd_eeprom_effect PRIVATE betaflight_SITL)
  target_include_directories(sitl_osd_eeprom_effect PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
endif()
