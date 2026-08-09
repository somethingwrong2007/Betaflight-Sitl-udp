cmake_minimum_required(VERSION 3.10)
project(betaflight-sitl-udp C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Platform flags
# =============================================================================
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
  # Rename sitl.c's getCycleCounter() so wincompat.c can provide a sleeping
  # variant that stops the SITL scheduler's gyro busy-wait.
  set_source_files_properties("${BF_ROOT}/src/platform/SIMULATOR/sitl.c" PROPERTIES COMPILE_OPTIONS "-DgetCycleCounter=sitlGetCycleCounter")
  # The dyad-based TCP bridge is replaced by serial_tcp_win.c; dyad_update()
  # busy-spins when no streams exist, so wincompat.c provides a sleeping stub.
  set_source_files_properties("${BF_ROOT}/lib/main/dyad/dyad.c" PROPERTIES COMPILE_OPTIONS "-Ddyad_update=dyad_update_real")
  list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/udplink_windows.c")
  list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/wincompat.c")
  list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main_windows.c")
  list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_tcp_win.c")
  list(APPEND BF_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/ws_proxy_win.c")
endif()

# =============================================================================
# Executable
# =============================================================================
add_executable(betaflight_SITL ${BF_SOURCES})

# =============================================================================
# Compile definitions
# =============================================================================
target_compile_definitions(betaflight_SITL PRIVATE
  SITL
  SIMULATOR
  SIMULATOR_BUILD
  __TARGET__=\"SITL\"
  __REVISION__=\"1\"
  __FORKNAME__=\"betaflight\"
  __MCU_NAME__=\"SIMULATOR\"
  HSE_VALUE=8000000
  FLASH_SIZE=2048
)

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
