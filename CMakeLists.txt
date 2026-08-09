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
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_ACC -DUSE_GYRO -DUSE_BARO -DUSE_GPS -DUSE_OSD -DUSE_CMS -DUSE_BLACKBOX")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_SERIAL_TCP -DUSE_PWM_OUTPUT")

# =============================================================================
# Betaflight SITL source files
# Extracted from official Makefile for TARGET=SITL
# =============================================================================
set(BF_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/extern/betaflight")

set(BF_SOURCES
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

  # Target - SITL (RESTORED: official UDP server)
  ${BF_ROOT}/src/main/target/SITL/sitl.c
  ${BF_ROOT}/src/main/target/SITL/target.c
  ${BF_ROOT}/src/main/target/SITL/udplink.c

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
  ${BF_ROOT}/src/main/pg/gps_rescue.c
  ${BF_ROOT}/src/main/pg/vcd.c

  # Scheduler
  ${BF_ROOT}/src/main/scheduler/scheduler.c

  # External libs
  ${BF_ROOT}/lib/main/dyad/dyad.c
  ${BF_ROOT}/lib/main/google/olc/olc.c
)

# =============================================================================
# Executable
# =============================================================================
add_executable(betaflight_SITL ${BF_SOURCES})

# =============================================================================
# Compile definitions
# =============================================================================
target_compile_definitions(betaflight_SITL PRIVATE
  SITL
  SIMULATOR_BUILD
  SIMULATOR_MULTITHREAD
  __TARGET__=\"SITL\"
  __REVISION__=\"1\"
  HSE_VALUE=8000000
  FLASH_SIZE=2048
  USE_ACC
  USE_GYRO
  USE_BARO
  USE_GPS
  USE_OSD
  USE_CMS
  USE_BLACKBOX
  USE_SERIAL_TCP
  USE_PWM_OUTPUT
  USE_DSHOT
  USE_RX_MSP
  USE_RX_PWM
)

# =============================================================================
# Include directories
# =============================================================================
target_include_directories(betaflight_SITL PRIVATE
  ${BF_ROOT}/src/main
  ${BF_ROOT}/src/platform/SIMULATOR/include
  ${BF_ROOT}/lib/main/dyad
  ${BF_ROOT}/lib/main/google/olc
)

# =============================================================================
# Platform-specific linking
# =============================================================================
if(WIN32)
  # MinGW: link Winsock2 and Windows Multimedia (for timers)
  target_link_libraries(betaflight_SITL PRIVATE
    ws2_32      # Winsock2 (socket, bind, recvfrom, sendto)
    winmm       # Windows Multimedia (timeGetTime, etc.)
    pthread     # POSIX threads (via winpthread)
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
