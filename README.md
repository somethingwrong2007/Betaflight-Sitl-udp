# BF-SITL-UDP

Betaflight SITL for Windows and Linux, in two flavours built from one
codebase:

- **UDP**: a standalone simulator server that talks to a physics engine over
  UDP and serves the Betaflight Configurator over TCP/WebSocket.
- **LOCAL**: an in-process DLL (`betaflight_SITL.dll`) that an engine steps
  directly, with no UDP in the control path.

The `extern/betaflight` submodule is pinned and never modified; all simulator
code lives in `src/` and is wired in through per-file CMake symbol renames.

## Features

- 1 kHz or 4 kHz gyro/filter/PID loop (`SITL_GYRO_HZ`, `BF_SITL_GYRO_HZ`).
- Three time bases: real-time, FDM-packet-driven, and host-stepped (LOCAL).
- Stock MSP/CLI over TCP 5761, a built-in WebSocket proxy on 6761 and a local
  web configurator on 8080.
- Config persistence (`eeprom.bin`), including one EEPROM per aircraft at
  runtime in the LOCAL build.
- Save, Save and Reboot, CLI `save`/`exit`/`defaults`, `diff`/`dump` and
  `--config` import.
- Reboot handling: the standalone build relaunches itself, the LOCAL build
  reboots in-process and drops the configurator link so it reconnects.
- Virtual blackbox (`.BFL`) with a bounded, per-aircraft log folder.
- Virtual battery, RPM and ESC temperature fed into the firmware.
- OSD canvas and font capture for hosts that draw the HUD themselves.

## Ports

| Port | Direction | Protocol | Purpose |
|------|-----------|----------|---------|
| 9001 | out | UDP `servo_packet_raw` | Motor/servo PWM (raw bridge format) |
| 9002 | out | UDP `servo_packet` | Motor speeds (normalized Gazebo format) |
| 9003 | in | UDP `fdm_packet` | FDM state; drives the virtual clock in UDP mode |
| 9004 | in | UDP `rc_packet` | RC channels |
| 5761 | both | TCP | MSP / CLI |
| 6761 | both | WebSocket | Configurator bridge to 5761 |
| 8080 | - | HTTP | Local web configurator (`bfweb-server.mjs`) |

## Repository layout

```
CMakeLists.txt          build system and Betaflight symbol renames
src/
  main_windows.c        entry point and scheduler loop
  wincompat.c           POSIX shims, virtual clock, reboot handling, hooks
  serial_tcp_win.c      Winsock serial bridge (MSP over TCP)
  ws_proxy_win.c        WebSocket proxy (6761 -> 5761)
  udplink_windows.c     Winsock UDP transport and FDM clock
  sitl_local.c          LOCAL link: step API, overrides, per-aircraft state
  sitl_osd.c            OSD canvas/font capture
bfweb-server.mjs        local web configurator server
start-bf.cmd/.ps1       Windows launcher
tools/                  test harnesses and probes
extern/betaflight/      pinned Betaflight submodule (do not edit)
```

Customisations use per-file `-D` renames. Examples: `sitl.c` time functions
become `sitl*` so `wincompat.c` can supply stepped virtual time;
`systemReset` is renamed so it can persist and restart first; `battery.c`,
`msp_box.c` and `isCrashFlipModeActive` callers are renamed for the LOCAL
overrides.

## Build

### Linux

```bash
./setup.sh
cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release
cmake --build build-linux -j$(nproc)
```

### Windows (cross-compile from Linux/WSL)

```bash
sudo apt install mingw-w64 gcc-mingw-w64-x86-64-posix g++-mingw-w64-x86-64-posix cmake
./setup.sh

# Standalone UDP server
cmake -S . -B build-win-cmake \
  -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64-toolchain.cmake \
  -DSITL_TIME_MODE=UDP -DCMAKE_BUILD_TYPE=Release
cmake --build build-win-cmake -j$(nproc)

# In-process DLL
cmake -S . -B build-win-local \
  -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64-toolchain.cmake \
  -DSITL_TIME_MODE=UDP -DSITL_LINK_MODE=LOCAL -DCMAKE_BUILD_TYPE=Release
cmake --build build-win-local --config Release -j$(nproc)
```

The Windows binaries need `libwinpthread-1.dll`. The LOCAL DLL also needs
`libgcc_s_seh-1.dll` and `libstdc++-6.dll` next to it. The CI job
`betaflight-sitl-windows-full` collects them.

### CMake options

| Option | Values | Default | Description |
|--------|--------|---------|-------------|
| `SITL_TIME_MODE` | `REALTIME` / `UDP` | `REALTIME` | Scheduler time base |
| `SITL_LINK_MODE` | `UDP` / `LOCAL` | `UDP` | Server or in-process DLL |
| `SITL_GYRO_HZ` | 100-32000, `0` | `0` | Loop rate; `0` = 4000 for LOCAL, 1000 otherwise. Sets both task periods and the virtual gyro sample rate |
| `SITL_ATTITUDE_DIRECT` | flag | off | Inject the FDM quaternion directly instead of using `USE_IMU_CALC` |
| `DEFAULT_BLACKBOX_DEVICE` | flag | `VIRTUAL` | Fresh EEPROMs log blackbox to files |
| `SITL_BRUSHLESS_PWM_RATE` | Hz | 20000 | PWM rate used by config validation |

The loop rate is used in three places and they must agree: task periods,
`VIRTUAL_GYRO_SAMPLE_RATE_HZ`, and the host step (`1e6 / SITL_GYRO_HZ`
microseconds: 1000 us at 1 kHz, 250 us at 4 kHz). `BF_SITL_GYRO_HZ` overrides
the rate at boot (100-10000) and re-derives the sample rate and filter chains.
The boot `rates:` line in `sitl-audit.log` records what the firmware actually
derived.

### CI

`.github/workflows/build.yml` builds on push/PR:

- `betaflight-sitl-linux`
- `betaflight-sitl-windows`
- `betaflight-sitl-windows-full`

## Running

Standalone server:

```bash
./betaflight_SITL --ip 127.0.0.1 --gpx
```

| Option | Meaning |
|--------|---------|
| `--ip <addr>` | Where to send motor packets (default `127.0.0.1`) |
| `--config <file>` | Load a CLI config file, save it, exit |
| `--gpx` | Write `sitl_track.gpx` |
| `--help` | Usage |

The first run creates `eeprom.bin` (32 KiB) in the working directory.

### Windows quick start

Double-click `start-bf.cmd` (or run `start-bf.ps1`). It starts
`build-win-cmake\betaflight_SITL.exe` if needed, starts `bfweb-server.mjs` on
port 8080 and opens the configurator. The server injects a small script that
presets manual mode with `ws://127.0.0.1:6761` and disables the service worker,
so the preset applies on every load.

Build the configurator once:

```bash
cd bf-configurator
npm ci
npm run build
```

## Time base modes

### REALTIME

The stock scheduler busy-waits to the gyro deadline. Timing is exact but the
busy-wait uses about one CPU core.

### UDP

The virtual clock follows the `fdm_packet` timestamps on 9003:

- Timestamps are accumulated and consumed in 100 us quanta.
- A packet's time is committed only after its sensor data has been written.
- With no packets the clock freezes: the loop idles at ~0% CPU, failsafe timers
  do not advance, and the MSP link stays connected.
- RC packets on 9004 are still processed while idle. The CLI entry guard uses
  real time.
- A single delta is capped at 5 s so a stale packet cannot jump the clock.

### LOCAL

The engine calls `sitl_local_step()` from its own tick (250 us for the default
4 kHz loop). There is no UDP in the control path.

```c
sitl_local_init();
while (physicsTick) {
    sitl_local_input_t in = /* quaternion, body rates, accel, RC, battery, RPM */;
    sitl_local_output_t out;
    sitl_local_step(&in, 250, &out);
    // out.pwm_output_raw[0..motor_count-1]   motors, 1000..2000
    // out.servo_output_raw[0..servo_count-1] fixed-wing surfaces
}
sitl_local_shutdown();
```

`sitl_local_step()` writes the virtual sensors, advances the clock in scheduler
quanta that divide the gyro period, runs the scheduler and returns the motor
outputs for that same state. The quantum is the largest power of two <= 100 us
that divides `gyro.targetLooptime` (100 us at 1 kHz, 50 us at 4 kHz). RC uses
a latest-value cache with a fixed 125 Hz frame cadence (one COMPLETE frame per
8 ms, PENDING in between), which matches a real receiver for feedforward and
RC smoothing. The link forces the UDP RX provider and the ADC battery meters at
boot, so RC and voltage work regardless of the stored EEPROM.

## LOCAL DLL API

### Inputs and outputs

`sitl_local_input_t` mirrors `fdm_packet` plus telemetry:

- orientation quaternion, body rates (rad/s), linear acceleration, velocity,
  position
- `rc_channels[16]` (AETR + aux, 1000..2000)
- `battery_voltage` (V), `battery_current` (A)
- `motor_rpm[4]`, `motor_temperature[4]` (degC; <= 0 keeps the previous value)

`sitl_local_output_t` carries `pwm_output_raw[]`, `servo_output_raw[]`,
`motor_count`, `servo_count` and `armed`.

### Motor count and mixer

`motor_count`/`servo_count` follow the active mixer: QUADX = 4 motors,
AIRPLANE/FLYING_WING = 1 motor + 6/2 servos, HEX6 = 6 motors. Change the mixer
in the configurator (or CLI `mixer <name>`) and save/reboot; the DLL re-applies
mixer, motor and servo setup between steps.

### Telemetry

RPM and ESC temperature enter the firmware through the DShot telemetry bridge.
They feed the RPM filter, dynamic idle, blackbox eRPM, `MSP_MOTOR_TELEMETRY`,
`MSP_ESC_SENSOR_DATA` and the OSD over-temperature alarm. The OSD's numeric ESC
temperature element reads the ESC-sensor store, which the SITL does not fill.

The virtual battery meters are pinned before `batteryInit()`/`tasksInit()` so
`TASK_BATTERY_VOLTAGE`, `TASK_BATTERY_CURRENT` and `TASK_BATTERY_ALERTS` are
enabled even when the stored EEPROM says `NONE`. Battery presence is probed
from the 50 Hz voltage task until the cell count exists, including while armed.

### OSD

With `vcd_video_system = HD` the firmware drives an MSP displayport. The LOCAL
link decodes it in-process:

| Export | Purpose |
|--------|---------|
| `sitl_local_get_osd_frame(&frame)` | Current canvas (cells, attributes, revision) |
| `sitl_local_get_osd_revision()` | Changes when the canvas changes |
| `sitl_local_get_osd_font_revision()` | Font revision, 0 while no complete font is cached |
| `sitl_local_get_osd_font(buf, size)` | 256 characters of 54 bytes (13824 bytes), or 0 |
| `sitl_local_osd_refresh()` | Force a redraw |

The Configurator's font upload is captured and stored as
`<eeprom directory>\osd_font.bin`, so each aircraft keeps its own font. SD
(MAX7456) OSD output is not captured.

### Modes

Mode conditions are addressed by their permanent ID (`0` ARM, `1` ANGLE,
`2` HORIZON, `28` AIR MODE, `29` 3D DISABLE, `35` FLIP OVER AFTER CRASH).
Ranges use 25 us steps from 900 us.

| Export | Purpose |
|--------|---------|
| `sitl_local_get_mode_range(id, &aux, &start, &end)` | Read the first condition; `aux = 0xFF` if unset |
| `sitl_local_set_mode_range(id, aux, start, end)` | Replace the box's conditions, persist; `aux = 0xFF` clears |
| `sitl_local_get_mode_active(id)` | 1 while the box is active |
| `sitl_local_is_crash_flip_active()` | 1 while crash flip is latched |
| `sitl_local_get_3d_enabled()` / `set_3d_enabled()` | `FEATURE_3D`, re-derives boxes and mixer profile |

Crash flip is latched when the craft arms with the box already on, and cleared
by disarming or switching the box off. Enabling the box after arming does
nothing. The LOCAL motor backend is PWM, so the LOCAL build reproduces this
latch itself and feeds the mixer and OSD through `sitlIsCrashFlipModeActive()`.

`isRangeActive()` treats the end step as exclusive. A `1500..2000 us` condition
does not match a channel value of 2000; bind `1500..2100` or drive 1750.

### Synchronous accessors

These read and write the same state the MSP handlers use.

| Export | Purpose |
|--------|---------|
| `sitl_local_get_arming_flags()` | Arming-disable bitmask (as `MSP_STATUS_EX`) |
| `sitl_local_is_arming_disabled()` | True while arming is blocked |
| `sitl_local_get_armed()` | Armed state |
| `sitl_local_disarm()` | Disarm (idempotent; release the ARM channel too) |
| `sitl_local_get_flight_modes()` | `flightModeFlags` |
| `sitl_local_get_rate(index, rcRate[3], rcExpo[3], superRate[3])` | Per-axis rates in Rates-tab units |
| `sitl_local_set_rate(...)` | Write rates, persist via the background thread |
| `sitl_local_get_rate_mode()` / `set_rate_mode(mode)` | 0 BETAFLIGHT, 1 RACEFLIGHT, 2 KISS, 3 ACTUAL, 4 QUICK |
| `sitl_local_get_arm_switch(&aux, &start, &end)` | ARM condition |
| `sitl_local_set_arm_switch(aux, start, end)` | Bind or clear the ARM condition |
| `sitl_local_get_mode_range()` / `set_mode_range()` | Any mode box |
| `sitl_local_get_mode_active()` / `is_crash_flip_active()` | Active/latched state |
| `sitl_local_get_3d_enabled()` / `set_3d_enabled()` | 3D feature |
| `sitl_local_set_blackbox_dir(path)` | Per-aircraft blackbox folder |
| `sitl_local_set_blackbox_max_logs(n)` | Log cap at runtime |
| `sitl_local_get_blackbox_dir(buf, size)` | Current folder |
| `sitl_local_set_eeprom_path(path)` | Queue an EEPROM switch |
| `sitl_local_reload_config()` | Queue a reload of the current EEPROM |
| `sitl_local_shutdown()` | Stop the FC and release the ports |

## Reboots

### Standalone (UDP)

`systemReset()` closes the blackbox, saves when required, and spawns a hidden
child (`BF_SITL_REBOOT_CHILD=1`). The child waits 2 s for the ports, then
starts a fresh FC. The configurator reconnects in ~2-3 s.

### LOCAL

All reboot paths (Save and Reboot, CLI `save`/`exit`/`defaults`, CMS, Enter
bootloader/DFU) use one handler:

1. Immediately: close the blackbox, disarm, clear CLI and
   `REBOOT_REQUIRED` arming flags, and close the configurator's TCP/WebSocket
   sessions. The configurator sees a real disconnect and reconnects.
2. On the next `sitl_local_step()`: persist the RAM config, apply a pending
   EEPROM path, re-pin the LOCAL overrides (UDP RX, ADC battery meters, PWM
   motor backend), then re-apply mixer/motor/servo, filters (only when their
   settings changed) and the `initPhase3` modules.

The EEPROM is not re-read on Save and Reboot: it was just written from RAM, and
`activateConfig()` would rebuild the RC/PID chains under a live loop. Reloads
use `loadEEPROM()` (values only) and rebuild only what changed.

Simulator behaviour to be aware of: the gyro filter state is kept across a
reboot (zeroing it injects a step the D-term amplifies), the virtual clock keeps
running, and `ARMING_DISABLED_BOOT_GRACE_TIME` is not re-armed.

## Configuration

Settings live in `eeprom.bin` (32 KiB):

- **Save**: configurator Save or CLI `save` (`MSP_EEPROM_WRITE`).
- **Save and Reboot**: writes the EEPROM, then reboots.
- **Load**: at startup.
- **Reset**: CLI `defaults` + `save`.
- **Import/export**: CLI `diff`/`dump`; `--config <file>`.

The file is created in the working directory, then the executable folder, then
`%LOCALAPPDATA%\Betaflight-SITL`, then `%TEMP%`. Set `BF_SITL_EEPROM` to use a
specific file.

Per-aircraft EEPROM (LOCAL):

```c
sitl_local_set_eeprom_path("E:\\MySim\\Aircraft\\747\\eeprom.bin");
sitl_local_reload_config();   // both queue work for the next step
```

The aircraft being left is saved to its own file. A missing file is created
with factory defaults. Both calls are deferred while armed.

### What a Save does

The configurator writes its `MSP_SET_*` blocks and then `MSP_EEPROM_WRITE`. The
LOCAL link re-pins its runtime overrides on the next step (virtual receiver,
`FEATURE_RX_UDP`, ADC battery meters, PWM motor backend, DShot telemetry flag)
without rebuilding the gyro/PID/filter chains unless the settings changed.

`sitl-audit.log` records the applied state: `save state:`, `re-pin after config
write:` and periodic `state` lines with config hashes, loop periods, battery
state and filter settings.

## Protocols

Structs are defined in
`extern/betaflight/src/platform/SIMULATOR/target/SITL/target.h`.

### `fdm_packet` (9003, 144 bytes)

| Field | Type | Notes |
|-------|------|-------|
| `timestamp` | double | Seconds; drives the UDP clock. Must be monotonic. `0` is accepted by the Unreal bridge, which fills in its own clock |
| `imu_angular_velocity_rpy[3]` | double | rad/s |
| `imu_linear_acceleration_xyz[3]` | double | m/s^2, body frame |
| `imu_orientation_quat[4]` | double | w, x, y, z |
| `velocity_xyz[3]` | double | m/s, earth frame |
| `position_xyz[3]` | double | m / lat / lon / alt |
| `pressure` | double | Pa |

### Extended telemetry (optional tail)

| Offset | Type | Field |
|--------|------|-------|
| 144 | double | battery voltage (V) |
| 152 | double | battery current (A) |
| 160 | double[4] | motor RPM |
| 192 | double[4] | ESC temperature (degC) |

Total 224 bytes. A 144-byte sender still works: voltage defaults to 16.8 V,
current and RPM to 0, temperature to 25 degC.

### `rc_packet` (9004)

```c
typedef struct {
    double timestamp;                              // seconds
    uint16_t channels[SIMULATOR_MAX_RC_CHANNELS];  // 16 channels, 1000-2000
} rc_packet;
```

Map normalized inputs with `1500 + input * 500`.

### Servo packets (9001 / 9002)

```c
typedef struct {
    float motor_speed[4];   // 9002: 0..1, -1..1 in 3D mode
} servo_packet;

typedef struct {
    uint16_t motorCount;
    float pwm_output_raw[SIMULATOR_MAX_PWM_CHANNELS];
} servo_packet_raw;
```

Motor packets are sent once per PID iteration, i.e. only while the virtual
clock advances.

## Blackbox

The virtual blackbox writes standard `.BFL` logs.

- Fresh EEPROMs default to `blackbox_device = VIRTUAL`.
- The UDP build writes `LOG00001.BFL` to the working directory.
- The LOCAL build uses `%LOCALAPPDATA%\Betaflight-SITL\blackbox`.
- The folder keeps the newest 10 logs (`BF_SITL_BLACKBOX_MAX_LOGS`, `0` =
  unlimited); survivors are renumbered so the highest number is the newest.
- `blackbox_mode = NORMAL` records while armed, `ALWAYS` from boot.
- `blackbox_sample_rate` selects 1/1 to 1/16 of the PID rate (default 1/4).

Per-aircraft folder and cap at runtime: `sitl_local_set_blackbox_dir()`,
`sitl_local_set_blackbox_max_logs()`, `sitl_local_get_blackbox_dir()`.

Reboot closes the log first, so Save and Reboot does not truncate it.

## CHIRP

`USE_CHIRP` is enabled for system identification:

1. `set debug_mode = CHIRP` and `save`.
2. Bind a switch to the CHIRP mode.
3. Arm, then enable the switch. Each axis sweeps in turn (roll, pitch, yaw).
4. Disarm and open the log; `debug[0..3]` carry phase, axis, frequency and
   excitation.

Chirp parameters (`chirp_frequency_start_deci_hz`, `..._end_deci_hz`,
`chirp_time_seconds`, per-axis amplitudes) are recorded in the log header.
The LOCAL build re-applies `debug_mode` and changed filter settings on save.

## Environment variables

| Variable | Purpose |
|----------|---------|
| `BF_SITL_EEPROM` | EEPROM file path (default `eeprom.bin`) |
| `BF_SITL_GYRO_HZ` | Loop-rate override, 100-10000 |
| `BF_SITL_AUDIT_LOG` | Audit log path (default `%LOCALAPPDATA%\Betaflight-SITL\sitl-audit.log`) |
| `BF_SITL_BURST` | `1` enables the burst recorder (default off) |
| `BF_SITL_BURST_LOG` | Burst log path |
| `BF_SITL_BURST_HZ` / `..._PERIOD_S` / `..._FULL` | Burst rate, dump period, per-loop records |
| `BF_SITL_BLACKBOX_DIR` | Blackbox folder |
| `BF_SITL_BLACKBOX_MAX_LOGS` | Rolling window size (default 10) |
| `BF_SITL_BLACKBOX_FLUSH_MS` | Blackbox flush interval (default 100 ms) |
| `BF_SITL_TCP_BASE` | Base of the TCP block (default 5760, MSP on 5761) |
| `BF_SITL_WS_PORT` | WebSocket port (default 6761) |
| `BF_SITL_DT_SNAP` | `0` disables snapping a near-grid host step |
| `BFWEB_PORT` | Local web configurator port (default 8080) |
| `BF_SITL_REBOOT_CHILD` | Internal auto-restart marker |

## Troubleshooting

### Crash log (LOCAL)

A vectored exception handler writes fatal exceptions to
`%LOCALAPPDATA%\Betaflight-SITL\sitl-crash.log` (or `BF_SITL_AUDIT_LOG`):

```
code=0xC000001D addr=0x… module=0x… rva=0x… (addr2line …)
```

The DLL is built with `-g`, so:

```
addr2line -f -C -e betaflight_SITL.dll 0x<rva>
```

Only fatal codes are logged (access violation, illegal instruction, arithmetic
faults, stack overflow, heap corruption, fastfail). Debugger and language
pseudo-exceptions such as `0x40010006`, `0x406D1388` and `0xE06D7363` are
ignored. The handler uses a pre-opened handle, static buffers and one
`WriteFile`, with a per-thread re-entrancy guard, and always returns
`EXCEPTION_CONTINUE_SEARCH` so the host's crash reporter still sees the fault.

### Common notes

- `sitl-launch.out.log` / `sitl-launch.err.log` and `bfweb-server.*.log` hold
  launcher output.
- When the last TCP client disconnects in CLI mode, the link injects
  `exit noreboot` so the CLI ends without terminating the process.
- The Chinese locale logs a harmless `zh/messages.json` 404 and falls back to
  English.
- Chrome/Edge 147+ require the WebSocket handshake to echo the `binary`
  subprotocol; the proxy does this.
- If the host loads an old DLL, check that the new
  `betaflight_SITL.dll` is next to the executable and restart the host.

## Tools

| Tool | Purpose |
|------|---------|
| `sitl_local_save_compare` | Save/save-reboot/CLI/OSD/battery/mode/crash-flip comparisons and benchmarks |
| `run-save-matrix.ps1` | Full save matrix (52 cases) |
| `sitl_local_host` | 1 kHz host for testing the configurator without an engine |
| `sitl_osd_check` | Dump the OSD canvas; `--reinit` checks the boot splash |
| `sitl_local_reconnect_probe` | MSP TCP reconnect after a reboot |
| `ws_reboot_reconnect_probe.mjs` | WebSocket reconnect path used by the configurator |
| `analyze_gyro_noise.py` | Filter/PID analysis from burst logs |

Run the matrix:

```powershell
pwsh -File tools\run-save-matrix.ps1
```

The harness uses its own port block (`BF_SITL_TCP_BASE=15760`,
`BF_SITL_WS_PORT=16761`) and `%TEMP%` files, so it can run next to a live host.
