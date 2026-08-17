# BF-SITL-UDP

Betaflight SITL for Windows and Linux, with two build flavors sharing one
codebase: a standalone UDP server (`SITL_LINK_MODE=UDP`) and an in-process
DLL for engines that want the flight controller inside their own process
(`SITL_LINK_MODE=LOCAL`). It keeps the official Betaflight SITL target intact
(the `extern/betaflight` submodule is pinned and never modified) and layers
all simulator plumbing on top: a Winsock UDP/TCP layer, an optional
FDM-packet-driven virtual clock, a WebSocket bridge for the web configurator,
config persistence, automatic process restart on firmware reboots (standalone)
or in-process reboot recovery (DLL), and virtual blackbox logging.

The main use case is coupling Betaflight to an external physics simulator
(e.g. an Unreal Engine project) over UDP, while still using the stock
Betaflight Configurator for tuning.

## Feature highlights

- Two compile-time scheduler modes:
  - `REALTIME`: the official scheduler busy-waits to the exact gyro deadline.
  - `UDP`: the virtual clock is driven by FDM packet timestamps, so
    gyro/filter/PID run at 1 kHz of *simulator* time while the process uses a
    few percent of one CPU core and idles at ~0% when no packets arrive.
- Default 1 kHz gyro/filter/PID loop, overridable at build time or runtime.
- MSP over TCP 5761, built-in WebSocket proxy on 6761, and an optional local
  web configurator on 8080 (offline, no VPN needed).
- Full config persistence: Save, Save-and-Reboot, CLI `save`, `defaults`,
  `diff`/`dump`, and `--config` file import.
- Firmware reboots ("Save and Reboot", CLI `save`/`exit`) relaunch the SITL
  process automatically; the connection comes back after ~2 s.
- Virtual blackbox enabled by default: logs are written to `LOG00001.BFL`
  in the working directory and can be opened in the configurator's Blackbox
  tab.
- Windows-only fixes for real-world pitfalls: non-inheritable sockets (no
  duplicate listeners after auto-restart), a lock-free motor-update path, and
  clean blackbox shutdown on reboot.

## Ports

| Port | Direction | Protocol | Purpose |
|------|-----------|----------|---------|
| 9001 | out | UDP `servo_packet_raw` | Motor outputs (raw PWM bridge format) |
| 9002 | out | UDP `servo_packet` | Motor outputs (normalized Gazebo format) |
| 9003 | in  | UDP `fdm_packet` | Flight dynamics / IMU state (drives the virtual clock in UDP mode) |
| 9004 | in  | UDP `rc_packet` | RC channel inputs |
| 5761 | both| TCP | MSP / CLI (UART1) |
| 6761 | both| WebSocket | Configurator bridge to TCP 5761 |
| 8080 | -   | HTTP | Optional local Betaflight Configurator (start-bf) |

## Repository layout and submodule policy

```
CMakeLists.txt            build system, all Betaflight symbol renames
src/
  main_windows.c          Windows entry point and scheduler run loop
  wincompat.c             POSIX shims, virtual clock, auto-restart, systemReset
  win_socket_util.h       socket handle-inheritance fix
  serial_tcp_win.c        Winsock MSP serial bridge (TCP 5761)
  ws_proxy_win.c          WebSocket proxy (6761 -> 5761)
  udplink_windows.c       Winsock UDP transport + FDM clock hooks
cmake/
  mingw-w64-toolchain.cmake
bfweb-server.mjs          local web configurator server (8080)
start-bf.cmd / .ps1       one-click launcher (Windows)
extern/betaflight/        pinned Betaflight submodule - DO NOT EDIT
```

Because the submodule must stay pristine, almost every customization is done
with per-file CMake `-D` symbol renames. For example:

- `sitl.c` time functions (`micros`, `millis`, `delayMicroseconds`, ...) are
  renamed to `sitl*` so `wincompat.c` can provide stepped virtual time.
- `sitl.c`'s `systemReset` is renamed to `sitlSystemResetNative` so a custom
  `systemReset()` can save config and auto-restart first.
- `msp.c`'s `systemReset` is renamed to `sitlSystemReset` (persist EEPROM
  before rebooting), and `msp_serial.c`'s `millis` to `sitlMspMillis`
  (real-time CLI guard while the virtual clock is frozen).

## Build

### Linux (native)

```bash
./setup.sh                                # init submodules
cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release
cmake --build build-linux -j$(nproc)
```

### Windows (cross-compile from Linux/WSL)

```bash
sudo apt install mingw-w64 gcc-mingw-w64-x86-64-posix g++-mingw-w64-x86-64-posix cmake
./setup.sh
cmake -S . -B build-win-cmake \
  -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64-toolchain.cmake \
  -DSITL_TIME_MODE=UDP \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-win-cmake -j$(nproc)
```

The Windows executable needs `libwinpthread-1.dll` next to it. GCC/C++ runtime
DLLs are statically linked; GitHub Actions collects everything into the
`betaflight-sitl-windows-full` artifact.

### CMake options

| Option | Values | Default | Description |
|--------|--------|---------|-------------|
| `SITL_TIME_MODE` | `REALTIME` / `UDP` | `REALTIME` | Scheduler time base |
| `SITL_LINK_MODE` | `UDP` / `LOCAL` | `UDP` | `UDP` builds the standalone server; `LOCAL` builds an in-process DLL (`betaflight_SITL.dll`) with a synchronous step API |
| `SITL_GYRO_HZ` | 100-10000 | `1000` | Gyro/filter/PID frequency |
| `SITL_ATTITUDE_DIRECT` | defined / not defined | not defined | When defined manually (compile flag), the FDM quaternion is injected as attitude and the onboard estimator is bypassed. The default keeps `USE_IMU_CALC` on: attitude is estimated by the firmware's Mahony filter from the virtual accelerometer/gyroscope/magnetometer feeds. |
| `DEFAULT_BLACKBOX_DEVICE` | (defined) | `BLACKBOX_DEVICE_VIRTUAL` | Fresh EEPROMs log blackbox to files by default |
| `SITL_BRUSHLESS_PWM_RATE` | Hz | `20000` | Virtual brushless PWM rate used by config validation; raised so "sync PWM with PID" mode does not force `pid_process_denom` up |

`SITL_GYRO_HZ` can also be overridden at runtime with `BF_SITL_GYRO_HZ`.

### CI

`.github/workflows/build.yml` builds Linux and Windows binaries on push/PR:

- `betaflight-sitl-linux` - native Linux executable
- `betaflight-sitl-windows` - Windows executable only
- `betaflight-sitl-windows-full` - executable + `libwinpthread-1.dll`

## Running

The executable is the official Betaflight SITL server:

```bash
./betaflight_SITL --ip 127.0.0.1 --gpx
```

- `--ip <address>`: IP address to send motor outputs to (default `127.0.0.1`)
- `--config <file>`: load a CLI config file, save it to EEPROM, then exit
- `--gpx`: write a GPS track to `sitl_track.gpx`
- `--help`, `-h`: show usage

The first run creates `eeprom.bin` (32 KiB) in the working directory.

### Quick start (Windows)

Double-click `start-bf.cmd` (or run `start-bf.ps1`), which:

1. Starts `build-win-cmake\betaflight_SITL.exe` hidden if it is not running.
2. Starts `bfweb-server.mjs` on `http://127.0.0.1:8080` (if not already
   running; override the port with `BFWEB_PORT`).
3. Opens the configurator in the default browser.

`bfweb-server.mjs` serves a locally built `bf-configurator/src/dist` and
injects a small script into `index.html` that presets the connection
settings: manual connection mode, WebSocket URL `ws://127.0.0.1:6761`, and
automatic development options disabled. No files from `bf-configurator` are
modified, and the configurator's service worker is neutralized so the preset
always applies. A page served from 127.0.0.1 connecting back to 127.0.0.1 is
exempt from browser local-network permission prompts.

To build the local configurator once (needed for `start-bf`):

```bash
cd bf-configurator
npm ci
npm run build
```

## Time base (scheduler) modes

### REALTIME (default)

The official Betaflight scheduler busy-waits to the exact gyro deadline.
Gyro/filter/PID are locked at `SITL_GYRO_HZ` (default 1 kHz). Timing is exact
to the microsecond but the busy-wait consumes about one CPU core.

### UDP (FDM packet-driven)

The virtual clock is driven by `fdm_packet` timestamps arriving on UDP 9003:

- Each packet's timestamp delta is accumulated and consumed in 100 us quanta,
  so gyro/filter/PID fire exactly once per 1000 us of simulator time at
  default settings.
- The receive thread writes the virtual sensors for a packet before it commits
  that packet's time delta, so the flight loop never consumes time that has no
  matching IMU/sensor data (a packet's time becomes available only after its
  sensors are written).
- While packets arrive the flight loop runs at the configured rate and CPU
  use stays low (a few percent of one core).
- When no packets arrive the virtual clock freezes: the flight loop idles at
  ~0% CPU, failsafe/signal-loss timers do not expire, and the serial/MSP link
  stays alive so the configurator remains connected.
- RC frames on 9004 are still consumed while idle, and the CLI entry guard
  uses real time, so CLI works even with a frozen virtual clock.
- A single FDM delta is capped at 5 s (Windows) so a stale packet cannot jump
  the clock; longer gaps simply mean the next packet continues from there.

```bash
cmake -S . -B build-win-cmake -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64-toolchain.cmake -DSITL_TIME_MODE=UDP
cmake -S . -B build-win-cmake -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64-toolchain.cmake -DSITL_TIME_MODE=REALTIME
```

### LOCAL (in-process library, zero UDP)

For engines that want the flight controller inside their own process (e.g.
Unreal's async physics tick at 1000 Hz), build the SITL as a DLL:

```bash
cmake -S . -B build-win-local -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64-toolchain.cmake \
  -DSITL_TIME_MODE=UDP -DSITL_LINK_MODE=LOCAL -DCMAKE_BUILD_TYPE=Release
cmake --build build-win-local --config Release -j$(nproc)
```

This produces `betaflight_SITL.dll` (exports `sitl_local_init`,
`sitl_local_step`, `sitl_local_time_us`, `sitl_local_shutdown`) plus a
`sitl_local_tester.exe` harness and a `sitl_local_host.exe` debug host that
keeps the FC stepping at 1000 Hz for configurator/MSP testing. The DLL needs `libwinpthread-1.dll`,
`libgcc_s_seh-1.dll` and `libstdc++-6.dll` next to it (GCC 13 on CI names
the same files under the 13-posix runtime).

The host calls the API synchronously - no UDP, no stale reads:

```c
sitl_local_init();                                  // boots the FC once
while (physicsTick) {
    sitl_local_input_t in = /* FDM state, same fields/conventions as fdm_packet */;
    sitl_local_output_t out;
    sitl_local_step(&in, 1000, &out);               // 1000 = 1 ms virtual step
    // out.pwm_output_raw[0..motor_count-1] are the motor PWM values (1000..2000)
}
```

`sitl_local_step()` feeds the virtual sensors, advances the virtual clock by
`dtUs` on the same 100 us quantum grid as UDP mode (so the non-realtime
tasks, including RX and failsafe, get scheduler time before each gyro
deadline), runs the scheduler and returns the motor outputs for that exact
state in the same call. RC channels are taken from
`in.rc_channels` (AETR + aux, 1000..2000). RC uses the AJ92/SimITL "latest
value cache" model plus a fixed 125 Hz frame cadence: the channel cache is
refreshed whenever the host data changes, and the frame status reports one
COMPLETE frame every 8 ms of virtual time (PENDING in between, stamped on
`lastRcFrameTimeUs`). RXLOSS is impossible by construction, and
feedforward/smoothing see the same ~125 Hz frame stream as a real receiver
instead of a 1 kHz duplicate-frame flood (which inflated setpoint-speed
impulses on stick snaps). `sitl_local_init()` also forces the UDP RX provider
and the ADC battery/current meters so RC and voltage work regardless of the
EEPROM configuration.

### Synchronous state access (no serial traffic)

For hosts that want to read or write the same flight-controller state the
configurator sees, without touching the MSP stream, the DLL exports a small
set of synchronous accessors. They read/write the exact same globals the MSP
handlers use, so both sides always agree:

| Export | Returns |
|--------|---------|
| `sitl_local_get_arming_flags()` | arming-disable bitmask (same value as MSP_STATUS_EX; 0 = may arm) |
| `sitl_local_is_arming_disabled()` | `true` while any arming-disable reason is blocking arming |
| `sitl_local_get_armed()` | `true` when armed |
| `sitl_local_get_flight_modes()` | `flightModeFlags` bitmask (ANGLE/HORIZON/MAG/...) |
| `sitl_local_get_rate(index, &rcRate, &rcExpo, &superRate, &yawRate)` | rate profile `index` (any out-of-range index = current profile); units match the Rates tab (rcRate/yawRate 0..2.5, rcExpo/superRate 0..1.0) |
| `sitl_local_set_rate(rcRate, rcExpo, superRate, yawRate)` | writes the current profile (same fields as MSP_SET_RC_TUNING, roll/pitch kept symmetric when they were equal) and persists it via the background thread |
| `sitl_local_get_rate_mode()` | rate mode of the current profile: 0 = BETAFLIGHT, 1 = RACEFLIGHT, 2 = KISS, 3 = ACTUAL, 4 = QUICK; -1 before init |
| `sitl_local_set_rate_mode(mode)` | sets the rate mode (same values) and persists it; returns 0, or -1 for an invalid mode |
| `sitl_local_get_arm_switch(&auxChannel, &startStep, &endStep)` | ARM mode condition: RC channel index (4 = AUX1) and the 25 us-step range; `auxChannel` is `0xFF` when no ARM switch is configured |

All accessors are plain memory reads (safe from the UE tick); the only
write, `sitl_local_set_rate()`, writes the RAM profile immediately and defers
the EEPROM persist to the background MSP thread, so no file I/O happens on
the UE thread. Arming itself still goes through the RC auxiliary channel -
drive the channel returned by `sitl_local_get_arm_switch()`.

Motor RPM from `in.motor_rpm[0..3]` (and the UDP extended tail) is bridged
into the firmware's DSHOT-telemetry consumers (`getDshotRpm`,
`getDshotRpmAverage`, `getDshotErpm`, `getMotorFrequencyHz`,
`getMinMotorFrequencyHz`), so the RPM filter (`USE_RPM_FILTER` is enabled in
LOCAL builds), dynamic idle, OSD and the configurator's motor telemetry all
see the simulated RPM. 4 motors are supported.

The virtual accelerometer feeds `in.linear_acceleration_xyz` (FRD specific
force, m/s²). If the host's feed is missing or not gravity-compensated
(magnitude outside 0.9..1.1 g), the DLL derives a healthy 1 g specific force
from the FDM attitude quaternion instead - otherwise Betaflight's Mahony
estimator disables accel correction and roll/pitch never converge.

The virtual EEPROM is stored at a fixed, writable location in LOCAL mode:
`%LOCALAPPDATA%\Betaflight-SITL\eeprom.bin` (override with `BF_SITL_EEPROM`).
This keeps configuration persistent no matter where the host process (UE) is
launched from. Plain "Save" works and persists; "Save and Reboot" also
persists but does not restart the in-process FC, so use plain Save or restart
the host session for a full reboot.

For save/connection troubleshooting, the DLL appends an audit trail to
`%LOCALAPPDATA%\Betaflight-SITL\sitl-audit.log`: the resolved EEPROM path at
init, WebSocket configurator connect/disconnect events, and every MSP
`writeEEPROM` that reaches the firmware (with the armed state at that moment).
If a Save produces no `writeEEPROM` entry, the command was rejected before
writing - the usual cause is saving while the FC is armed (Betaflight rejects
`MSP_EEPROM_WRITE` while armed), so disarm before saving.

The web configurator still works: boot keeps the TCP/WebSocket proxy on
127.0.0.1:5761/6761 and a background thread services MSP. Caveats:

- Attitude is estimated by the firmware's Mahony filter (`USE_IMU_CALC`) from
  the virtual acc/gyro/mag feeds. Defining `SITL_ATTITUDE_DIRECT` instead
  injects the FDM quaternion directly and disables the estimator.
- "Save and Reboot" persists the configuration but does not restart the
  in-process FC (there is no process to relaunch); restart the host session
  for a full reboot.
- Rebooting via the configurator (CLI `exit`, "Save and Reboot") keeps the
  MSP/CLI background thread alive: the LOCAL reboot handler persists the
  config and returns control to the thread instead of letting the stock
  `mspRebootFn` spin in its `while (true);` reset loop, so the configurator
  can reconnect immediately afterwards. Motor outputs also survive the
  reboot (`motorShutdown()` is a no-op in LOCAL mode - there is no MCU reset
  to stop the ESCs for), so arming still produces PWM after a CLI exit.
- "Enter bootloader / DFU" from the configurator does not kill the host:
  the stock `systemResetToBootloader()` calls `exit(0)`, which would
  terminate the process from a DLL. LOCAL mode treats it like the firmware
  reboot (persist + keep running) - there is no bootloader to enter.
- Leaving the CLI tab drops the configurator link by design: the
  configurator's own reboot flow (CLI `exit` + `MSP_REBOOT`) tears the
  "manual/WebSocket" connection down after a short flush delay, then waits
  for the user to reconnect (reconnection is instant with this build).
  Enabling Auto-Connect in the configurator makes it reconnect on its own.
- The MSP thread runs concurrently with the scheduler; configurator operations
  are infrequent, but they are not synchronized against the flight loop.

## Data flow / protocol

All structs are defined in
`extern/betaflight/src/platform/SIMULATOR/target/SITL/target.h`.

### fdm_packet (9003, 144 bytes = 18 doubles)

| Field | Type | Notes |
|-------|------|-------|
| `timestamp` | double | seconds; drives the virtual clock in UDP mode |
| `imu_angular_velocity_rpy[3]` | double | rad/s |
| `imu_linear_acceleration_xyz[3]` | double | m/s^2, body frame |
| `imu_orientation_quat[4]` | double | w, x, y, z |
| `velocity_xyz[3]` | double | m/s, earth frame |
| `position_xyz[3]` | double | m / lat / lon / alt |
| `pressure` | double | Pa (legacy bridges) |

The timestamp must increase monotonically (a `double` in seconds, not a
float). The Unreal bridge in FPVSkyline accepts `0` and fills in its own
monotonic clock automatically.

### Extended FDM packet (Windows UDP mode, optional)

The first 144 bytes stay the official `fdm_packet`. A sender may append
simulator telemetry that the SITL feeds into the virtual battery/RPM sensors:

| Offset | Type | Field |
|--------|------|-------|
| 144 | double | battery voltage (V) |
| 152 | double | battery current (A) |
| 160 | double[4] | motor RPM (per motor) |

Total extended size: 192 bytes. Senders that only send the official 144-byte
packet still work: voltage defaults to 16.8 V (4S), current to 0 A and RPM to
0, so the FC always sees a battery.

With the extended fields, the configurator and blackbox show real voltage,
current and mAh draw. Fresh EEPROMs default `battery_meter` and
`current_meter` to `ADC` (the UDP-fed path); existing EEPROMs need this once:

```
set battery_meter = ADC
set current_meter = ADC
save
```

The per-motor RPM is parsed and kept available, but Betaflight's RPM filter
and blackbox RPM fields require DSHOT telemetry, which the official SITL
target excludes on x86, so the firmware does not consume RPM yet.

### rc_packet (9004)

```c
typedef struct {
    double timestamp;                              // seconds
    uint16_t channels[SIMULATOR_MAX_RC_CHANNELS];  // 16 channels, 1000-2000
} rc_packet;
```

If the simulator produces normalized `-1..1` stick values, map them with
`1500 + input * 500` before sending.

### servo packets (9001 / 9002)

```c
typedef struct {
    float motor_speed[4];   // 9002: normalized [0,1], [-1,1] in 3D mode
} servo_packet;

typedef struct {
    uint16_t motorCount;                // 9001: number of motors (4)
    float pwm_output_raw[SIMULATOR_MAX_PWM_CHANNELS]; // raw PWM values
} servo_packet_raw;
```

Motor packets are emitted once per PID loop iteration, i.e. only while the
virtual clock is advancing. When disarmed, 9002 is all zeros and 9001 holds
the idle PWM (1000).

## Configuration persistence

Settings saved from the configurator are stored in `eeprom.bin` (32 KiB),
mirroring the FC's flash:

- **Save**: click Save in the configurator or type `save` in the CLI panel
  (sends `MSP_EEPROM_WRITE`).
- **Save and Reboot**: also works reliably - `MSP_REBOOT` writes the EEPROM
  first, then reboots.
- **Load**: automatic at startup from `eeprom.bin`.
- **Reset**: `defaults` then `save` in the CLI panel.
- **Text import/export**: `diff`/`dump` prints a text config;
  `betaflight_SITL --config <file>` loads it, saves it, and exits.

`eeprom.bin` is created in the working directory, falling back to the
executable's folder, then `%LOCALAPPDATA%\Betaflight-SITL`, then the temp
directory. Always launch from the same folder for one persistent config.

For multiple independent configurations, point `BF_SITL_EEPROM` at a
specific file (the directory is created automatically):

```powershell
$env:BF_SITL_EEPROM = "E:\sim\flight.bin"
.\betaflight_SITL.exe
```

### Automatic restart on firmware reboot (Windows)

This applies to the standalone UDP build. The LOCAL (DLL) build never
restarts the process: its reboot handler persists the config and keeps the
FC running in-process (see the LOCAL caveats above).

Any firmware reboot (`MSP_REBOOT`, CLI `save`, CLI `exit`, CMS save-exit)
no longer leaves the simulator dead:

1. `systemReset()` calls `blackboxFinish()` (clean log close), writes the
   EEPROM when the command is a save-type reboot, then spawns a hidden copy
   of the process (`BF_SITL_REBOOT_CHILD=1`).
2. The child waits 2 s for the old process to release all ports, then starts
   as the "reborn" FC with the same working directory and log files.
3. The configurator connection drops for ~2-3 s; reconnect manually if the
   page does not retry automatically.

Windows sockets are marked non-inheritable so the child does not inherit the
parent's listeners (which would leave connections landing on a socket nobody
accepts from). CLI semantics are preserved: `save` persists, plain `exit`
reboots without saving.

### Tuning note: configurator shows 999/333

The Setup page computes `pidHz = 1e6 / cycleTime` and `gyroHz = pidHz *
pid_process_denom`. On real hardware, `use_unsynced_pwm = OFF` with a 480 Hz
PWM protocol forces `pid_process_denom = 3`, so the PID loop runs at ~333 Hz
and the display reads `999/333` (the gyro itself is still 1000 Hz). This
build raises the virtual brushless PWM rate to 20000 Hz, so that limit never
applies: both `use_unsynced_pwm` settings keep a 1 kHz PID loop. If a saved
config still contains `pid_process_denom = 3`, set it back to 1 once (`set
pid_process_denom = 1` + `save`).

## Blackbox

The SITL target builds Betaflight's virtual blackbox device, which writes
standard `.BFL` logs to the working directory:

- Fresh EEPROMs default to `blackbox_device = VIRTUAL` (compile-time default).
- Existing EEPROMs keep their saved value; set it once with
  `set blackbox_device = VIRTUAL` + `save`.
- Logs are named `LOG00001.BFL`, `LOG00002.BFL`, ... (auto-incrementing).
- `blackbox_mode = NORMAL` (default) records while armed; `ALWAYS` records
  from boot without arming; `MOTOR_TEST` records during motor tests.
- `blackbox_sample_rate` selects 1/1, 1/2, 1/4, 1/8 or 1/16 of the PID rate
  (default 1/4, i.e. 250 Hz at a 1 kHz PID loop).

Workflow:

1. Fly (or set `blackbox_mode = ALWAYS` for bench recordings).
2. Disarm / stop - the log is flushed and closed.
3. Open the Betaflight Configurator's Blackbox tab and load
   `build-win-cmake\LOG00001.BFL` (or wherever the working directory is).

Recorded fields include loop iteration, gyro (filtered and unfiltered), PID
terms (P/I/D/F per axis), RC commands, setpoints, battery, motors, and the
IMU quaternion. Rebooting the SITL calls `blackboxFinish()` first, so a
Save-and-Reboot never truncates an open log.

## CHIRP auto-tuning support

The build enables Betaflight's `USE_CHIRP` excitation feature, which injects
a swept-sine test signal into the PID loop for offline system
identification / auto-tuning from blackbox logs:

1. `set debug_mode = CHIRP` + `save` (persisted).
2. In the configurator's Modes tab, assign a switch/aux channel to the
   **CHIRP** mode.
3. Arm, then flip the CHIRP switch. The FC sweeps each axis in turn
   (roll -> pitch -> yaw) using the chirp profile settings:
   `chirp_frequency_start_deci_hz` (default 2 -> 0.2 Hz),
   `chirp_frequency_end_deci_hz` (6000 -> 600 Hz), `chirp_time_seconds`
   (20 s), per-axis amplitudes and the lead/lag phase-compensation
   frequencies.
4. Disarm; open the blackbox log. The header records all `chirp_*`
   parameters and the log contains `debug[0..3]` CHIRP channels (phase,
   active axis, instantaneous frequency, raw excitation) for the analysis
   tool.

## Environment variables

| Variable | Purpose |
|----------|---------|
| `BF_SITL_EEPROM` | Path to the virtual EEPROM file (default `eeprom.bin` in CWD) |
| `BF_SITL_GYRO_HZ` | Runtime gyro/filter/PID frequency override (100-10000) |
| `BFWEB_PORT` | Port for the local web configurator (default 8080) |
| `BF_SITL_REBOOT_CHILD` | Internal marker for the auto-restart child process |

## Windows notes and troubleshooting

- The executable needs `libwinpthread-1.dll` next to it.
- `sitl-launch.out.log` / `sitl-launch.err.log` in `build-win-cmake` contain
  SITL output; `bfweb-server.*.log` in the repo root contain web server
  output. Logs continue into the same files after an auto-restart.
- When the last TCP client disconnects while the FC is in CLI mode, SITL
  injects `exit noreboot` so the CLI session ends without killing the
  process; the next connection starts clean.
- The web configurator logs a harmless `zh/messages.json 404` warning for
  the Chinese locale and falls back to English.
- Chrome/Edge 147+ require WebSocket handshakes to echo the `binary`
  subprotocol; the built-in proxy handles this automatically.

## Protocol reference

See the Betaflight SITL documentation for the complete `fdm_packet`,
`servo_packet`, and `rc_packet` definitions and the official simulation
bridge documentation.
