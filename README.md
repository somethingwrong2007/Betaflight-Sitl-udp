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
    // out.servo_output_raw[0..servo_count-1] are fixed-wing surface PWM values
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

**Motor telemetry (RPM + ESC temperature).** `in.motor_rpm[4]` and
`in.motor_temperature[4]` (degrees Celsius) are pushed into the firmware as
DSHOT/BLHeli telemetry, the only path the SITL can populate: RPM drives the
RPM filter, dynamic idle, blackbox eRPM and the `MSP_MOTOR_TELEMETRY` RPM
field, and the temperature shows up in the configurator's Motors tab
(`MSP_MOTOR_TELEMETRY`), in `MSP_ESC_SENSOR_DATA` (DJI FPV) and in the OSD ESC
over-temperature alarm (`esc_temp_alarm`, via `getDshotSensorData()`), clamped
to the 0..255 degC the telemetry field carries. The OSD's numeric ESC
temperature *element* reads the ESC-sensor store instead, which the SITL never
fills (there is no serial ESC telemetry), so that one element stays at 0.
`motor_temperature` is appended after `rc_channels`, so every earlier field
keeps its offset - but the struct does grow, so rebuild the host when the DLL
is replaced. Values <= 0 mean "no data" and keep the previous temperature;
25 degC is the startup default.

**Mixer type / motor count.** The mixer mode drives the output configuration:
`motor_count` and `servo_count` in `sitl_local_output_t` follow the current
mixer (QUADX = 4 motors, AIRPLANE/FLYING_WING = 1 motor + 6/2 servos, HEX6 =
6 motors, ...). Change it from the configurator (or CLI `mixer <name>`) and
`save`/reboot: the DLL re-applies the mixer, motor and servo setup on the
next `sitl_local_step` (on the host thread, between steps, so it never races
the flight loop). Fixed-wing is fully compiled in (`USE_SERVOS`,
`USE_UNCOMMON_MIXERS`), and `servo_output_raw[]` carries the surface PWM
after the motors.

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
| `sitl_local_disarm()` | put the aircraft back into the disarmed state (same call the configurator's Disarm button makes: `disarm(DISARM_REASON_ARMING_DISABLED)`). Idempotent, leaves no arming-disable flag behind, so it can arm again normally; returns 0, or -1 when the FC is not running. Arming is still driven by the RC ARM switch, so a host that keeps the ARM channel high must release it too or the scheduler arms again |
| `sitl_local_get_flight_modes()` | `flightModeFlags` bitmask (ANGLE/HORIZON/MAG/...) |
| `sitl_local_get_rate(index, rcRate[3], rcExpo[3], superRate[3])` | rate profile `index` (any out-of-range index = current profile); each array is per-axis in ROLL, PITCH, YAW order and in the same display units as the Rates tab for the current rate mode; pass NULL to skip a group |
| `sitl_local_set_rate(rcRate[3], rcExpo[3], superRate[3])` | writes the current profile per axis (same display units, converted and clamped per rate mode like the Rates tab) and persists it via the background thread; pass NULL to leave a group unchanged |
| `sitl_local_get_rate_mode()` | rate mode of the current profile: 0 = BETAFLIGHT, 1 = RACEFLIGHT, 2 = KISS, 3 = ACTUAL, 4 = QUICK; -1 before init |
| `sitl_local_set_rate_mode(mode)` | sets the rate mode (same values) and persists it; returns 0, or -1 for an invalid mode |
| `sitl_local_get_arm_switch(&auxChannel, &startStep, &endStep)` | ARM mode condition: RC channel index (4 = AUX1) and the 25 us-step range; `auxChannel` is `0xFF` when no ARM switch is configured |
| `sitl_local_set_arm_switch(auxChannel, startStep, endStep)` | bind BOXARM to an aux channel + 25 us-step range and persist it (same semantics as MSP_SET_MODE_RANGE); `auxChannel = 0xFF` clears the ARM condition; returns 0, or -1 for invalid parameters / no free slot |
| `sitl_local_set_blackbox_dir(path)` | redirect the blackbox log folder (e.g. one folder per aircraft); creates the directory and re-scans for correct log numbering; returns 0, or -1 for a NULL/empty/too-long path |
| `sitl_local_set_eeprom_path(path)` | queue a switch of the virtual EEPROM to another file (e.g. one per aircraft); an empty path restores the default `%LOCALAPPDATA%\Betaflight-SITL\eeprom.bin`. Applied by the next `sitl_local_step()`: the aircraft being left is saved into its own file first. A path that does not exist yet is created with factory defaults. Returns 0 when queued, or -1 for a NULL/too-long path |
| `sitl_local_reload_config()` | queue a re-read of the selected EEPROM (plus a pending path switch) and re-apply it: LOCAL link overrides (UDP RX, ADC battery shims, PWM motor backend), mixer/motor/servo setup, filters and the PG config. Runs inside the next `sitl_local_step()`, never while armed. Returns 0 when queued, or -1 when not initialised or currently armed |

The `get_*` accessors and `sitl_local_set_rate()`/`sitl_local_set_rate_mode()`
are plain memory reads/writes (safe from the UE tick); `sitl_local_set_rate()`
writes the RAM profile immediately and defers the EEPROM persist to the
background MSP thread, so no file I/O happens on the UE thread.
`sitl_local_set_eeprom_path()` and `sitl_local_reload_config()` only record a
request: the file I/O and the re-initialisation run inside the next
`sitl_local_step()`, between scheduler passes, so they can never tear the
flight loop apart. Arming itself still goes through the RC auxiliary channel -
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
simulator telemetry that the SITL feeds into the virtual battery and motor
telemetry:

| Offset | Type | Field |
|--------|------|-------|
| 144 | double | battery voltage (V) |
| 152 | double | battery current (A) |
| 160 | double[4] | motor RPM (per motor) |
| 192 | double[4] | motor ESC temperature (degC, per motor) |

Total extended size: 224 bytes. Senders that only send the official 144-byte
packet still work: voltage defaults to 16.8 V (4S), current to 0 A, RPM to 0
and ESC temperature to 25 degC, so the FC always sees a battery. A value <= 0
in the temperature block means "no data" and keeps the previous temperature.

With the extended fields, the configurator and blackbox show real voltage,
current and mAh draw. Fresh EEPROMs default `battery_meter` and
`current_meter` to `ADC` (the UDP-fed path); existing EEPROMs need this once:

```
set battery_meter = ADC
set current_meter = ADC
save
```

The per-motor RPM and ESC temperature are pushed into the firmware through the
DSHOT telemetry bridge (`getDshotRpm`/`getDshotErpm` and the DSHOT telemetry
state), which is where the RPM filter, dynamic idle, blackbox eRPM, the
configurator's Motors tab (`MSP_MOTOR_TELEMETRY`), `MSP_ESC_SENSOR_DATA` and
the OSD ESC over-temperature alarm read them from. The OSD's numeric ESC
temperature element instead reads the ESC-sensor store, which the SITL never
fills, so that element stays at 0.

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

### Per-aircraft EEPROM at runtime (LOCAL DLL)

The LOCAL link can move the virtual EEPROM between files while the host
process keeps running, so every aircraft can own its own `eeprom.bin`:

```c
sitl_local_set_eeprom_path("E:\\MySim\\Aircraft\\747\\eeprom.bin");
sitl_local_reload_config();
// both only queue work - the switch happens inside the next sitl_local_step()
```

Sequence and rules:

- `sitl_local_set_eeprom_path()` records the new path. On the next step the
  aircraft being left is written to the file it came from (so its tune is
  never lost) and that file is closed, then the new path is opened.
- A path that does not exist yet is created with **factory defaults** - a new
  aircraft does not silently inherit the previous one's rates/PID/mixer. The
  configurator then writes it like a fresh FC.
- `sitl_local_reload_config()` is optional after a path switch (the switch
  already re-reads the file); call it on its own to re-apply the current file,
  e.g. after changing it outside the process.
- Both are ignored while armed: the request stays queued and is applied on the
  first step after disarming (call `sitl_local_disarm()` first if the aircraft
  is airborne and you do not want to wait for the ARM switch). They do nothing
  before `sitl_local_init()`.
- Everything the boot path derives from the config is re-applied, including the
  LOCAL-mode overrides (virtual UDP receiver, ADC battery shims, virtual PWM
  motor backend) and the mixer/motor/servo setup, so the motor outputs keep
  working after a switch.
- The configurator connection (TCP 5761 / WebSocket 6761) is not affected; it
  simply shows the newly loaded aircraft after the next MSP poll.

### Automatic restart on firmware reboot (Windows)

This applies to the standalone UDP build. The LOCAL (DLL) build never
restarts the process: its reboot handler reproduces the reboot in-process
(see "Firmware reboot in-process" below).

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

### Firmware reboot in-process (LOCAL DLL)

Every reboot path - the configurator's **Save and Reboot** (`MSP_REBOOT` after
`MSP_EEPROM_WRITE`), CLI `save`/`exit`/`defaults`, CMS, and **Enter
bootloader/DFU** - funnels into one handler that reproduces, as closely as an
in-process DLL can, what real hardware does:

1. **Immediately** (on whatever thread asked): close the blackbox log, disarm
   (a real reboot drops the motor output - the very next `sitl_local_step()`
   already returns disarmed motor values, so rebooting while airborne stops
   the motors instead of leaving the old configuration flying), reset the
   arming state machines, drop the CLI mode and the `ARMING_DISABLED_CLI` /
   `ARMING_DISABLED_REBOOT_REQUIRED` flags.
2. **On the next `sitl_local_step()`**, between scheduler passes: persist the
   current RAM config, apply a pending `sitl_local_set_eeprom_path()` and
   re-apply every boot-derived setting - the LOCAL link overrides re-pin the UDP
   receiver, the ADC battery meters and the virtual PWM motor backend; then
   mixer/motor/servo, gyro filters (rebuilt only when their settings changed) and
   debug mode - and finally re-init the `initPhase3` modules (`blackboxInit()`,
   `gyroStartCalibration()`) plus the rest of `initPhase3` that the reload
   path cannot cover (`gyroSetTargetLooptime()` + `initDshotTelemetry()`,
   `initBoardAlignment()`, `imuInit()`, `failsafeInit()`, `mixerInitProfile()`
   with its dynamic-idle / VBAT-sag / RPM-limiter / ez-landing runtime,
   `positionInit()`, `autopilotInit()`), exactly like a boot.

   The EEPROM itself is **not** re-read on this path (the file was written from
   RAM a few lines earlier): re-reading it cannot change a value, but
   `readEEPROM()` ends in `activateConfig()`, which re-initialises the
   stick-transient chain (feedforward / setpoint smoothing, PID and RC state)
   under a live flight loop. That alone made the aircraft respond differently to
   the same sticks after a "Save and Reboot" - see
   "What a configurator Save does to a running LOCAL build (measured)" below.
   `sitl_local_reload_config()` and a runtime `sitl_local_set_eeprom_path()`
   follow the same rule: the selected file is read with `loadEEPROM()` (values
   only - no `activateConfig()`), the chains are rebuilt only where the loaded
   settings actually differ, and a file that matches the configuration the FC is
   already running makes the whole request a no-op (the usual "switch to another
   aircraft" case, which previously shook the aircraft for nothing).
3. The configurator connection survives the reboot (the MSP thread is moved
   back to its idle parser state instead of being left in `mspRebootFn()`'s
   `while (true);`). Expect a short RX re-acquisition right after the reboot,
   like real hardware.

Three deliberate deviations, all simulator-specific:

- The gyro filter chain (LPF1/LPF2, notches, dynamic notch, RPM filter) keeps
  its state across a reload/reboot and is only rebuilt when the filter
  settings themselves changed. The LOCAL gyro stream never stops, so zeroing
  that state injects a step into the filtered rate which the PID's D-term
  (`delta/dt`) amplifies into a full-authority spike - measured as motors
  slamming between their limits and reversing within a few ms right after a
  reload/reboot, i.e. the "violent shake after Save and Reboot" symptom. A
  real FC zeroes those filters on boot, but its motors are off and its gyro
  stream restarts, so it never sees that step. A change to the gyro filter
  settings still rebuilds the filters (logged to the audit log), exactly like
  a real FC applying a filter change live.

- The virtual clock is **not** reset (the host drives it and the scheduler
  anchors its deadline grid to it), so uptime/stats keep counting across a
  reboot.
- `ARMING_DISABLED_BOOT_GRACE_TIME` is **not** re-armed, so the craft stays
  immediately armable after a "Save and Reboot" instead of waiting out
  `powerOnArmingGraceTime`.

The gyro calibration a real boot performs is kept (`gyroStartCalibration()`),
but the virtual gyro reports it as complete immediately, so it never delays
arming here.

**Configurator writes cannot break the LOCAL link.** The configurator's Save
writes settings over MSP (feature bitset, battery/ESC meters, motor config,
mixer, board alignment, ...) without rebooting, which can undo the runtime
state the LOCAL link pins at boot: the virtual receiver takeover,
`FEATURE_RX_UDP`, the ADC battery meter sources, the virtual PWM motor protocol
and the DShot-telemetry flag. Every `MSP_EEPROM_WRITE` therefore re-pins those
overrides on the next `sitl_local_step()` (flags/mode sources only - no mixer,
motor or filter state, so it is safe while armed) and logs a before/after
fingerprint of the runtime state to
`%LOCALAPPDATA%\Betaflight-SITL\sitl-audit.log` (`save state: ...` followed by
`re-pin after config write: ...` when something had to be restored).

On top of that, a **state flight recorder** watches everything control-relevant
- config hashes (system/PID/rates/gyro/motor/battery/mixer/features/RX/IMU/acc),
the applied runtime values (motor protocol and output range, idle, feature
*runtime* mask, battery meter sources, virtual-receiver takeover ownership,
mixer config vs runtime, `pid_process_denom` / `targetLooptime` / `pidDT`, gyro
filter and notch settings, current PID/rates) - once per ~1000 steps and after
every save, and writes two `state ...` lines to the same audit log whenever any
of it changes. Diffing those two lines pinpoints what a configurator write (or
anything else) changed at runtime, including state no configurator page shows.

### What a configurator Save does to a running LOCAL build (measured)

`sitl_local_save_compare` replays one deterministic scenario (armed, 1500
throttle, roll stick doublet plus a fixed gyro waveform) before and after a save
on the *same* flight controller and diffs the motor outputs sample by sample:

```
sitl_local_save_compare save-only       plain "Save" (MSP_EEPROM_WRITE)
sitl_local_save_compare save-reboot     "Save and Reboot" (+ MSP_REBOOT)
sitl_local_save_compare cfg-sets        the configurator's MSP_SET_* burst only
sitl_local_save_compare cfg-save        that burst + MSP_EEPROM_WRITE
sitl_local_save_compare cfg-save-reboot that burst + MSP_EEPROM_WRITE + MSP_REBOOT
sitl_local_save_compare cfg-change      flips a real filter byte and writes it back
sitl_local_save_compare api-only        MSP round trip, no EEPROM write (bisect)
sitl_local_save_compare control         no MSP traffic at all (baseline)
sitl_local_save_compare sensitivity     no save; doubles the gyro noise instead
```

`cfg-*` replays the configurator's writes byte for byte - it reads each settings
block over MSP and writes the very same bytes back - so the firmware handlers the
configurator drives are exercised, and every block is read back afterwards to
prove the config did not change. `cfg-sets 0..3` replays a single block (bisect).

Measured before the fixes below (motor PWM, 1000..2000 us):

| mode | before vs after | after vs after (repeat) |
| --- | --- | --- |
| `control` | identical (max 0.0009 us) | identical |
| `api-only` | identical (max 0.0009 us) | identical |
| `save-only` | changed by up to 153 us | identical |
| `save-reboot` | changed by up to 153 us | identical |
| `cfg-sets` | **changed by up to 318 us** | identical |
| `cfg-sets 0` (`MSP_SET_RC_TUNING`) | changed by up to 153 us | identical |
| `cfg-sets 1` (`MSP_SET_FILTER_CONFIG`) | **changed by up to 318 us** | identical |
| `cfg-sets 2/3` (`MSP_SET_PID_ADVANCED` / `MSP_SET_PID`) | identical | identical |
| `sensitivity` (2x gyro noise) | changes by 10.6 us | identical |

A Save did **not** change the steady state - the traces were bit-identical while
the stick was still - but the response to a stick input was different afterwards:
the difference started exactly at the stick step, peaked at ~153 us (~15% of the
output range, 318 us for the filter block) and decayed as the transient settled.
Repeating the after-trace
reproduced the new response exactly, so a save left the loop on a *different but
stable* operating point rather than on a transient that just needed longer
settling: after any Save the aircraft responded differently to the sticks until
the process was restarted.

Where it comes from: a configurator Save is `MSP_EEPROM_WRITE`, and the firmware
handler does

```c
writeEEPROM();
readEEPROM();        // msp.c: writeReadEeprom()
```

`readEEPROM()` re-reads the EEPROM and calls `activateConfig()`, which
re-initialises runtime state *while the flight loop keeps running*:
`initRcProcessing()` (resets the feedforward / setpoint-smoothing chain and the
derived rate tables), `pidInit()`, `rcControlsInit()`, `failsafeReset()`,
`accInitFilters()`, `initActiveBoxIds()`. On real hardware that only ever happens
at boot with the motors off; in a LOCAL build the same code runs with a live gyro
stream and a live PID loop, which is why a Save can change how the aircraft
reacts to stick input. Steady-state behaviour, PID/rate/filter settings and every
value in the `state` fingerprint of `sitl-audit.log` stay identical - the change
lives in the stick-transient state that fingerprint does not cover.

The bigger share comes from the configurator's own `MSP_SET_*` burst, which runs
*before* the EEPROM write. Two of those handlers rebuild stateful chains in the
firmware:

```c
case MSP_SET_RC_TUNING:      ... initRcProcessing();                      // 153 us
case MSP_SET_FILTER_CONFIG:  ... gyroInitFilters(); pidInitFilters(...);  // 318 us
```

Fixed in the LOCAL build:

- `MSP_EEPROM_WRITE` no longer re-reads the EEPROM (`readEEPROM` is renamed to
  `sitlMspReadEEPROM` for msp.c): the config is persisted and the FC keeps flying
  with the config it already has. PID/rate writes still apply live through their
  own MSP handlers (`pidInitConfig`), and boot-time settings (filters, mixer,
  features, ...) apply on "Save and Reboot" or through `sitl_local_reload_config()`
  - the same split real hardware has.
- The in-process firmware reboot no longer re-reads the EEPROM either: it was
  written from RAM a few lines earlier in the same operation, so re-reading cannot
  change a value, while everything the config derives is re-applied explicitly
  (mixer/motor/servo, gyro filters when their settings changed, `debugMode`, the
  `initPhase3` profile state) by `localRunPendingReloadInternal(false)`.
- The three handler calls above are renamed for msp.c in LOCAL builds
  (`sitlMspInitRcProcessing`, `sitlMspGyroInitFilters`, `sitlMspPidInitFilters`)
  and rebuild the chain **only when the underlying settings really changed** - the
  same "snapshot and compare" rule the reload path already used for the gyro
  filters, extended to the D-term filters and to the rate/RC processing inputs. A
  genuine filter or rate edit still applies immediately, exactly like a real FC
  applying it live; a save that changes nothing no longer touches the loop.

After the fixes every save-like mode measures identical (max 0.0009 us - the
harness's own noise floor), while the controls that must stay sensitive still do:
`sensitivity` reports a real loop change at 10.6 us, and `cfg-change` shows 235 us
of change when a filter byte is flipped and returns to the baseline once the
original bytes are written back. An explicit config reload
(`sitl_local_reload_config()`, and the per-aircraft EEPROM path switch) still
reads the selected file by design: a different aircraft *should* start from that
file's state.

#### Scenario / operation matrix

`tools/run-save-matrix.ps1` runs the whole grid and prints a PASS/FAIL summary
(exit code 0 = everything behaved as expected):

```
pwsh -File tools\run-save-matrix.ps1
```

Stick scenarios: `roll-step`, `pitch-step`, `yaw-step`, `snap` (fast doublet),
`throttle`, `combined` (all three axes at once). Operations: `save-only`,
`save-reboot`, `cfg-save`, `cfg-save-reboot` (the configurator's read-modify-
write burst + save), `aux-save` (the save happens with every non-ARM AUX channel
high, i.e. with whatever other mode boxes the config has bound there active),
`armed-save` (the save attempt while armed, which the firmware must reject), the
edit flows (`cfg-change-filter`, `cfg-change-rate`, `cfg-change-pid`,
`cfg-change`), a double save (`twice`), five saves in a row (`repeat`) and an
in-process FC restart (`reinit`). A no-op save must leave the traces identical; a
real edit must change them; reverting must return to the baseline, and `reinit`
(`shutdown()` + `init()`) must reproduce the fresh-boot traces. `cfg-change-rate`
is the one case whose *reverted* trace is allowed to differ, and only from the
stick doublet on: the rate edit itself re-seeds the RC-smoothing / feedforward
state via `initRcProcessing()`, so the steady state returns but the filter state
is not rewound (a real FC behaves the same way). The script checks that the
divergence starts at or after the doublet instead of demanding bit equality.
Current state: all 13 cases pass.

Known gap: the CLI's own `save` (`#` to enter the CLI, then `save`) is not part
of the matrix yet - the CLI is entered (`ARMING_DISABLED_CLI` shows up) and the
harness can drive it, but the `save` command did not reach `writeEEPROM()` in
that emulation, so the mode (`cli-save`) is experimental and excluded for now.

#### How this compares to AJ92/SimITL

SimITL is the reference implementation this link design follows (its own
`extern/betaflightext` overrides plus a stock-ish Betaflight fork). Its save and
reboot handling differs from ours in two important ways:

| | AJ92/SimITL | this build (LOCAL) |
| --- | --- | --- |
| plain Save (`MSP_EEPROM_WRITE`) | stock firmware: `writeReadEeprom()` -> `writeEEPROM(); readEEPROM();` -> `activateConfig()`, i.e. `initRcProcessing()` / `pidInit()` / `rcControlsInit()` / `accInitFilters()` run live | persist only; those live re-inits are gated on "the settings really changed" (stock behaviour measured: 153 us / 318 us of different stick response) |
| Save and Reboot | `systemReset()` calls `init()`, a full in-process firmware init, and their `systemInit()` resets the virtual clock to 0 and `cliMode` | persist + re-apply (mixer/motor/servo, filters only when changed, `debugMode`, `initPhase3` modules); nothing is re-initialised that the firmware's own boot built, so the running loop is never disturbed (see "The shake: one root cause, two triggers") |
| leaving the CLI | the reboot's `systemInit()` sets `cliMode = false` | `sitlLocalRequestReboot()` clears `cliMode` and the CLI arming block; a watcher clears them when the CLI is left without a reboot |
| virtual EEPROM | their `target.c` closes a stale handle on re-open ("can just restart without closing the fileDesc") | `sitl_local_shutdown()` flushes+closes it, and `sitl_local_init()` now also closes a stale handle before boot (adopted from SimITL) |
| configurator link | WebSocket straight onto the serial port (5761) via libwebsockets | plain TCP 5761 plus a separate WebSocket proxy on 6761 |

Netting it out: SimITL deliberately does a *full* `init()` on reboot and leaves
the firmware's own save path untouched, which is simple but means a plain Save
re-initialises the stick-transient chain under a live flight loop. This build
keeps the same semantics a real FC has (Save persists, boot-time settings need a
reboot) while making both paths measurably non-invasive - which is what the
matrix above checks.

#### The shake: a restart did not reproduce a fresh boot

The "the DLL flew fine until it was restarted, then it shook" symptom had **one**
root cause: on a second boot the firmware built a *different* gyro filter chain
than on the first, so the same stick input produced a different motor response
(measured: 236 us at the stick step - 24% of the 1000..2000 us range - plus a
persistent offset from the re-integrated I term). Two rounds were needed because
earlier fixes had moved the trigger:

| trigger | how it fired | fix |
| --- | --- | --- |
| configurator **Save** / **Save and Reboot** | the page writes its settings first (`MSP_SET_PID`, `MSP_SET_FILTER_CONFIG`, `MSP_SET_RC_TUNING`), then `MSP_EEPROM_WRITE` -> `writeReadEeprom()` -> `writeEEPROM(); readEEPROM();` -> `activateConfig()`, all under a live loop | the Save no longer re-reads the EEPROM, and those three handlers rebuild only when the settings really changed |
| **in-process FC restart / level reload** (a regression of ours) | `sitl_local_init()` was made to re-derive the same three chains on a *second* init inside one process, to make a restart byte-identical to a fresh boot | removed again: the boot path is left exactly as the firmware builds it |
| **runtime `sitl_local_set_eeprom_path()` + `sitl_local_reload_config()`** (per-aircraft switch) | the reload ran `readEEPROM()` -> `activateConfig()` and two unconditional `pidInit()` calls inside a running loop | the file is loaded values-only, the chains are rebuilt only where the loaded settings differ, and a file identical to what the FC is running makes the request a no-op (`reload: configuration identical, nothing to re-apply`) |

That second trigger is still the pitfall worth remembering: it was added while
fixing a *measurement* (a synthetic 315 us "restart is not a fresh boot"
difference) and it made the shake appear from boot instead of after a save. Do not
"helpfully" re-initialise what the firmware itself just built while the loop is
running - the trigger only reappears somewhere else.

With the live re-inits gone, the remaining restart problem was a **boot-ordering
bug of our own**, and it is the real answer to "why did restarting the DLL
change how it flew":

`useDshotTelemetry` says whether the simulator's bridged motor RPM may feed the
firmware (RPM filter, ESC/OSD/MSP telemetry). `rpmFilterInit()` runs inside the
boot's `gyroInitFilters()` and returns early - leaving `numHarmonics = 0`, i.e.
the RPM filter disabled for the whole run - when the flag is false. The LOCAL
link set the flag only *after* the boot (`localApplyLinkOverrides()`), and the
flag is a plain global that a fresh process starts with at 0:

| boot | flag when `rpmFilterInit()` runs | gyro chain |
| --- | --- | --- |
| first boot in a fresh process | 0 (loader zero) | RPM filter **off** |
| any in-process re-init (host `shutdown()`+`init()`, level reload, PIE restart) | still 1 from the previous run | RPM filter **on** |

Two different filter chains from the same saved configuration. Fixed by setting
the flag in `sitlLocalPreMotorInit()` - the hook `sitlBoot()` runs after the
EEPROM read and before `initPhase2/3`, i.e. before the boot derives anything from
it - so every boot, first or re-init, builds the chain the configuration asks
for. To make the restart an actual power cycle, `sitlLocalResetVolatileControlState()`
also zeroes the leftovers the firmware's `init()` does not rebuild (PID
integrators and part of `pidRuntime`, the attitude quaternion, the raw gyro
state) before a second boot. It runs with the loop stopped, so unlike the
mid-flight rebuilds above it cannot inject a step into a live filter.

`sitl_local_shutdown()` was also fixed: it flushes and closes the virtual EEPROM
(`localFlushEepromWrite()` -> `configLock()`) and stops the TCP/WebSocket
listeners (`serialTcpStop()` / `wsProxyStop()`), so a later `sitl_local_init()`
can bind 5761/6761 again instead of hanging in `FLASH_Unlock`.

How it was pinned down:

- the **state flight recorder** in `sitl-audit.log` (config hashes + runtime
  values, timestamped) *disproved* the "EEPROM corrupted / tuning changed"
  hypotheses: every control-relevant field was constant across sessions;
- the **1 kHz burst recorder** (`sitl-burst.log`: fed gyro, PID term
  contributions, motors, attitude, modes, and the loop period the firmware really
  used) showed a D-dominated ~120 Hz limit cycle with saturated motors - the
  signature of a mid-flight re-init, not of a gain problem;
- **A/B builds** separated the suspects: attitude estimator bypassed, Save
  re-reading the EEPROM again, and finally the historical commit `a315ef9` (the
  state before the save/reboot work). `a315ef9` was clean at boot but shook after
  saving the PID tab, while the newer build shook from boot - which located the
  moved trigger.
- the **two-boot fingerprint** (`ctrl boot#1` / `ctrl boot#n` lines in
  `sitl-audit.log`) compared what each boot derived; the chains matched except for
  the attitude leftovers, which led to adding the volatile-state reset;
- the **per-step loop signals** (`sitl_local_get_loop_state()`, the harness's
  `loop signals` and `probe` output) finally located the RPM filter: the value
  entering the gyro lowpass chain (`sampleSum`, the LPF2/downsample output) was
  bit-identical between the two boots, while the lowpass filter's *output* was
  not - and `rpmFilterRun()` is the only stage between them. Reconstructing the
  filter's actual input from its state and gain (`lpf1In` in the probe output)
  showed it was a pure pass-through on the first boot and an active notch bank on
  the restart.

#### Control-loop diagnostics

#### The RPM filter: it works, and what it costs

The RPM filter is *not* broken - measured with the harness's `rpm-tone` mode,
which feeds a pure sine at the motor fundamental into the roll gyro and compares
what feeds the filter chain (the LPF2/downsample output) with what reaches the
PID (`gyroADCf`):

```
sitl_local_save_compare rpm-tone roll-step 200      # 200 Hz = 12000 rpm

rpm filter as configured      input 136.486 counts -> pid   0.000 counts (-119.0 dB)
rpm filter off (harmonics 0)  input 136.486 counts -> pid 100.347 counts (  -2.7 dB)
rpm filter restored           input 136.486 counts -> pid   0.000 counts (-120.5 dB)
```

The notch bank sits exactly on the motor harmonic (a perfect null), passes
everything else with only the normal lowpass attenuation, and follows the
configurator live (the mode flips `rpm_filter_harmonics` through
`MSP_SET_FILTER_CONFIG`, i.e. the same path the Filters tab uses). The bridge's
units are right as well: Betaflight's `getMotorFrequencyHz()` is the *mechanical*
rotation frequency (`erpmToHz = ERPM_PER_LSB / 60 / (poles/2)`), and the bridge
feeds `rpm / 60`, so the notches land on `harmonic * motorHz` as intended (the
loop-state diagnostic prints them: `notchHz=200/400/480` for a 12000 rpm feed).

What the filter costs is phase, and at a 1 kHz loop that is the whole story:
each notch is a full-depth (weight 100%) band-stop, so a bank of 4 motors x 3
harmonics sitting at 150..480 Hz eats phase in the 100..200 Hz band where the
control loop's margin lives. `tools/rpm_notch_response.py` reproduces the exact
`rpmNotchApply()` recurrence and prints the numbers:

| notch bank (4 motors) | phase at 120 Hz |
| --- | --- |
| 12000 rpm feed (200/400/480 Hz per motor) | -67 deg |
| same, dropping the harmonics BF has to clamp | -66 deg |
| 15000 rpm (250 Hz) + clamped 480 Hz | -25 deg |
| 33000 rpm, all harmonics clamped to 480 Hz | -3 deg |
| RPM filter off | 0 deg |

Two consequences worth knowing:

- At a 1 kHz loop `rpmFilter.maxHz = 0.48/looptimeUs = 480 Hz`, so any harmonic
  above that is *clamped* to 480 Hz (0.96 Nyquist): it cannot filter anything the
  1 kHz-sampled gyro can represent, but it still costs phase. Betaflight's
  configurator warns about exactly this ("RPM filter harmonics above the Nyquist
  frequency - increase the PID loop frequency"). This is a property of the loop
  rate, not of the bridge.
- A real gyro sees motor vibration at those harmonics, which is what the filter
  removes; a rigid-body simulation's gyro is clean, so the filter has nothing to
  remove and only the phase cost remains. With a default tune at 1 kHz (about
  -65 deg of delay-induced phase at 120 Hz) a -25..-67 deg notch bank is enough
  to tip the loop into a sustained ~100..150 Hz oscillation.

**Bridge gap fixed here:** Betaflight's `getMotorFrequencyHz()` does not return
the raw RPM - `dshotTelemetryProcess()` lowpass-filters it with
`rpm_filter_lpf_hz` (default 150 Hz, dt = `gyro.targetLooptime`) before the RPM
filter, the OSD and the dynamic idle use it. The bridge returned the raw value,
so the notch frequencies tracked every PID-driven RPM change. The same LPF is now
applied in the bridge (`sitlSimMotorFreqLpf*` in `src/wincompat.c`, rebuilt
whenever `gyro.targetLooptime` changes).

To make an RPM-related shake diagnosable from a single flight, the 1 kHz burst
recorder now also writes `rpm0..rpm3`, the three notch frequencies it derives and
the filtered gyro (`gyroADCfR/P/Y`) after the PID terms - so the oscillation
frequency can be matched against the notch bank and the RPM behind it.

The harness also has a `closed-loop` mode that closes the loop around a
first-order plant (rate = motor differential through a 25 ms lag, with the RPM
derived from the motor outputs like Unreal's ESC model); at every plant gain
tried the RPM filter did *not* push that simple loop into a limit cycle, which is
why the flight-test data above matters for reproducing the reported shake.

`sitl_local_get_loop_state()` (see `src/sitl_local.h`) returns a per-step snapshot
of the signals that drive the rate loop: the scaled and filtered gyro, the stick
setpoint in RC units, every PID term, the measured loop periods, the gyro filter
chain internals (downsample output, lowpass state/gain, dynamic-lowpass
settings), and the configured gains/pointers. The save-invariance harness prints
it as `loop signals` and a `probe` table, which is what makes a difference
attributable to a single stage instead of "the motors changed". It is read-only
and safe to call from the stepping thread.

Also fixed along the way:

- **A test tool must never write the user's EEPROM**: the harness works on a copy
  in `%TEMP%` and selects it with `sitl_local_set_eeprom_path()` *before*
  `sitl_local_init()` (a `BF_SITL_EEPROM` set by the host before the DLL is loaded
  is not reliable - the DLL has its own CRT environment). With that, a writing
  test case leaves the real `eeprom.bin` byte-identical (SHA256 checked).
- **`reinit` is no longer expected to be identical** to the pre-restart state: the
  FC is left exactly as its own boot built it, which is what keeps the shake away.
  The save / save-and-reboot cases are the ones that must stay identical, and the
  matrix checks exactly that.

Harness notes: it needs an ARM switch in the EEPROM (it probes every AUX channel
and both switch positions until the firmware really arms, then turns the runaway
takeoff protection off the way a real takeoff does), it runs the FC at 1 kHz
synchronously, it compares traces by scenario phase and aligns them to the 8 ms
RC-frame cadence, and its output goes to stderr because LOCAL mode reopens stdout
to `NUL`.

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
- Logs are named `LOG00001.BFL`, `LOG00002.BFL`, ... (auto-incrementing). In
  the standalone build they land in the process working directory; in the
  LOCAL DLL build they are redirected to the same stable folder as the
  virtual EEPROM (`%LOCALAPPDATA%\Betaflight-SITL\LOG00001.BFL`), because the
  host engine's working directory is not under your control. The host can
  override the folder at runtime per aircraft with
  `sitl_local_set_blackbox_dir("E:\\MySim\\Aircraft1")` - the directory is
  created if missing and the log numbering is re-scanned, so existing logs in
  that folder are never overwritten (call it before arming / logging starts).
- `blackbox_mode = NORMAL` (default) records while armed; `ALWAYS` records
  from boot without arming; `MOTOR_TEST` records during motor tests.
- `blackbox_sample_rate` selects 1/1, 1/2, 1/4, 1/8 or 1/16 of the PID rate
  (default 1/4, i.e. 250 Hz at a 1 kHz PID loop).

Workflow:

1. Fly (or set `blackbox_mode = ALWAYS` for bench recordings).
2. Disarm / stop - the log is flushed and closed.
3. Open the Betaflight Configurator's Blackbox tab and load
   `build-win-cmake\LOG00001.BFL` (standalone), or
   `%LOCALAPPDATA%\Betaflight-SITL\LOG00001.BFL` (LOCAL DLL).

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

Note for the LOCAL DLL build: the firmware applies several settings (debug
mode, gyro/dterm filters) only at boot, and the LOCAL "reboot" does not re-run
the boot sequence. The DLL re-applies this boot-time config on every config
save / reboot: `debug_mode` is re-synced (so `set debug_mode = CHIRP` +
`save` works for blackbox recording without reloading the DLL) and the
gyro/dterm filter chains are rebuilt from the saved settings (Filter tab).
Filter re-init is skipped while armed so it never races the flight loop; a
later save/reboot while disarmed applies it. Settings that still need a full
host-session restart are the hardware-class ones (sensor selection, motor
protocol/rate, serial port roles) - changing those is not expected in the
sim workflow.

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
