# BF-SITL-UDP

Official Betaflight SITL compiled as a standalone UDP server for Windows and Linux.

- UDP 9001: Motor outputs, RealFlight bridge format (servo_packet_raw)
- UDP 9002: Motor outputs, Gazebo format (servo_packet)
- UDP 9003: Flight dynamics in (fdm_packet)
- UDP 9004: RC channels in (rc_packet)
- TCP 5761: MSP / Betaflight Configurator (UART1)

The Betaflight submodule is pinned to current master, where the SITL target
lives under `src/platform/SIMULATOR`. The CMake build mirrors the official
`make TARGET=SITL` source list, and a small Winsock shim replaces the POSIX
UDP layer when cross-compiling for Windows.

## Build

### Linux (native)
```bash
./setup.sh
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Windows (cross-compile from Linux)
```bash
sudo apt install mingw-w64 cmake
mkdir build-win && cd build-win
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/mingw-w64-toolchain.cmake
make -j$(nproc)
```

## Usage

The executable is the official Betaflight SITL server:

```bash
./betaflight_SITL --ip 127.0.0.1 --gpx
```

- `--ip <address>`: IP address to send motor outputs to (default: `127.0.0.1`)
- `--config <file>`: load a CLI config file, save it to EEPROM, then exit
- `--gpx`: write a GPS track to `sitl_track.gpx`
- `--help`, `-h`: show usage

The first run creates `eeprom.bin` (32 KiB) in the working directory.

## Time base (scheduler) modes

The scheduler time base is selected at compile time with `SITL_TIME_MODE`
(default `REALTIME`):

### REALTIME (default)

The official Betaflight scheduler busy-waits to the exact gyro deadline.
Gyro/filter/PID are locked at `SITL_GYRO_HZ` (default 1 kHz). Timing is exact
to the microsecond but the busy-wait consumes about one CPU core.

### UDP (Unreal FDM-driven)

The virtual clock is driven by Unreal FDM packets on UDP 9003 (the same
packet-stepping idea as AJ92/SimITL): each packet's timestamp delta advances
the virtual clock in 100 us quanta, so gyro/filter/PID fire once per 1000 us
of Unreal time at near-zero CPU. When no packets are arriving the flight loop
idles (virtual clock frozen, CPU ~0) while the serial/MSP link stays alive,
so the Betaflight configurator remains connected.

```bash
cmake ..                        # REALTIME (default)
cmake .. -DSITL_TIME_MODE=UDP   # Unreal FDM-driven
```

The gyro/filter/PID frequency defaults to 1 kHz and can be changed with the
`SITL_GYRO_HZ` CMake option, or at runtime with the `BF_SITL_GYRO_HZ`
environment variable.

## Configuration persistence

Settings saved from the Betaflight configurator are stored in `eeprom.bin`
(32 KiB) in the working directory, mirroring the FC's flash:

- Save: click **Save** in the configurator (or type `save` in its CLI panel)
  - this sends `MSP_EEPROM_WRITE` and the SITL flushes `eeprom.bin`.
- Load: automatic at startup from `eeprom.bin`.
- Reset: `defaults` then `save` in the CLI panel.
- Text import/export: `diff`/`dump` in the CLI panel produces a text config;
  `.\betaflight_SITL.exe --config <file>` loads it, saves it to `eeprom.bin`,
  and exits.

`eeprom.bin` is created next to the working directory (falling back to the
executable's folder, then `%LOCALAPPDATA%\Betaflight-SITL`), so always launch
from the same folder if you want one persistent configuration.

## Betaflight web configurator

The build includes a built-in WebSocket proxy on `ws://127.0.0.1:6761` that
bridges to the MSP port on TCP 5761, so the online Betaflight configurator can
connect without an external websockify:

1. Run `betaflight_SITL`.
2. Open the configurator and enable manual connection mode.
3. Enter `ws://127.0.0.1:6761` in the port field and click Connect.

Chrome/Edge 147+ reject WebSocket handshakes that do not echo the client's
`binary` subprotocol; the built-in proxy handles this automatically.

### Quick start (Windows)

For an offline, VPN-free web UI (also useful for local game tooling):

1. Build the official `betaflight-configurator` repo once (`npm ci && npm run
   build`) so its `src/dist` directory exists under `bf-configurator/`.
2. Run `.\start-bf.ps1` - it starts `betaflight_SITL.exe`, starts the local web
   server on `http://127.0.0.1:8080` and opens the configurator in your
   browser.

   Or just double-click `start-bf.cmd` - same thing, no PowerShell needed.

`bfweb-server.mjs` serves the built `src/dist` directory and injects a tiny
script into `index.html` that presets the connection settings in localStorage:
dev auto-options are disabled and virtual mode is turned off, so the
configurator defaults to the manual connection with `ws://127.0.0.1:6761`
already in the port field - clicking Connect connects straight to the SITL.
No files from `bf-configurator` are modified. A page served from 127.0.0.1
connecting back to 127.0.0.1 is exempt from browser local-network permission
prompts.

## Windows notes

The Windows executable needs `libwinpthread-1.dll` next to it (the
`betaflight-sitl-windows-full` artifact includes it). GCC and C++ runtime DLLs
are statically linked.

## Protocol

See Betaflight SITL documentation for `fdm_packet`, `servo_packet`, and `rc_packet` structures.
