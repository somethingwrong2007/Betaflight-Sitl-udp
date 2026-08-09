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

On Windows the SITL scheduler normally spin-waits for the next gyro tick,
which consumes a full CPU core even when no flight simulator is attached.
This build replaces that busy-wait with a sleeping poll by default, cutting
SITL CPU use from ~100% of a core to ~2% at the cost of a 10 kHz -> ~200 Hz
gyro/PID loop (fine for SITL and ground-station use). The serial/MSP task is
rescheduled to 1 kHz so the configurator stays responsive. To restore the
official busy-wait behavior, set `BF_SITL_LOW_CPU=0`:

```powershell
$env:BF_SITL_LOW_CPU = 0  # official 10 kHz busy-wait
.\betaflight_SITL.exe
```

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

`bfweb-server.mjs` serves the built `src/dist` directory and injects a tiny
script into `index.html` that presets the connection settings in localStorage:
the Connect menu's Manual entry opens with `ws://127.0.0.1:6761` already in the
port field. No files from `bf-configurator` are modified. A page served from
127.0.0.1 connecting back to 127.0.0.1 is exempt from browser local-network
permission prompts.

## Windows notes

The Windows executable needs `libwinpthread-1.dll` next to it (the
`betaflight-sitl-windows-full` artifact includes it). GCC and C++ runtime DLLs
are statically linked.

## Protocol

See Betaflight SITL documentation for `fdm_packet`, `servo_packet`, and `rc_packet` structures.
