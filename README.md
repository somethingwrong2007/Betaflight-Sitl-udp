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

## Windows notes

The Windows executable needs `libwinpthread-1.dll` next to it (the
`betaflight-sitl-windows-full` artifact includes it). GCC and C++ runtime DLLs
are statically linked.

## Protocol

See Betaflight SITL documentation for `fdm_packet`, `servo_packet`, and `rc_packet` structures.
