# BF-SITL-UDP

Official Betaflight SITL compiled as a standalone UDP server for Windows and Linux.

- UDP 9002: Motor outputs (servo_packet)
- UDP 9003: Flight dynamics (fdm_packet)
- UDP 9004: RC channels (rc_packet)
- TCP 5761: MSP / Betaflight Configurator

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

```bash
./betaflight_SITL [eeprom_file] [target_ip]
```

- `eeprom_file`: Path to EEPROM binary (default: `./eeprom.bin`)
- `target_ip`: IP address to send motor outputs to (default: `127.0.0.1`)

## Protocol

See Betaflight SITL documentation for `fdm_packet`, `servo_packet`, and `rc_packet` structures.
