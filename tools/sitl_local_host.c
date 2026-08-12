/**
 * Persistent 1000 Hz host for the LOCAL-mode DLL (debug/testing).
 *
 * Keeps the FC booted and stepping so the configurator / MSP can be tested
 * over 127.0.0.1:5761/6761 without Unreal. Ctrl+C to stop.
 */

#include "sitl_local.h"

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

int main(void)
{
    if (sitl_local_init() != 0) {
        fprintf(stderr, "sitl_local_init() failed\n");
        return 1;
    }

    sitl_local_input_t in;
    memset(&in, 0, sizeof(in));
    in.orientation_quat[0] = 1.0;
    in.position_xyz[2] = 1.0;
    in.battery_voltage = 16.8;
    for (int i = 0; i < SITL_LOCAL_MAX_RC_CHANNELS; i++) {
        in.rc_channels[i] = 1500;
    }

    sitl_local_output_t out;
    fprintf(stderr, "[host] running at 1000 Hz, Ctrl+C to stop\n");
    while (1) {
        sitl_local_step(&in, 1000, &out);
#ifdef _WIN32
        Sleep(1);
#endif
    }
    return 0;
}
