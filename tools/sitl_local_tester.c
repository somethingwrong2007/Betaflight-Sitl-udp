/**
 * Minimal 1000 Hz harness for the LOCAL-mode DLL.
 *
 * Feeds a level hover state for 3 seconds, then a throttle step, and prints
 * the motor outputs the flight controller produced for that exact state.
 * Link against betaflight_SITL.dll (built with -DSITL_LINK_MODE=LOCAL).
 */

#include "sitl_local.h"

#include <stdio.h>
#include <string.h>

int main(void)
{
    if (sitl_local_init() != 0) {
        fprintf(stderr, "sitl_local_init() failed\n");
        return 1;
    }

    sitl_local_input_t in;
    memset(&in, 0, sizeof(in));
    in.orientation_quat[0] = 1.0; // level
    in.position_xyz[2] = 1.0;     // 1 m altitude
    in.battery_voltage = 16.8;
    for (int i = 0; i < SITL_LOCAL_MAX_RC_CHANNELS; i++) {
        in.rc_channels[i] = 1500;
    }
    in.rc_channels[2] = 1500; // throttle mid
    in.rc_channels[4] = 2000; // AUX1 high (arm switch, if configured)

    sitl_local_output_t out;

    printf("--- hover phase (3 s) ---\n");
    for (int i = 0; i < 3000; i++) {
        sitl_local_step(&in, 1000, &out);
        if (i < 100 || i % 500 == 499) {
            printf("step %4d t=%6llu us armed=%d motors=%d  M0=%.0f M1=%.0f M2=%.0f M3=%.0f\n",
                   i, (unsigned long long)sitl_local_time_us(), out.armed, out.motor_count,
                   out.pwm_output_raw[0], out.pwm_output_raw[1],
                   out.pwm_output_raw[2], out.pwm_output_raw[3]);
        }
    }

    printf("--- throttle step to 1600 (1 s) ---\n");
    in.rc_channels[2] = 1600;
    for (int i = 0; i < 1000; i++) {
        sitl_local_step(&in, 1000, &out);
        if (i % 200 == 199) {
            printf("step %4d t=%6llu us armed=%d motors=%d  M0=%.0f M1=%.0f M2=%.0f M3=%.0f\n",
                   i, (unsigned long long)sitl_local_time_us(), out.armed, out.motor_count,
                   out.pwm_output_raw[0], out.pwm_output_raw[1],
                   out.pwm_output_raw[2], out.pwm_output_raw[3]);
        }
    }

    sitl_local_shutdown();
    return 0;
}
