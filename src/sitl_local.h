/**
 * In-process Betaflight SITL API for simulators (SITL_LINK_MODE=LOCAL).
 *
 * Instead of exchanging fdm_packet / motor packets over UDP, the host engine
 * (e.g. Unreal's async physics tick at 1000 Hz) calls sitl_local_step()
 * synchronously: the flight controller receives the simulator state, runs one
 * scheduler pass on its virtual clock, and returns the motor outputs for that
 * exact state with zero network latency.
 *
 * The web configurator still works: the DLL keeps the TCP/WebSocket proxy
 * alive on 127.0.0.1:5761 / 6761, and a background thread services MSP.
 */

#ifndef SRC_SITL_LOCAL_H
#define SRC_SITL_LOCAL_H

#include <stdint.h>
#include <stdbool.h>

#if defined(_WIN32) && defined(SITL_LOCAL)
#define SITL_LOCAL_API __declspec(dllexport)
#else
#define SITL_LOCAL_API
#endif

#define SITL_LOCAL_MAX_RC_CHANNELS 16
#define SITL_LOCAL_MAX_MOTORS      16

typedef struct {
    double timestamp;                   // seconds, monotonic (logging only)
    double angular_velocity_rpy[3];     // rad/s, FRD body frame
    double linear_acceleration_xyz[3];  // m/s^2, FRD body specific force
    double orientation_quat[4];         // w,x,y,z, same convention as the UDP fdm_packet
    double velocity_xyz[3];             // m/s, earth frame ENU (east, north, up)
    double position_xyz[3];             // longitude, latitude, altitude (like UDP mode)
    double pressure;                    // Pa (ignored: derived from altitude)
    double battery_voltage;             // V
    double battery_current;             // A
    double motor_rpm[4];                // per-motor RPM (telemetry)
    uint16_t rc_channels[SITL_LOCAL_MAX_RC_CHANNELS]; // 1000..2000, AETR + aux
} sitl_local_input_t;

typedef struct {
    uint8_t motor_count;
    float pwm_output_raw[SITL_LOCAL_MAX_MOTORS]; // 1000..2000 (raw PWM)
    bool armed;
} sitl_local_output_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Boot the flight controller (equivalent to starting betaflight_SITL.exe).
 * Call once from the host process before the first step.
 */
SITL_LOCAL_API int sitl_local_init(void);

/**
 * Advance the virtual clock by dtUs and run one scheduler pass.
 * dtUs is normally 1000 (1 ms) for a 1000 Hz host tick.
 * Sensors/RC are taken from `in`, motor outputs are returned in `out`.
 */
SITL_LOCAL_API void sitl_local_step(const sitl_local_input_t *in, uint32_t dtUs,
                                    sitl_local_output_t *out);

/** Current virtual time in microseconds (the FC clock). */
SITL_LOCAL_API uint64_t sitl_local_time_us(void);

/** Stop the background MSP thread. Does not exit the host process. */
SITL_LOCAL_API void sitl_local_shutdown(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // SRC_SITL_LOCAL_H
