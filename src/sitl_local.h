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

/**
 * Synchronous access to the in-memory flight-controller state, for hosts that
 * want to read/write the same structures the configurator sees over MSP
 * without touching the serial stream. All values below are the exact same
 * memory the MSP handlers read/write, so both sides stay consistent.
 */

/** Bitmask of arming-disable reasons; same value as MSP_STATUS_EX. 0 = may arm. */
SITL_LOCAL_API uint32_t sitl_local_get_arming_flags(void);

/** True when the FC is armed (same flag as the configurator's status icon). */
SITL_LOCAL_API bool sitl_local_get_armed(void);

/** flightModeFlags bitmask (ANGLE/HORIZON/MAG/... active modes). */
SITL_LOCAL_API uint32_t sitl_local_get_flight_modes(void);

/**
 * Read a rate profile. index in [0, CONTROL_RATE_PROFILE_COUNT) selects that
 * profile; any other value reads the current profile.
 * Units match the configurator's Rates tab: rcRate/yawRate in 0..2.5,
 * rcExpo/superRate in 0..1.0.
 */
SITL_LOCAL_API void sitl_local_get_rate(int index, float *rcRate, float *rcExpo,
                                        float *superRate, float *yawRate);

/**
 * Write the current rate profile (same fields as MSP_SET_RC_TUNING) and
 * persist it to the virtual EEPROM via the background thread. Roll/pitch
 * stay symmetric when they were equal, mirroring the MSP setter.
 */
SITL_LOCAL_API void sitl_local_set_rate(float rcRate, float rcExpo,
                                        float superRate, float yawRate);

/**
 * Locate the ARM switch in the mode-activation conditions. On success
 * auxChannel is the RC channel index (4 = AUX1) and startStep/endStep the
 * 25us-step range (900..2100us). When no ARM condition is configured,
 * auxChannel is set to 0xFF and the steps to 0.
 */
SITL_LOCAL_API void sitl_local_get_arm_switch(uint8_t *auxChannel,
                                              uint8_t *startStep,
                                              uint8_t *endStep);

/** Stop the background MSP thread. Does not exit the host process. */
SITL_LOCAL_API void sitl_local_shutdown(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // SRC_SITL_LOCAL_H
