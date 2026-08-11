/**
 * In-process local link (SITL_LINK_MODE=LOCAL).
 *
 * The host engine calls sitl_local_step() synchronously at its own tick rate
 * (normally 1000 Hz). The virtual clock is advanced by the caller-supplied
 * dt, one scheduler pass runs gyro/filter/PID, and the motor outputs for that
 * exact state are returned in the same call - no UDP, no stale reads.
 *
 * Sensor feeds mirror sitl.c updateState() conventions (Gazebo bridge): FRD
 * angular velocity/acceleration, Gazebo-format quaternion, ENU velocity and
 * lon/lat/alt position.
 */

#ifdef SITL_LOCAL

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "platform.h"

#include "common/maths.h"

#include "drivers/accgyro/accgyro_virtual.h"
#include "drivers/barometer/barometer_virtual.h"
#include "drivers/compass/compass_virtual.h"
#include "drivers/dma.h"
#include "io/gps_virtual.h"
#include "flight/imu.h"
#include "sitl_gyro.h"
#include "fc/runtime_config.h"
#include "config/feature.h"
#include "msp/msp.h"
#include "sensors/battery.h"

#include "sitl_local.h"
#include "sim_telemetry.h"

#include <windows.h>

#ifndef USE_GPS_LAP_TIMER
#include "pg/gps_lap_timer.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define LOCAL_RAD2DEG  (180.0 / M_PI)
#define LOCAL_ACC_SCALE (256 / 9.80665)
#define LOCAL_GYRO_SCALE (16.4)

// Boot sequence lives in main_windows.c so both the standalone exe and the
// DLL share the same initialization.
extern void sitlBoot(int argc, char *argv[]);

// wincompat.c: stepped virtual clock used by the scheduler.
extern void sitlStepTime(uint64_t stepUs);
extern uint64_t micros64(void);

// Firmware entry points used by the synchronous step.
extern void scheduler(void);
extern void rxUpdateUdpChannels(const uint16_t *channels, uint8_t channelCount);
extern void rxInit(void);

// msp_serial.h pulls in io/serial.h, which collides with the MinGW windows.h
// include chain; declare just what the background MSP keep-alive needs (the
// enum layout matches msp_serial.h).
typedef enum {
    LOCAL_MSP_EVALUATE_NON_MSP_DATA = 0,
    LOCAL_MSP_SKIP_NON_MSP_DATA
} localMspEvaluateNonMspData_e;
extern void mspSerialProcess(localMspEvaluateNonMspData_e evaluateNonMspData,
                             mspProcessCommandFnPtr mspProcessCommandFn,
                             mspProcessReplyFnPtr mspProcessReplyFn);

// udplink_windows.c captures the motor packets pwmCompleteMotorUpdate()
// produces so the DLL can return them without any network I/O.
extern void sitlLocalCaptureMotorPacket(const void *data, size_t size);
extern bool sitlLocalTakeMotorPacket(void *out, size_t size);

static bool gLocalRunning = false;
static HANDLE gMspThread = NULL;
static volatile LONG gMspThreadStop = 0;

static uint16_t gLastRc[SITL_LOCAL_MAX_RC_CHANNELS];
static bool gLastRcValid = false;
static uint64_t gLastRcAnnounceUs = 0;

static double gGpsOriginLat = 0.0;
static double gGpsOriginLon = 0.0;
static bool gGpsOriginSet = false;

// --- LOCAL-mode link stubs ---
// The DLL keeps a few firmware paths alive that the executable build
// garbage-collects (PE export/import bookkeeping). Their implementations are
// target-only and not compiled for SITL; provide inert definitions so the
// library links. The virtual-sensor path never calls them.
uint8_t mpuGyroReadRegister(void *dev, uint8_t reg)
{
    (void)dev;
    (void)reg;
    return 0;
}

int dmaGetHandlerCount(void)
{
    return 0;
}

dmaChannelDescriptor_t dmaDescriptors[1] = {{0}};

bool useDshotTelemetry;

#ifndef USE_GPS_LAP_TIMER
gpsLapTimerConfig_t gpsLapTimerConfig_System;
#endif

static DWORD WINAPI localMspThreadProc(LPVOID arg)
{
    (void)arg;
    while (!gMspThreadStop) {
        mspSerialProcess(LOCAL_MSP_EVALUATE_NON_MSP_DATA,
                         mspFcProcessCommand, mspFcProcessReply);
        Sleep(5);
    }
    return 0;
}

int sitl_local_init(void)
{
    if (gLocalRunning) {
        return 0;
    }

    sitlBoot(0, NULL);

    // Make the local link self-sufficient regardless of the EEPROM contents:
    // force the UDP RX provider (sensor input arrives via sitl_local_step,
    // not a serial receiver) and pin the battery meters to the ADC shims that
    // read simTelemetrySet() values.
    featureEnableImmediate(FEATURE_RX_UDP);
    rxInit();
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
    batteryConfigMutable()->currentMeterSource = CURRENT_METER_ADC;

    gLocalRunning = true;
    gMspThreadStop = 0;
    gMspThread = CreateThread(NULL, 0, localMspThreadProc, NULL, 0, NULL);
    if (gMspThread == NULL) {
        gLocalRunning = false;
        return -1;
    }
    return 0;
}

void sitl_local_step(const sitl_local_input_t *in, uint32_t dtUs,
                     sitl_local_output_t *out)
{
    if (out) {
        memset(out, 0, sizeof(*out));
    }
    if (!gLocalRunning || !in) {
        return;
    }

    // --- virtual accelerometer (same signs/scales as sitl.c updateState) ---
    const int16_t ax = (int16_t)constrain(-in->linear_acceleration_xyz[0] * LOCAL_ACC_SCALE, -32767, 32767);
    const int16_t ay = (int16_t)constrain(-in->linear_acceleration_xyz[1] * LOCAL_ACC_SCALE, -32767, 32767);
    const int16_t az = (int16_t)constrain(-in->linear_acceleration_xyz[2] * LOCAL_ACC_SCALE, -32767, 32767);
    virtualAccSet(virtualAccDev, ax, ay, az);

    // --- virtual gyro (Gazebo bridge axis mapping) ---
    double gyroRoll, gyroPitch, gyroYaw;
    sitlGyroBodyFromSim(in->angular_velocity_rpy, ENABLE_GAZEBO_BRIDGE,
                        &gyroRoll, &gyroPitch, &gyroYaw);
    const int16_t gx = (int16_t)constrain(gyroRoll  * LOCAL_GYRO_SCALE * LOCAL_RAD2DEG, -32767, 32767);
    const int16_t gy = (int16_t)constrain(gyroPitch * LOCAL_GYRO_SCALE * LOCAL_RAD2DEG, -32767, 32767);
    const int16_t gz = (int16_t)constrain(gyroYaw   * LOCAL_GYRO_SCALE * LOCAL_RAD2DEG, -32767, 32767);
    virtualGyroSet(virtualGyroDev, gx, gy, gz);

    // --- pressure derived from altitude (Gazebo bridge convention) ---
    const double altMeters = in->position_xyz[2];
    const int32_t pressure = (int32_t)(101325.0 * pow(1.0 - 2.25577e-5 * altMeters, 5.25588));
    virtualBaroSet(pressure, 2500);

    // --- attitude quaternion (Gazebo plugin format -> NWU body-to-world) ---
    const float pktQw = (float)in->orientation_quat[0];
    const float pktQx = (float)in->orientation_quat[1];
    const float pktQy = -(float)in->orientation_quat[2];
    const float pktQz = -(float)in->orientation_quat[3];
    static const float k = 0.70710678f;
    const float attQw = k * (pktQw - pktQz);
    const float attQx = k * (pktQx - pktQy);
    const float attQy = k * (pktQy + pktQx);
    const float attQz = k * (pktQz + pktQw);

    // --- synthetic magnetometer feed (same earth field as sitl.c) ---
    {
        static const float fieldN = 2046.8f;
        static const float fieldW = -71.5f;
        static const float fieldU = -3547.2f;
        const float r00 = 1.0f - 2.0f * (attQy * attQy + attQz * attQz);
        const float r01 = 2.0f * (attQx * attQy - attQw * attQz);
        const float r02 = 2.0f * (attQx * attQz + attQw * attQy);
        const float r10 = 2.0f * (attQx * attQy + attQw * attQz);
        const float r11 = 1.0f - 2.0f * (attQx * attQx + attQz * attQz);
        const float r12 = 2.0f * (attQy * attQz - attQw * attQx);
        const float r20 = 2.0f * (attQx * attQz - attQw * attQy);
        const float r21 = 2.0f * (attQy * attQz + attQw * attQx);
        const float r22 = 1.0f - 2.0f * (attQx * attQx + attQy * attQy);
        const float magX = r00 * fieldN + r10 * fieldW + r20 * fieldU;
        const float magY = r01 * fieldN + r11 * fieldW + r21 * fieldU;
        const float magZ = r02 * fieldN + r12 * fieldW + r22 * fieldU;
        virtualMagSet(lrintf(magX), lrintf(magY), lrintf(magZ));
    }

#if defined(SITL_ATTITUDE_DIRECT)
    imuSetAttitudeQuat(attQw, attQx, attQy, attQz);
#endif

    // --- virtual GPS (Gazebo mirror around the first packet origin) ---
    const double longitude = in->position_xyz[0];
    const double latitude = in->position_xyz[1];
    const double altitude = in->position_xyz[2];
    if (!gGpsOriginSet) {
        gGpsOriginLat = latitude;
        gGpsOriginLon = longitude;
        gGpsOriginSet = true;
    }
    const double correctedLat = 2.0 * gGpsOriginLat - latitude;
    const double correctedLon = 2.0 * gGpsOriginLon - longitude;
    const double vx = in->velocity_xyz[0];
    const double vy = in->velocity_xyz[1];
    const double vz = in->velocity_xyz[2];
    const double speed = sqrt(vx * vx + vy * vy);
    const double speed3D = sqrt(vx * vx + vy * vy + vz * vz);
    double course = atan2(vx, vy) * LOCAL_RAD2DEG;
    if (course < 0.0) {
        course += 360.0;
    }
    if (fabs(latitude) <= 90.0 && fabs(longitude) <= 180.0) {
        setVirtualGPS(correctedLat, correctedLon, altitude, speed, speed3D, course,
                      vy, vx, -vz);
    }

    // --- battery / RPM telemetry ---
    simTelemetrySet(in->battery_voltage, in->battery_current, in->motor_rpm, 4);

    // --- RC ---
    // Announce a new frame whenever the channel values change, and at least
    // every 8 ms (the ~120 Hz floor) even when the sticks are stationary.
    // Announcing only on change makes the FC lose signal while sticks are
    // centered (RXLOSS after the 150 ms fail-safe window).
    const uint64_t nowUs = micros64();
    const bool rcChanged = !gLastRcValid || memcmp(gLastRc, in->rc_channels, sizeof(gLastRc)) != 0;
    if (rcChanged || (nowUs - gLastRcAnnounceUs) >= 8000) {
        memcpy(gLastRc, in->rc_channels, sizeof(gLastRc));
        gLastRcValid = true;
        gLastRcAnnounceUs = nowUs;
        rxUpdateUdpChannels(in->rc_channels, SITL_LOCAL_MAX_RC_CHANNELS);
    }

    // --- run one scheduler pass on the virtual clock ---
    sitlStepTime(dtUs);
    scheduler();

    // --- motor outputs captured by udpSend() in LOCAL mode ---
    if (out) {
        servo_packet_raw raw;
        if (sitlLocalTakeMotorPacket(&raw, sizeof(raw))) {
            out->motor_count = raw.motorCount;
            for (int i = 0; i < raw.motorCount && i < SITL_LOCAL_MAX_MOTORS; i++) {
                out->pwm_output_raw[i] = raw.pwm_output_raw[i];
            }
        }
        out->armed = (armingFlags & ARMED) != 0;
    }
}

uint64_t sitl_local_time_us(void)
{
    return micros64();
}

void sitl_local_shutdown(void)
{
    if (gMspThread != NULL) {
        InterlockedExchange(&gMspThreadStop, 1);
        WaitForSingleObject(gMspThread, 1000);
        CloseHandle(gMspThread);
        gMspThread = NULL;
    }
    gLocalRunning = false;
}

#endif // SITL_LOCAL
