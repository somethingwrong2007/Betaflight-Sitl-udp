#ifndef SRC_SIM_TELEMETRY_H
#define SRC_SIM_TELEMETRY_H

#include <stdint.h>

// Sim telemetry fed from the extended FDM packet on UDP 9003 (Windows UDP
// mode) or from the LOCAL host call. The first 144 bytes stay the official
// fdm_packet; the optional tail carries battery voltage, battery current,
// per-motor RPM and per-motor ESC temperature so the virtual FC can run with
// realistic battery/motor telemetry.
//
// temperature is in degrees Celsius, one entry per ESC; a NULL pointer (or a
// <= 0 value) means "no data" and keeps the previous temperature.
void simTelemetrySet(double voltage, double current,
                     const double *rpm, int rpmCount,
                     const double *temperature, int temperatureCount);

uint16_t simTelemetryVoltageCentiVolts(void);
float simTelemetryCurrentAmps(void);
float simTelemetryMahDrawn(void);
void simTelemetryCurrentRefresh(int32_t lastUpdateAtUs);
float simTelemetryMotorFrequencyHz(uint8_t motorIndex);

// ESC temperature in whole degrees Celsius, clamped to the 0..255 range the
// DSHOT telemetry field uses (25 before the host sends anything).
uint8_t simTelemetryEscTemperatureCelsius(uint8_t motorIndex);

#endif
