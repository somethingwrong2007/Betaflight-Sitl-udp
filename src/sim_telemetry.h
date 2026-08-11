#ifndef SRC_SIM_TELEMETRY_H
#define SRC_SIM_TELEMETRY_H

#include <stdint.h>

// Sim telemetry fed from the extended FDM packet on UDP 9003 (Windows UDP
// mode). The first 144 bytes stay the official fdm_packet; the optional tail
// carries battery voltage, battery current and per-motor RPM so the virtual
// FC can run with realistic battery/RPM data.
void simTelemetrySet(double voltage, double current, const double *rpm, int rpmCount);

uint16_t simTelemetryVoltageCentiVolts(void);
float simTelemetryCurrentAmps(void);
float simTelemetryMahDrawn(void);
void simTelemetryCurrentRefresh(int32_t lastUpdateAtUs);
float simTelemetryMotorFrequencyHz(uint8_t motorIndex);

#endif
