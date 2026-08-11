/**
 * Windows (MinGW) UDP transport for the Betaflight SITL target.
 *
 * The official Betaflight udplink.c is POSIX-only. This file provides the
 * same udpLink_t API using Winsock2 so the official SITL server can run as a
 * standalone .exe without modifying the Betaflight submodule.
 */

#include <winsock2.h>
#include <ws2tcpip.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "udplink.h"
#include "win_socket_util.h"
#include "sim_telemetry.h"

static bool wsaInitialized = false;

#ifdef SITL_UDP_TIME
// Unreal FDM clock hook. The official SITL receive thread calls
// udpRecv(&stateLink, &fdmPkt, sizeof(fdm_packet), 100) on port 9003; the
// fdm_packet struct starts with `double timestamp` (seconds). Each valid
// datagram here adds the timestamp delta to a pending queue and signals the
// main loop, which steps the virtual clock by that amount (see main_windows.c
// and wincompat.c). This keeps all Unreal integration outside the Betaflight
// submodule.
#define SITL_FDM_PORT       9003
#define SITL_FDM_PACKET_SIZE 144 // sizeof(fdm_packet): 18 doubles
// Optional extended tail appended after the official fdm_packet:
// double battery_voltage (V), double battery_current (A),
// double motor_rpm[4]. Senders that only send the 144-byte packet keep the
// defaults below.
#define SITL_FDM_EXTENDED_SIZE 192
#define SITL_FDM_EXT_BATTERY_VOLTAGE 144
#define SITL_FDM_EXT_BATTERY_CURRENT 152
#define SITL_FDM_EXT_MOTOR_RPM 160
#define SITL_FDM_EXT_MOTOR_RPM_COUNT 4
// Cap the virtual time consumed per FDM packet at 5 s. Anything beyond that
// is a link restart, not a stutter: the run loop drains the delta in 100 us
// steps, so short UE hitches (frame drops, async-physics stalls) no longer
// freeze the scheduler and stop the motor output.
#define SITL_MAX_FDM_DELTA_US 5000000

static HANDLE gFdmEvent = NULL;
static volatile LONG64 gFdmPendingUs = 0;
static volatile LONG gFdmPacketCount = 0;
static double gFdmLastTs = -1.0;
static double gFdmRemainderUs = 0.0;

static double sitlBatteryVoltage = 16.8; // V, 4S default so the FC always sees a battery
static double sitlBatteryCurrent = 0.0;  // A
static double sitlMotorRpm[SITL_FDM_EXT_MOTOR_RPM_COUNT] = { 0.0 };
static double sitlMahDrawn = 0.0;        // mAh

void simTelemetrySet(double voltage, double current, const double *rpm, int rpmCount)
{
    if (voltage > 0.0) {
        sitlBatteryVoltage = voltage;
    }
    if (current >= 0.0) {
        sitlBatteryCurrent = current;
    }
    for (int i = 0; i < SITL_FDM_EXT_MOTOR_RPM_COUNT && i < rpmCount; i++) {
        if (rpm[i] >= 0.0) {
            sitlMotorRpm[i] = rpm[i];
        }
    }
}

uint16_t simTelemetryVoltageCentiVolts(void)
{
    return (uint16_t)(sitlBatteryVoltage * 100.0);
}

float simTelemetryCurrentAmps(void)
{
    return (float)sitlBatteryCurrent;
}

float simTelemetryMahDrawn(void)
{
    return (float)sitlMahDrawn;
}

void simTelemetryCurrentRefresh(int32_t lastUpdateAtUs)
{
    if (lastUpdateAtUs > 0) {
        // A * us -> mAh: 1 mAh = 3.6 A*s = 3.6e6 A*us
        sitlMahDrawn += sitlBatteryCurrent * (double)lastUpdateAtUs / 3.6e6;
    }
}

float simTelemetryMotorFrequencyHz(uint8_t motorIndex)
{
    if (motorIndex >= SITL_FDM_EXT_MOTOR_RPM_COUNT) {
        return 0.0f;
    }
    return (float)(sitlMotorRpm[motorIndex] / 60.0);
}

void sitlUdpFdmInit(void)
{
    if (gFdmEvent == NULL) {
        gFdmEvent = CreateEventA(NULL, FALSE, FALSE, NULL);
    }
}

HANDLE sitlUdpFdmEvent(void)
{
    return gFdmEvent;
}

// Returns and clears the accumulated FDM virtual-time microseconds.
uint64_t sitlUdpFdmTakePendingUs(void)
{
    return (uint64_t)InterlockedExchange64(&gFdmPendingUs, 0);
}

// True once at least two FDM datagrams have arrived (avoids switching on a
// single stray packet).
bool sitlUdpFdmHasData(void)
{
    return InterlockedCompareExchange(&gFdmPacketCount, 0, 0) >= 2;
}

void sitlUdpFdmReset(void)
{
    InterlockedExchange(&gFdmPacketCount, 0);
    InterlockedExchange64(&gFdmPendingUs, 0);
    gFdmLastTs = -1.0;
    gFdmRemainderUs = 0.0;
    if (gFdmEvent != NULL) {
        ResetEvent(gFdmEvent);
    }
}
#endif

static void ensureWsaStartup(void)
{
    if (!wsaInitialized) {
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0) {
            wsaInitialized = true;
        }
    }
}

int udpInit(udpLink_t *link, const char *addr, int port, bool isServer)
{
    ensureWsaStartup();

    SOCKET fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd == INVALID_SOCKET) {
        return -2;
    }
    socketNoInherit(fd);

    BOOL reuse = TRUE;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse));

    u_long nonblocking = 1;
    ioctlsocket(fd, FIONBIO, &nonblocking);

    link->fd = (int)fd;
    link->isServer = isServer;
    memset(&link->si, 0, sizeof(link->si));
    link->si.sin_family = AF_INET;
    link->si.sin_port = htons((u_short)port);
    link->port = port;

    if (addr == NULL) {
        link->si.sin_addr.s_addr = htonl(INADDR_ANY);
    } else {
        link->si.sin_addr.s_addr = inet_addr(addr);
    }

    if (isServer) {
        if (bind(fd, (const struct sockaddr *)&link->si, sizeof(link->si)) == SOCKET_ERROR) {
            closesocket(fd);
            return -1;
        }
    }

    return 0;
}

int udpSend(udpLink_t *link, const void *data, size_t size)
{
    return sendto((SOCKET)link->fd, data, (int)size, 0,
                  (const struct sockaddr *)&link->si, sizeof(link->si));
}

int udpRecv(udpLink_t *link, void *data, size_t size, uint32_t timeout_ms)
{
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET((SOCKET)link->fd, &fds);

    tv.tv_sec = (long)(timeout_ms / 1000);
    tv.tv_usec = (long)((timeout_ms % 1000) * 1000UL);

    const int ready = select(0, &fds, NULL, NULL, &tv);
    if (ready != 1) {
        return -1;
    }

#ifdef SITL_UDP_TIME
    if (size == SITL_FDM_PACKET_SIZE && link->port == SITL_FDM_PORT) {
        // Read into a buffer large enough for the optional telemetry tail so
        // recvfrom() does not truncate it, then hand the firmware exactly the
        // official fdm_packet bytes.
        uint8_t rxBuf[SITL_FDM_EXTENDED_SIZE];
        int rxLen = (int)sizeof(link->si);
        const int ret = recvfrom((SOCKET)link->fd, (char *)rxBuf, (int)sizeof(rxBuf), 0,
                                 (struct sockaddr *)&link->si, &rxLen);
        if (ret < SITL_FDM_PACKET_SIZE) {
            return -1;
        }
        memcpy(data, rxBuf, SITL_FDM_PACKET_SIZE);

        if (ret >= SITL_FDM_EXTENDED_SIZE) {
            double voltage = 0.0;
            double current = 0.0;
            double rpm[SITL_FDM_EXT_MOTOR_RPM_COUNT] = { 0.0 };
            memcpy(&voltage, rxBuf + SITL_FDM_EXT_BATTERY_VOLTAGE, sizeof(voltage));
            memcpy(&current, rxBuf + SITL_FDM_EXT_BATTERY_CURRENT, sizeof(current));
            for (int i = 0; i < SITL_FDM_EXT_MOTOR_RPM_COUNT; i++) {
                memcpy(&rpm[i], rxBuf + SITL_FDM_EXT_MOTOR_RPM + i * (int)sizeof(double), sizeof(double));
            }
            simTelemetrySet(voltage, current, rpm, SITL_FDM_EXT_MOTOR_RPM_COUNT);
        }

        const double ts = *(const double *)data;

        if (gFdmLastTs >= 0.0 && ts > gFdmLastTs) {
            const double deltaUs = (ts - gFdmLastTs) * 1e6 + gFdmRemainderUs;
            if (deltaUs > 0.0 && deltaUs <= SITL_MAX_FDM_DELTA_US) {
                const LONG64 wholeUs = (LONG64)deltaUs;
                gFdmRemainderUs = deltaUs - (double)wholeUs;
                InterlockedExchangeAdd64(&gFdmPendingUs, wholeUs);
                if (gFdmEvent != NULL) {
                    SetEvent(gFdmEvent);
                }
            } else {
                gFdmRemainderUs = 0.0;
            }
        } else if (ts < gFdmLastTs) {
            // Simulator clock restarted; resync the delta baseline only.
            gFdmRemainderUs = 0.0;
        }
        gFdmLastTs = ts;
        InterlockedIncrement(&gFdmPacketCount);
        return SITL_FDM_PACKET_SIZE;
    }
#endif

    int len = (int)sizeof(link->si);
    const int ret = recvfrom((SOCKET)link->fd, data, (int)size, 0,
                             (struct sockaddr *)&link->si, &len);
    return ret;
}
