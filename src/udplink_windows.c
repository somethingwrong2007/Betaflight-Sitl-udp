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

static bool wsaInitialized = false;

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

    int len = (int)sizeof(link->si);
    const int ret = recvfrom((SOCKET)link->fd, data, (int)size, 0,
                             (struct sockaddr *)&link->si, &len);
    return ret;
}
