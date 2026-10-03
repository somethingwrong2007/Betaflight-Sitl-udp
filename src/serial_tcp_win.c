/**
 * Windows TCP serial-port bridge for Betaflight SITL.
 *
 * The official serial_tcp.c relies on dyad's single-threaded event loop, which
 * races with the SITL tcpWorker thread on Windows: dyad_update() can destroy a
 * newly created CLOSED stream before serial_tcp.c finishes turning it into a
 * listening socket, causing heap corruption. This implementation replaces the
 * dyad-based bridge with one Winsock listener thread per opened UART port.
 */

#include <winsock2.h>
#include <ws2tcpip.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"
#include "drivers/serial_tcp.h"
#include "io/serial.h"
#include "win_socket_util.h"

#define BASE_PORT 5760
#define MAX_TCP_CLIENTS 8

void wsProxyStart(void);

static tcpPort_t tcpSerialPorts[SERIAL_PORT_COUNT];
static bool tcpPortInitialized[SERIAL_PORT_COUNT];
static SOCKET listenSockets[SERIAL_PORT_COUNT];
static SOCKET clientSockets[SERIAL_PORT_COUNT][MAX_TCP_CLIENTS];
static pthread_t clientThreads[SERIAL_PORT_COUNT][MAX_TCP_CLIENTS];
static pthread_mutex_t clientLocks[SERIAL_PORT_COUNT];
static pthread_t serverThreads[SERIAL_PORT_COUNT];
static bool tcpStart = false;

static const struct serialPortVTable tcpVTable;

static bool wsaReady = false;

static void ensureWsa(void)
{
    if (!wsaReady) {
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0) {
            wsaReady = true;
        }
    }
}

typedef struct {
    int id;
    int slot;
    SOCKET sock;
} tcpClientArg_t;

static void tcpClientDisconnect(tcpPort_t *s, int id, int slot, SOCKET sock)
{
    bool wasActive = false;

    pthread_mutex_lock(&clientLocks[id]);
    if (clientSockets[id][slot] == sock) {
        clientSockets[id][slot] = INVALID_SOCKET;
        wasActive = true;
        if (s->clientCount > 0) {
            s->clientCount--;
        }
        if (s->clientCount == 0) {
            s->connected = false;
        }
    }
    pthread_mutex_unlock(&clientLocks[id]);

    if (wasActive) {
        closesocket(sock);

        // If the last client went away while the FC was in CLI mode, push an
        // "exit noreboot" line so the CLI session ends and the next
        // connection starts clean in MSP mode. Plain "exit" would reboot the
        // FC, and on SITL that terminates the process. In MSP mode these
        // bytes are ignored as non-MSP noise, so the injection is safe either
        // way.
        if (s->clientCount == 0) {
            static const char exitCliCmd[] = "exit noreboot\r";
            tcpDataIn(s, (uint8_t *)exitCliCmd, (int)sizeof(exitCliCmd) - 1);
        }
    }
}

static void *tcpClientThread(void *arg)
{
    tcpClientArg_t *a = (tcpClientArg_t *)arg;
    tcpPort_t *s = &tcpSerialPorts[a->id];
    uint8_t buf[2048];

    for (;;) {
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(a->sock, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 50000; // 50 ms poll; sockets are non-blocking
        const int ready = select(0, &rfds, NULL, NULL, &tv);
        if (ready <= 0) {
            continue;
        }
        const int n = recv(a->sock, (char *)buf, sizeof(buf), 0);
        if (n <= 0) {
            break;
        }
        tcpDataIn(s, buf, n);
    }

    tcpClientDisconnect(s, a->id, a->slot, a->sock);
    free(a);
    return NULL;
}

static void *tcpServerThread(void *arg)
{
    const int id = (int)(intptr_t)arg;
    tcpPort_t *s = &tcpSerialPorts[id];
    SOCKET listenSock = listenSockets[id];

    for (;;) {
        SOCKET client = accept(listenSock, NULL, NULL);
        if (client == INVALID_SOCKET) {
            continue;
        }
        socketNoInherit(client);
        // Non-blocking: a peer that stops reading must never stall the FC
        // thread inside send() (see tcpDataOut WSAEWOULDBLOCK handling).
        {
            u_long nonblocking = 1;
            ioctlsocket(client, FIONBIO, &nonblocking);
        }
        // Disable Nagle: replies are small and should go out immediately
        // instead of waiting for the delayed-ACK timer.
        {
            BOOL noDelay = TRUE;
            setsockopt(client, IPPROTO_TCP, TCP_NODELAY, (const char *)&noDelay, sizeof(noDelay));
        }

        pthread_mutex_lock(&clientLocks[id]);
        int slot = -1;
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if (clientSockets[id][i] == INVALID_SOCKET) {
                slot = i;
                break;
            }
        }
        if (slot >= 0) {
            clientSockets[id][slot] = client;
            s->clientCount++;
            s->connected = true;
        }
        pthread_mutex_unlock(&clientLocks[id]);

        if (slot < 0) {
            closesocket(client);
            continue;
        }

        tcpClientArg_t *a = (tcpClientArg_t *)malloc(sizeof(*a));
        if (a == NULL) {
            tcpClientDisconnect(s, id, slot, client);
            continue;
        }
        a->id = id;
        a->slot = slot;
        a->sock = client;

        if (pthread_create(&clientThreads[id][slot], NULL, tcpClientThread, a) != 0) {
            tcpClientDisconnect(s, id, slot, client);
            free(a);
            continue;
        }
        pthread_detach(clientThreads[id][slot]);
    }

    return NULL;
}

static int tcpReconfigure(tcpPort_t *s, int id)
{
    if (tcpPortInitialized[id]) {
        return 0;
    }

    if (pthread_mutex_init(&s->txLock, NULL) != 0) {
        return -1;
    }
    if (pthread_mutex_init(&s->rxLock, NULL) != 0) {
        return -1;
    }
    if (pthread_mutex_init(&clientLocks[id], NULL) != 0) {
        return -1;
    }
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        clientSockets[id][i] = INVALID_SOCKET;
    }

    ensureWsa();

    SOCKET listenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSock == INVALID_SOCKET) {
        return -1;
    }
    socketNoInherit(listenSock);

    BOOL reuse = TRUE;
    setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((u_short)(BASE_PORT + id + 1));

    if (bind(listenSock, (const struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR) {
        closesocket(listenSock);
        return -1;
    }
    if (listen(listenSock, 8) == SOCKET_ERROR) {
        closesocket(listenSock);
        return -1;
    }

    listenSockets[id] = listenSock;

    s->connected = false;
    s->clientCount = 0;
    s->id = (uint8_t)id;

    if (pthread_create(&serverThreads[id], NULL, tcpServerThread, (void *)(intptr_t)id) != 0) {
        closesocket(listenSock);
        return -1;
    }

    tcpPortInitialized[id] = true;
    tcpStart = true;
    fprintf(stderr, "bind port %u for UART%u\n", (unsigned)(BASE_PORT + id + 1), (unsigned)id + 1);
    wsProxyStart();
    return 0;
}

serialPort_t *serTcpOpen(serialPortIdentifier_e identifier, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    const int id = findSerialPortIndexByIdentifier(identifier);
    if (id < 0 || id >= (int)ARRAYLEN(tcpSerialPorts)) {
        return NULL;
    }

    tcpPort_t *s = &tcpSerialPorts[id];
    if (tcpReconfigure(s, id) != 0) {
        return NULL;
    }

    s->port.vTable = &tcpVTable;

    // Common serial port initialisation
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    s->port.rxBufferSize = RX_BUFFER_SIZE;
    s->port.txBufferSize = TX_BUFFER_SIZE;
    s->port.rxBuffer = s->rxBuffer;
    s->port.txBuffer = s->txBuffer;
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    return (serialPort_t *)s;
}

static uint32_t tcpTotalRxBytesWaiting(const serialPort_t *instance)
{
    const tcpPort_t *s = (const tcpPort_t *)instance;
    pthread_mutex_lock((pthread_mutex_t *)&s->rxLock);
    const uint32_t count = (s->port.rxBufferHead >= s->port.rxBufferTail)
        ? (uint32_t)(s->port.rxBufferHead - s->port.rxBufferTail)
        : (uint32_t)(s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail);
    pthread_mutex_unlock((pthread_mutex_t *)&s->rxLock);
    return count;
}

static uint32_t tcpTotalTxBytesFree(const serialPort_t *instance)
{
    const tcpPort_t *s = (const tcpPort_t *)instance;
    pthread_mutex_lock((pthread_mutex_t *)&s->txLock);
    const uint32_t bytesUsed = (s->port.txBufferHead >= s->port.txBufferTail)
        ? (uint32_t)(s->port.txBufferHead - s->port.txBufferTail)
        : (uint32_t)(s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail);
    const uint32_t bytesFree = (s->port.txBufferSize - 1) - bytesUsed;
    pthread_mutex_unlock((pthread_mutex_t *)&s->txLock);
    return bytesFree;
}

static bool isTcpTransmitBufferEmpty(const serialPort_t *instance)
{
    const tcpPort_t *s = (const tcpPort_t *)instance;
    pthread_mutex_lock((pthread_mutex_t *)&s->txLock);
    const bool empty = s->port.txBufferTail == s->port.txBufferHead;
    pthread_mutex_unlock((pthread_mutex_t *)&s->txLock);
    return empty;
}

static uint8_t tcpRead(serialPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->rxLock);
    const uint8_t ch = s->port.rxBuffer[s->port.rxBufferTail];
    if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
        s->port.rxBufferTail = 0;
    } else {
        s->port.rxBufferTail++;
    }
    pthread_mutex_unlock(&s->rxLock);
    return ch;
}

static void tcpWrite(serialPort_t *instance, uint8_t ch)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->txLock);
    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }
    pthread_mutex_unlock(&s->txLock);
    tcpDataOut(s);
}

// Batch write: without this the generic serialWriteBufNoFlush() falls back to
// one blocking send() per byte (Nagle delays make each byte take tens of ms),
// which stalls MSP replies and makes the configurator time out on back-to-back
// requests. The caller has already checked that the TX ring can hold `count`.
static void tcpWriteBuf(serialPort_t *instance, const void *dataPtr, int count)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    const uint8_t *data = (const uint8_t *)dataPtr;
    pthread_mutex_lock(&s->txLock);
    for (int i = 0; i < count; i++) {
        s->port.txBuffer[s->port.txBufferHead] = data[i];
        if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
            s->port.txBufferHead = 0;
        } else {
            s->port.txBufferHead++;
        }
    }
    pthread_mutex_unlock(&s->txLock);
    tcpDataOut(s);
}

void tcpDataOut(tcpPort_t *instance)
{
    tcpPort_t *s = instance;
    uint8_t data[TX_BUFFER_SIZE];
    size_t len = 0;

    pthread_mutex_lock(&s->txLock);
    if (s->port.txBufferHead < s->port.txBufferTail) {
        const size_t chunk = s->port.txBufferSize - s->port.txBufferTail;
        memcpy(data, (const void *)&s->port.txBuffer[s->port.txBufferTail], chunk);
        len += chunk;
        s->port.txBufferTail = 0;
    }
    if (s->port.txBufferHead > s->port.txBufferTail) {
        const size_t chunk = s->port.txBufferHead - s->port.txBufferTail;
        memcpy(data + len, (const void *)&s->port.txBuffer[s->port.txBufferTail], chunk);
        len += chunk;
    }
    s->port.txBufferTail = s->port.txBufferHead;
    pthread_mutex_unlock(&s->txLock);

    if (len == 0) {
        return;
    }
    pthread_mutex_lock(&clientLocks[s->id]);
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        const SOCKET sock = clientSockets[s->id][i];
        if (sock == INVALID_SOCKET) {
            continue;
        }
        int sentTotal = 0;
        bool failed = false;
        while (sentTotal < (int)len) {
            const int sent = send(sock, (const char *)data + sentTotal, (int)len - sentTotal, 0);
            if (sent == SOCKET_ERROR) {
                // Non-blocking socket: if the peer is not reading, drop the
                // remainder instead of blocking forever (a stalled peer must
                // never freeze the FC thread that owns the MSP/CLI mutex).
                if (WSAGetLastError() == WSAEWOULDBLOCK) {
                    break;
                }
                failed = true;
                break;
            }
            if (sent <= 0) {
                break;
            }
            sentTotal += sent;
        }
        if (failed) {
            closesocket(sock);
            clientSockets[s->id][i] = INVALID_SOCKET;
            if (s->clientCount > 0) {
                s->clientCount--;
            }
            if (s->clientCount == 0) {
                s->connected = false;
            }
        }
    }
    pthread_mutex_unlock(&clientLocks[s->id]);
}

void tcpDataIn(tcpPort_t *instance, uint8_t *ch, int size)
{
    tcpPort_t *s = instance;
    pthread_mutex_lock(&s->rxLock);
    const uint32_t bufSize = s->port.rxBufferSize;
    while (size--) {
        const uint32_t used = (s->port.rxBufferHead >= s->port.rxBufferTail)
            ? (s->port.rxBufferHead - s->port.rxBufferTail)
            : (bufSize - s->port.rxBufferTail + s->port.rxBufferHead);
        if (used >= bufSize - 1) {
            // Ring buffer full: drop the rest of this packet instead of
            // corrupting the stream.
            break;
        }
        s->port.rxBuffer[s->port.rxBufferHead] = *(ch++);
        if (s->port.rxBufferHead + 1 >= s->port.rxBufferSize) {
            s->port.rxBufferHead = 0;
        } else {
            s->port.rxBufferHead++;
        }
    }
    pthread_mutex_unlock(&s->rxLock);
}

static const struct serialPortVTable tcpVTable = {
    .serialWrite = tcpWrite,
    .serialTotalRxWaiting = tcpTotalRxBytesWaiting,
    .serialTotalTxFree = tcpTotalTxBytesFree,
    .serialRead = tcpRead,
    .serialSetBaudRate = NULL,
    .isSerialTransmitBufferEmpty = isTcpTransmitBufferEmpty,
    .setMode = NULL,
    .setCtrlLineStateCb = NULL,
    .setBaudRateCb = NULL,
    .writeBuf = tcpWriteBuf,
    .beginWrite = NULL,
    .endWrite = NULL,
};

bool tcpIsStart(void)
{
    return tcpStart;
}

bool *tcpGetUsed(void)
{
    return tcpPortInitialized;
}

tcpPort_t *tcpGetPool(void)
{
    return tcpSerialPorts;
}
