/**
 * Built-in WebSocket proxy for the Betaflight web configurator.
 *
 * app.betaflight.com talks WebSocket, while Betaflight SITL exposes MSP over
 * plain TCP on port 5761. This module listens on ws://127.0.0.1:6761 and
 * bridges WebSocket binary frames to the local TCP MSP port, replacing the
 * external websockify step described in the official SITL docs.
 */

#include <winsock2.h>
#include <ws2tcpip.h>
#include <wincrypt.h>
#include <windows.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include "win_socket_util.h"

extern void sitlAuditLog(const char *fmt, ...);

#define WS_PORT 6761
#define MSP_PORT 5761
#define WS_GUID "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

static SOCKET wsListenSocket = INVALID_SOCKET;
static pthread_t wsListenThreadHandle;
static bool wsStarted = false;

static void sendAll(SOCKET sock, const uint8_t *data, size_t len)
{
    while (len > 0) {
        const int sent = send(sock, (const char *)data, (int)len, 0);
        if (sent == SOCKET_ERROR || sent <= 0) {
            break;
        }
        data += sent;
        len -= (size_t)sent;
    }
}

static bool readFull(SOCKET sock, uint8_t *buf, size_t len)
{
    size_t got = 0;
    while (got < len) {
        const int n = recv(sock, (char *)buf + got, (int)(len - got), 0);
        if (n <= 0) {
            return false;
        }
        got += (size_t)n;
    }
    return true;
}

static int computeAccept(const char *key, char *out, size_t outSize)
{
    char data[512];
    BYTE digest[20];
    DWORD digestLen = sizeof(digest);
    DWORD outLen = (DWORD)outSize;
    HCRYPTPROV prov = 0;
    HCRYPTHASH hash = 0;

    _snprintf(data, sizeof(data), "%s%s", key, WS_GUID);

    if (!CryptAcquireContext(&prov, NULL, NULL, PROV_RSA_FULL, CRYPT_VERIFYCONTEXT)) {
        return -1;
    }
    if (!CryptCreateHash(prov, CALG_SHA1, 0, 0, &hash)) {
        CryptReleaseContext(prov, 0);
        return -1;
    }
    if (!CryptHashData(hash, (const BYTE *)data, (DWORD)strlen(data), 0)) {
        CryptDestroyHash(hash);
        CryptReleaseContext(prov, 0);
        return -1;
    }
    if (!CryptGetHashParam(hash, HP_HASHVAL, digest, &digestLen, 0)) {
        CryptDestroyHash(hash);
        CryptReleaseContext(prov, 0);
        return -1;
    }
    if (!CryptBinaryToStringA(digest, digestLen, CRYPT_STRING_BASE64 | CRYPT_STRING_NOCRLF, out, &outLen)) {
        CryptDestroyHash(hash);
        CryptReleaseContext(prov, 0);
        return -1;
    }

    CryptDestroyHash(hash);
    CryptReleaseContext(prov, 0);
    return 0;
}

static bool wsHandshake(SOCKET sock)
{
    char buf[8192];
    size_t total = 0;

    while (total < sizeof(buf) - 1) {
        const int n = recv(sock, buf + total, (int)(sizeof(buf) - 1 - total), 0);
        if (n <= 0) {
            return false;
        }
        total += (size_t)n;
        buf[total] = '\0';
        if (strstr(buf, "\r\n\r\n") != NULL) {
            break;
        }
    }

    const char *keyHeader = strstr(buf, "Sec-WebSocket-Key:");
    if (keyHeader == NULL) {
        keyHeader = strstr(buf, "sec-websocket-key:");
    }
    if (keyHeader == NULL) {
        return false;
    }

    keyHeader += strlen("Sec-WebSocket-Key:");
    while (*keyHeader == ' ' || *keyHeader == '\t') {
        keyHeader++;
    }

    char key[256];
    size_t keyLen = 0;
    while (keyHeader[keyLen] != '\r' && keyHeader[keyLen] != '\n' && keyLen < sizeof(key) - 1) {
        key[keyLen] = keyHeader[keyLen];
        keyLen++;
    }
    key[keyLen] = '\0';

    char accept[128];
    if (computeAccept(key, accept, sizeof(accept)) != 0) {
        return false;
    }

    fprintf(stderr, "[wsproxy] key=[%s] accept=[%s]\n", key, accept);

    char subprotocol[128] = "";
    const char *protoHeader = strstr(buf, "Sec-WebSocket-Protocol:");
    if (protoHeader == NULL) {
        protoHeader = strstr(buf, "sec-websocket-protocol:");
    }
    if (protoHeader != NULL) {
        protoHeader += strlen("Sec-WebSocket-Protocol:");
        while (*protoHeader == ' ' || *protoHeader == '\t') {
            protoHeader++;
        }
        size_t protoLen = 0;
        while (protoHeader[protoLen] != '\r' && protoHeader[protoLen] != '\n'
               && protoHeader[protoLen] != ',' && protoLen < sizeof(subprotocol) - 1) {
            subprotocol[protoLen] = protoHeader[protoLen];
            protoLen++;
        }
        subprotocol[protoLen] = '\0';
        size_t end = protoLen;
        while (end > 0 && (subprotocol[end - 1] == ' ' || subprotocol[end - 1] == '\t')) {
            end--;
        }
        subprotocol[end] = '\0';
    }

    char response[512];
    int len;
    if (subprotocol[0] != '\0') {
        len = _snprintf(response, sizeof(response),
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: %s\r\n"
            "Sec-WebSocket-Protocol: %s\r\n"
            "\r\n", accept, subprotocol);
    } else {
        len = _snprintf(response, sizeof(response),
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: %s\r\n"
            "\r\n", accept);
    }
    if (len <= 0) {
        return false;
    }
    fprintf(stderr, "[wsproxy] request-headers:\n%.*s\n[wsproxy] response:\n%s\n", (int)total, buf, response);
    sendAll(sock, (const uint8_t *)response, (size_t)len);
    return true;
}

static void sendWsFrame(SOCKET sock, uint8_t opcode, const uint8_t *payload, size_t len)
{
    uint8_t header[14];
    size_t h = 0;

    header[h++] = (uint8_t)(0x80 | opcode);
    if (len < 126) {
        header[h++] = (uint8_t)len;
    } else if (len < 65536) {
        header[h++] = 126;
        header[h++] = (uint8_t)(len >> 8);
        header[h++] = (uint8_t)len;
    } else {
        header[h++] = 127;
        for (int i = 7; i >= 0; i--) {
            header[h++] = (uint8_t)(len >> (i * 8));
        }
    }

    sendAll(sock, header, h);
    sendAll(sock, payload, len);
}

static bool readWsFrame(SOCKET sock, uint8_t *payload, size_t payloadSize, size_t *payloadLen, bool *close)
{
    uint8_t header[2];
    uint8_t extended[8];
    uint8_t maskKey[4];
    uint64_t len = 0;

    if (!readFull(sock, header, 2)) {
        return false;
    }

    const uint8_t opcode = header[0] & 0x0F;
    const bool masked = (header[1] & 0x80) != 0;
    len = header[1] & 0x7F;

    if (len == 126) {
        if (!readFull(sock, extended, 2)) {
            return false;
        }
        len = ((uint64_t)extended[0] << 8) | extended[1];
    } else if (len == 127) {
        if (!readFull(sock, extended, 8)) {
            return false;
        }
        len = 0;
        for (int i = 0; i < 8; i++) {
            len = (len << 8) | extended[i];
        }
    }

    if (masked) {
        if (!readFull(sock, maskKey, 4)) {
            return false;
        }
    }

    if (len > payloadSize) {
        return false;
    }

    if (len > 0 && !readFull(sock, payload, (size_t)len)) {
        return false;
    }

    if (masked) {
        for (uint64_t i = 0; i < len; i++) {
            payload[i] ^= maskKey[i % 4];
        }
    }

    *payloadLen = (size_t)len;

    if (opcode == 0x8) {
        *close = true;
    } else if (opcode == 0x9) {
        sendWsFrame(sock, 0xA, payload, (size_t)len);
    } else if (opcode == 0x1 || opcode == 0x2) {
        *close = false;
        return true;
    }

    return !(*close);
}

static void *wsClientThread(void *arg)
{
    const SOCKET ws = (SOCKET)(intptr_t)arg;

    if (!wsHandshake(ws)) {
        fprintf(stderr, "[wsproxy] handshake failed\n");
        closesocket(ws);
        return NULL;
    }
    fprintf(stderr, "[wsproxy] WebSocket client connected\n");
    sitlAuditLog("configurator connected via WebSocket");

    SOCKET tcp = INVALID_SOCKET;
    for (int attempt = 0; attempt < 50; attempt++) {
        tcp = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (tcp != INVALID_SOCKET) {
            socketNoInherit(tcp);
            struct sockaddr_in addr;
            memset(&addr, 0, sizeof(addr));
            addr.sin_family = AF_INET;
            addr.sin_port = htons(MSP_PORT);
            addr.sin_addr.s_addr = inet_addr("127.0.0.1");

            if (connect(tcp, (const struct sockaddr *)&addr, sizeof(addr)) != SOCKET_ERROR) {
                break;
            }
            closesocket(tcp);
            tcp = INVALID_SOCKET;
        }
        if (attempt < 49) {
            Sleep(100);
        }
    }

    if (tcp == INVALID_SOCKET) {
        fprintf(stderr, "[wsproxy] could not reach MSP port %d\n", MSP_PORT);
        closesocket(ws);
        return NULL;
    }
    fprintf(stderr, "[wsproxy] MSP link established\n");

    uint8_t tcpBuf[4096];
    uint8_t wsPayload[65536];
    bool wsClosed = false;

    while (!wsClosed) {
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(ws, &rfds);
        FD_SET(tcp, &rfds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        const int ready = select(0, &rfds, NULL, NULL, &tv);
        if (ready <= 0) {
            continue;
        }

        if (FD_ISSET(tcp, &rfds)) {
            const int n = recv(tcp, (char *)tcpBuf, sizeof(tcpBuf), 0);
            if (n <= 0) {
                break;
            }
            sendWsFrame(ws, 0x2, tcpBuf, (size_t)n);
        }

        if (FD_ISSET(ws, &rfds)) {
            size_t payloadLen = 0;
            if (!readWsFrame(ws, wsPayload, sizeof(wsPayload), &payloadLen, &wsClosed)) {
                break;
            }
            if (payloadLen > 0) {
                sendAll(tcp, wsPayload, payloadLen);
            }
        }
    }

    fprintf(stderr, "[wsproxy] client disconnected\n");
    sitlAuditLog("configurator disconnected");
    closesocket(tcp);
    closesocket(ws);
    return NULL;
}

static void *wsListenThread(void *arg)
{
    (void)arg;
    for (;;) {
        SOCKET client = accept(wsListenSocket, NULL, NULL);
        if (client == INVALID_SOCKET) {
            continue;
        }
        socketNoInherit(client);
        pthread_t thread;
        pthread_create(&thread, NULL, wsClientThread, (void *)(intptr_t)client);
        pthread_detach(thread);
    }
    return NULL;
}

void wsProxyStart(void)
{
    if (wsStarted) {
        return;
    }

    wsListenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (wsListenSocket == INVALID_SOCKET) {
        return;
    }
    socketNoInherit(wsListenSocket);

    BOOL reuse = TRUE;
    setsockopt(wsListenSocket, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(WS_PORT);

    if (bind(wsListenSocket, (const struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR) {
        closesocket(wsListenSocket);
        wsListenSocket = INVALID_SOCKET;
        return;
    }
    if (listen(wsListenSocket, 8) == SOCKET_ERROR) {
        closesocket(wsListenSocket);
        wsListenSocket = INVALID_SOCKET;
        return;
    }

    if (pthread_create(&wsListenThreadHandle, NULL, wsListenThread, NULL) != 0) {
        closesocket(wsListenSocket);
        wsListenSocket = INVALID_SOCKET;
        return;
    }
    pthread_detach(wsListenThreadHandle);

    wsStarted = true;
    fprintf(stderr, "WebSocket proxy listening on ws://127.0.0.1:%d\n", WS_PORT);
}
