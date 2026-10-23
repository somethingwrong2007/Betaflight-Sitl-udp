/*
 * sitl_local_reconnect_probe - reproduces what the ground station does around a
 * flight-controller restart, so "the configurator keeps failing to reconnect"
 * can be told apart from "the flight controller never answers again".
 *
 * It hosts betaflight_SITL.dll in LOCAL mode and, for both restart flavours,
 * probes the MSP link the way the configurator does:
 *
 *   1. baseline        connect to the MSP port, MSP_API_VERSION
 *   2. restart A       sitl_local_shutdown() + sitl_local_init()
 *                      (what the Unreal host does: level reload, cold restart)
 *      A1 kept link   the pre-restart connection, still open: does it answer?
 *      A2 fresh link  a new connection: does it answer?
 *   3. restart B       MSP_SET_REBOOT on a live link ("Save and Reboot")
 *      B1 kept link   same question
 *      B2 fresh link  same question
 *
 * Each probe retries for up to 20 s on a 1 s cadence (the configurator's reboot
 * window) and reports WHY a link failed - refused, or accepted but silent -
 * instead of a bare timeout.
 *
 * Build (WSL, same toolchain as the DLL; links against it like the tester
 * does):
 *   cd build-win-local
 *   x86_64-w64-mingw32-gcc -O2 -I../src -o reconnect_probe.exe \
 *       ../tools/sitl_local_reconnect_probe.c -L. -lbetaflight_SITL -lws2_32
 *
 * Run next to a live host without fighting it for the ports (the DLL moves the
 * whole block with BF_SITL_TCP_BASE; the MSP port is base + 1):
 *   set BF_SITL_TCP_BASE=15760
 *   set BF_SITL_WS_PORT=16761
 *   reconnect_probe.exe
 *
 * The "link open but never answered" baseline this probe first reported was a
 * bug in the probe, not in the flight controller: it framed MSPv1 with
 * size = payload + 1, while the firmware's header carries the plain payload
 * length (msp_serial.c: hdrV1->size = dataLen). Every request was therefore one
 * byte short for the FC and it waited for a payload byte that never arrived.
 * The reader had the matching off-by-one. Both are fixed; a healthy FC answers
 * MSP_API_VERSION on the first attempt now.
 *
 * Two environment traps found while writing it, worth knowing for any future
 * harness:
 *   - the DLL redirects stdout to NUL during boot (src/main_windows.c), so a
 *     host must report on stderr;
 *   - BF_SITL_EEPROM must point at a scratch file, or a harness shares (and
 *     can overwrite) the aircraft config the running game uses.
 */

#define WIN32_LEAN_AND_MEAN
#include <stdint.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Resolved from the import library, exactly like tools/sitl_local_tester.c.
 * The C signatures live in src/sitl_local.h; the input/output structs are
 * opaque byte buffers here, which is all this probe needs. */
extern int sitl_local_init(void);
extern void sitl_local_step(const void *in, unsigned int dtUs, void *out);
extern void sitl_local_shutdown(void);

/* Optional: older builds do not export it (the Unreal link resolves it the
 * same way). */
typedef int (*fn_get_log)(char *out, int size);
static fn_get_log gGetLog = NULL;

static void resolve_optional_exports(void)
{
    HMODULE self = GetModuleHandleA("betaflight_SITL.dll");
    if (self != NULL) {
        gGetLog = (fn_get_log)(void *)GetProcAddress(self, "sitl_local_get_log");
    }
}

#define MSP_API_VERSION 1
#define MSP_SET_REBOOT 68

#define INPUT_BYTES 256   /* sitl_local_input_t + the temperature tail */
#define OUTPUT_BYTES 256

static const char *gDllDir = NULL;

static void sleep_ms(int ms)
{
    Sleep((DWORD)ms);
}

static unsigned long now_ms(void)
{
    return (unsigned long)GetTickCount();
}

/* ------------------------------------------------------------------ */
/* MSP over TCP (the transport the configurator's WebSocket bridges to) */
/* ------------------------------------------------------------------ */

static SOCKET tcp_connect(uint16_t port, int timeoutMs)
{
    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) {
        return INVALID_SOCKET;
    }
    u_long nb = 1;
    ioctlsocket(s, FIONBIO, &nb);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (connect(s, (const struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR) {
        fd_set wfds;
        struct timeval tv;
        FD_ZERO(&wfds);
        FD_SET(s, &wfds);
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
        if (select(0, NULL, &wfds, NULL, &tv) <= 0) {
            closesocket(s);
            return INVALID_SOCKET;
        }
    }
    nb = 0;
    ioctlsocket(s, FIONBIO, &nb);
    return s;
}

static int msp_send(SOCKET s, uint8_t cmd, const uint8_t *payload, int len)
{
    uint8_t frame[64];
    // MSPv1's size field is the *payload* length (msp_serial.c: hdrV1->size =
    // dataLen), and the checksum covers size ^ cmd ^ payload. Using len + 1
    // here made every request one byte short for the FC, so it waited for a
    // payload byte that never came and never answered ("link open, silent").
    const int size = len;
    int n = 0;
    frame[n++] = '$';
    frame[n++] = 'M';
    frame[n++] = '<';
    frame[n++] = (uint8_t)size;
    frame[n++] = cmd;
    uint8_t crc = (uint8_t)(size ^ cmd);
    for (int i = 0; i < len; i++) {
        frame[n++] = payload[i];
        crc ^= payload[i];
    }
    frame[n++] = crc;
    return send(s, (const char *)frame, n, 0) == n ? 0 : -1;
}

/* Reads until one complete MSP v1 frame arrives (or the timeout expires).
 * Returns: 1 = reply, 0 = nothing, -1 = link closed. */
static int msp_read_frame(SOCKET s, uint8_t *cmdOut, uint8_t *payloadOut, int *lenOut,
                          int timeoutMs)
{
    static uint8_t buf[1024];
    static int have = 0;
    const unsigned long deadline = now_ms() + (unsigned long)timeoutMs;

    for (;;) {
        for (int i = 0; i + 4 <= have; i++) {
            if (buf[i] != '$' || buf[i + 1] != 'M') {
                continue;
            }
            const uint8_t dir = buf[i + 2];
            const int size = buf[i + 3];      // payload length (see msp_send)
            if (dir != '>' && dir != '!') {
                continue;
            }
            if (i + 6 + size > have) {
                continue;   /* frame still incomplete */
            }
            const int cmd = buf[i + 4];
            uint8_t crc = buf[i + 3] ^ buf[i + 4];
            for (int k = 0; k < size; k++) {
                crc ^= buf[i + 5 + k];
            }
            if (crc != buf[i + 5 + size]) {
                continue;
            }
            *cmdOut = (uint8_t)cmd;
            *lenOut = size;
            if (size > 0) {
                memcpy(payloadOut, &buf[i + 5], (size_t)size);
            }
            const int used = i + 6 + size;
            memmove(buf, buf + used, (size_t)(have - used));
            have -= used;
            return dir == '>' ? 1 : -1;
        }

        if (now_ms() >= deadline) {
            return 0;
        }
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(s, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 200000;
        const int ready = select(0, &rfds, NULL, NULL, &tv);
        if (ready <= 0) {
            continue;
        }
        const int n = recv(s, (char *)buf + have, (int)sizeof(buf) - have, 0);
        if (n <= 0) {
            return -1;
        }
        have += n;
        if (have == (int)sizeof(buf)) {
            have = 0;   /* never happens with our frame sizes, but stay safe */
        }
    }
}

/* One MSP round trip: 1 = answered, 0 = silent, -1 = link closed. */
static int msp_request(SOCKET s, uint8_t cmd, const uint8_t *payload, int len,
                       uint8_t *reply, int *replyLen, int timeoutMs)
{
    if (msp_send(s, cmd, payload, len) != 0) {
        return -1;
    }
    uint8_t rcmd = 0;
    const int r = msp_read_frame(s, &rcmd, reply, replyLen, timeoutMs);
    return r;
}

/* ------------------------------------------------------------------ */
/* DLL                                                                 */
/* ------------------------------------------------------------------ */

static void *gIn = NULL;
static void *gOut = NULL;

static void step_fc(int steps)
{
    for (int i = 0; i < steps; i++) {
        if (i < 3 || (i % 50) == 49) {
            fprintf(stderr, "[probe]   step %d\n", i);
        }
        sitl_local_step(gIn, 1000, gOut);
    }
}

static void drain_fc_log(const char *tag)
{
    if (gGetLog == NULL) {
        return;
    }
    char chunk[4096];
    for (int guard = 0; guard < 64; guard++) {
        const int n = gGetLog(chunk, (int)sizeof(chunk) - 1);
        if (n <= 0) {
            break;
        }
        chunk[n] = '\0';
        fprintf(stderr, "  [fc/%s] %s", tag, chunk);
        if (chunk[n - 1] != '\n') {
            fprintf(stderr, "\n");
        }
    }
}

static uint16_t msp_port(void)
{
    const char *env = getenv("BF_SITL_TCP_BASE");
    const long base = (env != NULL && env[0] != '\0') ? strtol(env, NULL, 10) : 5760;
    return (uint16_t)(base + 1);
}

/* Retries like the configurator's reboot window: up to 20 s, 1 s cadence. */
static int probe_link(const char *label, SOCKET kept, uint16_t port,
                      const char *expected)
{
    const unsigned long deadline = now_ms() + 20000;
    int attempt = 0;
    int refused = 0, silent = 0, closed = 0;

    fprintf(stderr, "[probe] %s: expecting %s\n", label, expected);
    for (;;) {
        attempt++;
        SOCKET s = kept;
        int owned = 0;
        if (s == INVALID_SOCKET) {
            s = tcp_connect(port, 1000);
            if (s == INVALID_SOCKET) {
                refused++;
                fprintf(stderr, "  attempt %2d: connect refused/blocked\n", attempt);
                goto next;
            }
            owned = 1;
        }

        uint8_t reply[256];
        int replyLen = 0;
        const int r = msp_request(s, MSP_API_VERSION, NULL, 0, reply, &replyLen, 3000);
        if (r == 1) {
            fprintf(stderr, "  attempt %2d: answered (api %u.%u.%u) after %lu ms\n",
                   attempt,
                   replyLen > 0 ? reply[0] : 0,
                   replyLen > 1 ? reply[1] : 0,
                   replyLen > 2 ? reply[2] : 0,
                   now_ms() - (deadline - 20000));
            return s;
        }
        if (r == 0) {
            silent++;
            fprintf(stderr, "  attempt %2d: link open but no answer within 3 s\n", attempt);
        } else {
            closed++;
            fprintf(stderr, "  attempt %2d: link closed by the FC\n", attempt);
        }
        if (owned) {
            closesocket(s);
        } else if (r != 0) {
            closesocket(s);   /* a kept link that died is no longer usable */
            kept = INVALID_SOCKET;
            if (r == -1) {
                fprintf(stderr, "  (kept link closed - later attempts will reconnect)\n");
            }
        }

    next:
        if (now_ms() >= deadline) {
            fprintf(stderr, "[probe] %s: FAILED after %d attempts (%d refused, %d silent, %d closed)\n",
                   label, attempt, refused, silent, closed);
            return INVALID_SOCKET;
        }
        sleep_ms(1000);
    }
}

int main(int argc, char **argv)
{
    /* Report on stderr: the DLL's boot redirects stdout to NUL (see
     * src/main_windows.c's freopen), so anything printed there after
     * sitl_local_init() would vanish. Unbuffered because the DLL can also take
     * the process down, and buffered lines would be lost exactly when they
     * matter. */
    setvbuf(stderr, NULL, _IONBF, 0);
    (void)argc;
    (void)argv;

    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    resolve_optional_exports();

    gIn = calloc(1, INPUT_BYTES);
    gOut = calloc(1, OUTPUT_BYTES);
    if (gIn == NULL || gOut == NULL) {
        fprintf(stderr, "[probe] out of memory\n");
        return 2;
    }

    const uint16_t port = msp_port();
    fprintf(stderr, "[probe] MSP port %u (BF_SITL_TCP_BASE=%s)\n", (unsigned)port,
           getenv("BF_SITL_TCP_BASE") != NULL ? getenv("BF_SITL_TCP_BASE") : "unset");

    fprintf(stderr, "[probe] sitl_local_init()\n");
    if (sitl_local_init() != 0) {
        fprintf(stderr, "[probe] sitl_local_init() failed\n");
        return 2;
    }
    fprintf(stderr, "[probe] init returned\n");
    step_fc(50);
    fprintf(stderr, "[probe] stepped\n");
    drain_fc_log("init");
    fprintf(stderr, "[probe] log drained\n");

    SOCKET link = probe_link("baseline", INVALID_SOCKET, port, "MSP_API_VERSION");
    if (link == INVALID_SOCKET) {
        fprintf(stderr, "[probe] no baseline link; giving up\n");
        return 1;
    }

    /* --- restart A: the host's cold restart (shutdown + init) ------------- */
    fprintf(stderr, "\n[probe] === restart A: sitl_local_shutdown() + sitl_local_init() ===\n");
    sitl_local_shutdown();
    drain_fc_log("shutdown");
    if (sitl_local_init() != 0) {
        fprintf(stderr, "[probe] re-init failed\n");
        return 2;
    }
    step_fc(50);
    drain_fc_log("reinit");

    SOCKET a1 = probe_link("A1 kept link (pre-restart connection)", link, port,
                           "the configurator's stale link to answer");
    SOCKET a2 = probe_link("A2 fresh link", INVALID_SOCKET, port,
                           "a reconnecting configurator to succeed");
    if (a2 != INVALID_SOCKET) {
        link = a2;
    }
    if (a1 == INVALID_SOCKET && a2 == INVALID_SOCKET) {
        fprintf(stderr, "[probe] both A probes failed - reconnect is broken after a host restart\n");
        return 1;
    }

    /* --- restart B: MSP_SET_REBOOT on a live link ------------------------- */
    fprintf(stderr, "\n[probe] === restart B: MSP_SET_REBOOT (Save and Reboot) ===\n");
    {
        const uint8_t zero = 0;
        uint8_t reply[256];
        int replyLen = 0;
        const int r = msp_request(link, MSP_SET_REBOOT, &zero, 1, reply, &replyLen, 3000);
        fprintf(stderr, "[probe] reboot request -> %s\n",
               r == 1 ? "answered" : (r == 0 ? "no answer" : "closed"));
    }
    /* The reboot is applied by the host thread between scheduler passes. */
    step_fc(200);
    drain_fc_log("reboot");

    SOCKET b1 = probe_link("B1 kept link (live session across the reboot)", link, port,
                           "the surviving MSP session to answer");
    SOCKET b2 = probe_link("B2 fresh link", INVALID_SOCKET, port,
                           "a reconnecting configurator to succeed");
    if (b1 != INVALID_SOCKET) {
        closesocket(b1);
    }
    if (b2 != INVALID_SOCKET) {
        closesocket(b2);
    }

    fprintf(stderr, "\n[probe] done\n");
    return (b1 != INVALID_SOCKET || b2 != INVALID_SOCKET) ? 0 : 1;
}
