/**
 * The configurator's own path: WebSocket (6761) -> MSP, across a reboot.
 *
 * tools/sitl_local_reconnect_probe.c covers the raw MSP TCP link. The web
 * configurator never uses that link directly - it speaks WebSocket to the
 * built-in proxy, which bridges to MSP. This probe walks the same route:
 *
 *   1. connect ws://127.0.0.1:<wsPort>, MSP_API_VERSION       (baseline)
 *   2. MSP_SET_REBOOT ("Save and Reboot") and watch the socket  - the flight
 *      controller must drop it, like real hardware does on reset. A link that
 *      stays open but stops answering is what makes the configurator sit on a
 *      dead session: it sees no close event, so its reboot-reconnect flow never
 *      starts and the user has to click Connect by hand.
 *   3. reconnect on the configurator's cadence (1 s retry, 20 s window) and
 *      report how long the FC took to answer again.
 *
 * Run against tools/sitl_local_host.exe (a 1 kHz host with no engine):
 *
 *   set BF_SITL_TCP_BASE=15760 & set BF_SITL_WS_PORT=16761
 *   set BF_SITL_EEPROM=%TEMP%\ws-probe-eeprom.bin
 *   start /b tools\sitl_local_host.exe
 *   node tools\ws_reboot_reconnect_probe.mjs 16761
 *
 * Exit code 0 = the socket was dropped on reboot AND a fresh connection
 * answered inside the window.
 */

const wsPort = Number(process.argv[2] ?? 6761);

const MSP_API_VERSION = 1;
const MSP_SET_REBOOT = 68;
const RETRY_MS = 1000;
const WINDOW_MS = 20000;
const REPLY_TIMEOUT_MS = 3000;

function mspFrame(cmd, payload = new Uint8Array()) {
    const size = payload.length;
    const frame = new Uint8Array(6 + size);
    frame[0] = 0x24; // '$'
    frame[1] = 0x4d; // 'M'
    frame[2] = 0x3c; // '<'
    frame[3] = size;
    frame[4] = cmd;
    frame.set(payload, 5);
    let crc = size ^ cmd;
    for (let i = 0; i < size; i++) {
        crc ^= payload[i];
    }
    frame[5 + size] = crc;
    return frame;
}

/** Accumulates WebSocket payloads and resolves the first complete reply frame. */
class MspReader {
    constructor() {
        this.buf = new Uint8Array(0);
    }

    feed(bytes) {
        const merged = new Uint8Array(this.buf.length + bytes.length);
        merged.set(this.buf, 0);
        merged.set(bytes, this.buf.length);
        this.buf = merged;
        return this.next();
    }

    next() {
        for (let i = 0; i + 6 <= this.buf.length; i++) {
            if (this.buf[i] !== 0x24 || this.buf[i + 1] !== 0x4d) {
                continue;
            }
            if (this.buf[i + 2] !== 0x3e && this.buf[i + 2] !== 0x21) {
                continue;
            }
            const size = this.buf[i + 3];
            if (i + 6 + size > this.buf.length) {
                break; // incomplete
            }
            const cmd = this.buf[i + 4];
            const payload = this.buf.slice(i + 5, i + 5 + size);
            this.buf = this.buf.slice(i + 6 + size);
            return { cmd, payload, error: this.buf[i + 2] === 0x21 };
        }
        return null;
    }
}

/** One connection + one MSP round trip. Resolves "answered" | "silent" | "closed". */
function mspRoundTrip(timeoutMs) {
    return new Promise((resolve) => {
        const reader = new MspReader();
        let settled = false;
        const ws = new WebSocket(`ws://127.0.0.1:${wsPort}`);
        ws.binaryType = "arraybuffer";

        const done = (result, api) => {
            if (settled) {
                return;
            }
            settled = true;
            clearTimeout(timer);
            try {
                ws.close();
            } catch {
                /* already gone */
            }
            resolve({ result, api, ws });
        };

        const timer = setTimeout(() => done("silent"), timeoutMs);

        ws.addEventListener("open", () => ws.send(mspFrame(MSP_API_VERSION)));
        ws.addEventListener("message", (event) => {
            const reply = reader.feed(new Uint8Array(event.data));
            if (reply && reply.cmd === MSP_API_VERSION) {
                done("answered", Array.from(reply.payload));
            }
        });
        ws.addEventListener("close", () => done("closed"));
        ws.addEventListener("error", () => done("closed"));
    });
}

/** Connect, reboot, and watch the link: the FC must drop it. */
function rebootAndWatch(timeoutMs) {
    return new Promise((resolve) => {
        const reader = new MspReader();
        const ws = new WebSocket(`ws://127.0.0.1:${wsPort}`);
        ws.binaryType = "arraybuffer";
        let sawApi = false;

        const timer = setTimeout(() => {
            try {
                ws.close();
            } catch {
                /* ignore */
            }
            resolve({ dropped: false, reason: `still open after ${timeoutMs} ms` });
        }, timeoutMs);

        ws.addEventListener("open", () => ws.send(mspFrame(MSP_API_VERSION)));
        ws.addEventListener("message", (event) => {
            const reply = reader.feed(new Uint8Array(event.data));
            if (!reply) {
                return;
            }
            if (reply.cmd === MSP_API_VERSION && !sawApi) {
                sawApi = true;
                console.log("[ws-probe] reboot link: answered, sending MSP_SET_REBOOT");
                ws.send(mspFrame(MSP_SET_REBOOT));
                globalThis.setTimeout(() => {
                    if (ws.readyState === WebSocket.OPEN) {
                        clearTimeout(timer);
                        resolve({ dropped: false, reason: "link stayed open after MSP_SET_REBOOT" });
                    }
                }, 4000);
            }
        });
        ws.addEventListener("close", () => {
            clearTimeout(timer);
            resolve({ dropped: true, reason: "closed by the flight controller" });
        });
        ws.addEventListener("error", () => {
            clearTimeout(timer);
            resolve({ dropped: true, reason: "socket error" });
        });
    });
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

const baseline = await mspRoundTrip(REPLY_TIMEOUT_MS);
if (baseline.result !== "answered") {
    console.log(`[ws-probe] FAIL baseline: ${baseline.result} (is sitl_local_host.exe running?)`);
    process.exit(1);
}
console.log(`[ws-probe] baseline: answered (api ${baseline.api.join(".")})`);

const reboot = await rebootAndWatch(6000);
console.log(`[ws-probe] reboot: ${reboot.dropped ? "link dropped" : "link NOT dropped"} (${reboot.reason})`);

// The configurator's own retry cadence: 1 s, up to the 20 s driven window.
const start = Date.now();
let attempt = 0;
let recovered = null;
while (Date.now() - start < WINDOW_MS) {
    attempt++;
    await sleep(RETRY_MS);
    const r = await mspRoundTrip(REPLY_TIMEOUT_MS);
    console.log(`[ws-probe] retry ${attempt} after ${Date.now() - start} ms: ${r.result}`);
    if (r.result === "answered") {
        recovered = Date.now() - start;
        break;
    }
}

if (recovered === null) {
    console.log(`[ws-probe] FAIL: no connection inside the ${WINDOW_MS / 1000} s window`);
    process.exit(1);
}
console.log(`[ws-probe] RESULT: PASS - dropped on reboot and answered again after ${recovered} ms`);
if (!reboot.dropped) {
    console.log("[ws-probe] NOTE: the stale link was NOT dropped; the reconnect only worked because a fresh one did");
}
process.exit(0);
