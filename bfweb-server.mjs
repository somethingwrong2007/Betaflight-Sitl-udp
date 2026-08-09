#!/usr/bin/env node
/**
 * Local static server for the Betaflight web configurator (bf-configurator).
 *
 * Serves the built dist/ directory as-is, but injects a tiny script into
 * index.html that pre-fills the connection settings in localStorage:
 *   - portOverride : ws://127.0.0.1:6761
 *   - showManualMode : true  (manual connection appears in the Connect menu)
 *   - expertMode : true      (manual/virtual items are gated behind expert mode)
 *
 * No files from bf-configurator are modified; the injection happens in memory
 * on every request.
 */

import http from "node:http";
import { readFile, stat } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const DIST = path.join(__dirname, "bf-configurator", "src", "dist");
const PORT = Number(process.env.BFWEB_PORT || 8080);
const HOST = "127.0.0.1";

const MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".webmanifest": "application/manifest+json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".ico": "image/x-icon",
    ".jpg": "image/jpeg",
    ".gif": "image/gif",
    ".woff": "font/woff",
    ".woff2": "font/woff2",
    ".ttf": "font/ttf",
    ".map": "application/json",
};

const PRESET_SCRIPT = `<script>
try {
    localStorage.setItem("portOverride", JSON.stringify({ portOverride: "ws://127.0.0.1:6761" }));
    localStorage.setItem("showManualMode", JSON.stringify({ showManualMode: true }));
    localStorage.setItem("expertMode", JSON.stringify({ expertMode: true }));
} catch (e) { /* localStorage unavailable */ }
</script>`;

async function sendFile(res, filePath, type) {
    try {
        const data = await readFile(filePath);
        res.writeHead(200, {
            "Content-Type": type,
            "Cache-Control": "no-cache",
        });
        res.end(data);
    } catch {
        res.writeHead(404, { "Content-Type": "text/plain; charset=utf-8" });
        res.end("Not found");
    }
}

const server = http.createServer(async (req, res) => {
    const url = new URL(req.url, `http://${HOST}:${PORT}`);
    let pathname = decodeURIComponent(url.pathname);
    if (pathname === "/") {
        pathname = "/index.html";
    }

    const filePath = path.join(DIST, path.normalize(pathname).replace(/^([/\\])+/, ""));
    const ext = path.extname(filePath).toLowerCase();

    // No extension: treat as the SPA entry (hash routing means no server-side
    // fallback is needed, but keep this for convenience).
    if (!ext || ext === ".html") {
        try {
            const info = await stat(filePath);
            if (info.isFile()) {
                const html = await readFile(filePath, "utf8");
                const injected = html.replace(
                    "<head>",
                    "<head>\n" + PRESET_SCRIPT,
                );
                res.writeHead(200, { "Content-Type": "text/html; charset=utf-8", "Cache-Control": "no-cache" });
                res.end(injected);
                return;
            }
        } catch {
            // fall through to index.html below
        }
        const indexPath = path.join(DIST, "index.html");
        try {
            const html = await readFile(indexPath, "utf8");
            const injected = html.replace(
                "<head>",
                "<head>\n" + PRESET_SCRIPT,
            );
            res.writeHead(200, { "Content-Type": "text/html; charset=utf-8", "Cache-Control": "no-cache" });
            res.end(injected);
            return;
        } catch {
            res.writeHead(500, { "Content-Type": "text/plain; charset=utf-8" });
            res.end("index.html not found in " + DIST);
            return;
        }
    }

    await sendFile(res, filePath, MIME[ext] || "application/octet-stream");
});

server.listen(PORT, HOST, () => {
    console.log(`Betaflight web server: http://${HOST}:${PORT}/ (dist: ${DIST})`);
});
