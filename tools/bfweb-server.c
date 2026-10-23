/**
 * bfweb-server - a self-contained static web server for the Betaflight
 * configurator, with the same behaviour as bfweb-server.mjs but no Node
 * dependency.
 *
 * STATUS: standalone tool, not wired into the game. The packaged game serves
 * the configurator from its own HTTP server (BfSitlHttpServer, which answers
 * connections concurrently and drops stalled peers, so level transitions do not
 * interrupt the page), and the editor keeps using bfweb-server.mjs. Keep this
 * around if you ever want the page served by a separate, Node-free process -
 * e.g. running the configurator next to the standalone betaflight_SITL.exe.
 *
 *   bfweb-server.exe <root-dir> [port] [ws-url]
 *
 *   root-dir : directory holding index.html (the built configurator)
 *   port     : listen port on 127.0.0.1, default 8080, 0 = let the OS choose
 *   ws-url   : WebSocket endpoint injected as the connection preset,
 *              default ws://127.0.0.1:6761
 *
 * What it does, matching the Node server the repository ships:
 *   - serves the directory as-is, Cache-Control: no-cache;
 *   - injects a tiny script into index.html that pre-fills the connection
 *     settings in localStorage (manual mode + portOverride = ws-url), so the
 *     configurator connects to this machine's in-process flight controller
 *     without the user touching the Connect dialog;
 *   - replaces sw.js with an inert stub, because the Workbox service worker
 *     would otherwise serve the un-injected page from its cache;
 *   - binds the loopback interface only.
 *
 * On success the first line on stdout is
 *   BFWEB listening <port> <root>
 * so a host can discover the port it actually got (port 0 = OS choice).
 *
 * Build (WSL, same toolchain as the SITL DLL):
 *   x86_64-w64-mingw32-gcc -O2 -o build-win-local/bfweb-server.exe tools/bfweb-server.c -lws2_32
 */

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REQUEST_MAX 8192
#define PATH_MAX_CHARS 2048

static char gRoot[PATH_MAX_CHARS];
static char gWsUrl[512];

static const char* mimeFor(const char* path)
{
    const char* dot = strrchr(path, '.');
    if (dot == NULL) {
        return "application/octet-stream";
    }
    if (_stricmp(dot, ".html") == 0) return "text/html; charset=utf-8";
    if (_stricmp(dot, ".js") == 0)   return "text/javascript; charset=utf-8";
    if (_stricmp(dot, ".mjs") == 0)  return "text/javascript; charset=utf-8";
    if (_stricmp(dot, ".css") == 0)  return "text/css; charset=utf-8";
    if (_stricmp(dot, ".json") == 0) return "application/json; charset=utf-8";
    if (_stricmp(dot, ".webmanifest") == 0) return "application/manifest+json; charset=utf-8";
    if (_stricmp(dot, ".svg") == 0)  return "image/svg+xml";
    if (_stricmp(dot, ".png") == 0)  return "image/png";
    if (_stricmp(dot, ".ico") == 0)  return "image/x-icon";
    if (_stricmp(dot, ".jpg") == 0)  return "image/jpeg";
    if (_stricmp(dot, ".gif") == 0)  return "image/gif";
    if (_stricmp(dot, ".woff") == 0) return "font/woff";
    if (_stricmp(dot, ".woff2") == 0)return "font/woff2";
    if (_stricmp(dot, ".ttf") == 0)  return "font/ttf";
    if (_stricmp(dot, ".map") == 0)  return "application/json";
    if (_stricmp(dot, ".mcm") == 0)  return "text/plain; charset=utf-8";
    return "application/octet-stream";
}

/* The Workbox build of the configurator precaches index.html, so the injected
 * preset never runs twice. Same stub as the Node server serves. */
static const char* swStub(void)
{
    return "self.addEventListener('install', () => self.skipWaiting());\n"
           "self.addEventListener('activate', (event) => {\n"
           "    event.waitUntil((async () => {\n"
           "        if (self.caches) {\n"
           "            const keys = await self.caches.keys();\n"
           "            await Promise.all(keys.map((key) => self.caches.delete(key)));\n"
           "        }\n"
           "        await self.clients.claim();\n"
           "    })());\n"
           "});\n";
}

static void sendAll(SOCKET sock, const char* data, size_t len)
{
    while (len > 0) {
        const int sent = send(sock, data, (int)len, 0);
        if (sent <= 0) {
            return;
        }
        data += sent;
        len -= (size_t)sent;
    }
}

static void sendResponseHead(SOCKET sock, int status, const char* type, size_t length, int noCache)
{
    char head[512];
    const char* reason = (status == 200) ? "OK" : (status == 404 ? "Not Found" : "Bad Request");
    _snprintf(head, sizeof(head),
              "HTTP/1.1 %d %s\r\n"
              "Content-Type: %s\r\n"
              "Content-Length: %lu\r\n"
              "Connection: close\r\n"
              "%s"
              "\r\n",
              status, reason, type, (unsigned long)length,
              noCache ? "Cache-Control: no-cache\r\n" : "");
    sendAll(sock, head, strlen(head));
}

static void sendBody(SOCKET sock, const void* data, size_t length)
{
    if (length > 0) {
        sendAll(sock, (const char*)data, length);
    }
}

static void sendError(SOCKET sock, int status, const char* message)
{
    sendResponseHead(sock, status, "text/plain; charset=utf-8", strlen(message), 1);
    sendBody(sock, message, strlen(message));
}

static void sendFile(SOCKET sock, const char* fullPath, const char* urlPath)
{
    FILE* f = fopen(fullPath, "rb");
    if (f == NULL) {
        sendError(sock, 404, "Not found");
        return;
    }
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size < 0) {
        fclose(f);
        sendError(sock, 404, "Not found");
        return;
    }

    /* index.html gets the connection preset injected right after <head>. */
    const int isIndexHtml = (strstr(urlPath, "index.html") != NULL);
    if (isIndexHtml) {
        char* buffer = (char*)malloc((size_t)size + 1);
        if (buffer == NULL) {
            fclose(f);
            sendError(sock, 404, "Out of memory");
            return;
        }
        const size_t read = fread(buffer, 1, (size_t)size, f);
        fclose(f);
        buffer[read] = '\0';

        char preset[1200];
        _snprintf(preset, sizeof(preset),
                  "<script>\n"
                  "try {\n"
                  "    localStorage.setItem(\"automaticDevOptions\", JSON.stringify({ automaticDevOptions: false }));\n"
                  "    localStorage.setItem(\"showVirtualMode\", JSON.stringify({ showVirtualMode: false }));\n"
                  "    localStorage.setItem(\"showManualMode\", JSON.stringify({ showManualMode: true }));\n"
                  "    localStorage.setItem(\"expertMode\", JSON.stringify({ expertMode: true }));\n"
                  "    localStorage.setItem(\"portOverride\", JSON.stringify({ portOverride: \"%s\" }));\n"
                  "    localStorage.setItem(\"autoConnect\", JSON.stringify({ autoConnect: true }));\n"
                  "} catch (e) {}\n"
                  "</script>",
                  gWsUrl);

        const char* head = strstr(buffer, "<head>");
        if (head != NULL) {
            const size_t prefix = (size_t)(head - buffer) + strlen("<head>");
            const size_t total = prefix + strlen(preset) + (read - prefix);
            sendResponseHead(sock, 200, "text/html; charset=utf-8", total, 1);
            sendBody(sock, buffer, prefix);
            sendBody(sock, preset, strlen(preset));
            sendBody(sock, buffer + prefix, read - prefix);
        } else {
            sendResponseHead(sock, 200, "text/html; charset=utf-8", read, 1);
            sendBody(sock, buffer, read);
        }
        free(buffer);
        return;
    }

    sendResponseHead(sock, 200, mimeFor(fullPath), (size_t)size, 1);
    char chunk[16384];
    size_t got;
    while ((got = fread(chunk, 1, sizeof(chunk), f)) > 0) {
        sendBody(sock, chunk, got);
    }
    fclose(f);
}

/* Keep the URL inside root: reject absolute paths and any ".." segment. */
static int isSafePath(const char* path)
{
    if (path == NULL || path[0] == '\0') {
        return 0;
    }
    if (strstr(path, "..") != NULL) {
        return 0;
    }
    if (strchr(path, ':') != NULL) {
        return 0;
    }
    return 1;
}

static void handleClient(SOCKET sock)
{
    char request[REQUEST_MAX];
    int total = 0;
    while (total < (int)sizeof(request) - 1) {
        const int n = recv(sock, request + total, (int)sizeof(request) - 1 - total, 0);
        if (n <= 0) {
            return;
        }
        total += n;
        request[total] = '\0';
        if (strstr(request, "\r\n\r\n") != NULL) {
            break;
        }
    }

    char method[16] = {0};
    char url[PATH_MAX_CHARS] = {0};
    if (sscanf(request, "%15s %2047s", method, url) != 2) {
        sendError(sock, 400, "Bad request");
        return;
    }
    if (strcmp(method, "GET") != 0 && strcmp(method, "HEAD") != 0) {
        sendError(sock, 400, "Bad request");
        return;
    }

    char path[PATH_MAX_CHARS];
    _snprintf(path, sizeof(path), "%s", url);
    char* query = strchr(path, '?');
    if (query != NULL) {
        *query = '\0';
    }
    char* hash = strchr(path, '#');
    if (hash != NULL) {
        *hash = '\0';
    }
    if (path[0] == '\0' || strcmp(path, "/") == 0) {
        _snprintf(path, sizeof(path), "/index.html");
    }
    if (!isSafePath(path)) {
        sendError(sock, 400, "Bad request");
        return;
    }

    if (_stricmp(path, "/sw.js") == 0) {
        const char* stub = swStub();
        sendResponseHead(sock, 200, "text/javascript; charset=utf-8", strlen(stub), 1);
        sendBody(sock, stub, strlen(stub));
        return;
    }

    char fullPath[PATH_MAX_CHARS + 8];
    _snprintf(fullPath, sizeof(fullPath), "%s%s", gRoot, path);
    for (char* p = fullPath; *p != '\0'; ++p) {
        if (*p == '/') {
            *p = '\\';
        }
    }

    /* A directory (or a path without extension) falls back to index.html, which
     * is what the configurator's hash routing expects. */
    const DWORD attrs = GetFileAttributesA(fullPath);
    if (attrs != INVALID_FILE_ATTRIBUTES && (attrs & FILE_ATTRIBUTE_DIRECTORY)) {
        strncat(fullPath, "\\index.html", sizeof(fullPath) - strlen(fullPath) - 1);
    }
    sendFile(sock, fullPath, path);
}

static DWORD WINAPI clientThread(LPVOID arg)
{
    const SOCKET sock = (SOCKET)(intptr_t)arg;
    handleClient(sock);
    shutdown(sock, SD_BOTH);
    closesocket(sock);
    return 0;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        printf("usage: %s <root-dir> [port] [ws-url]\n", argv[0]);
        return 2;
    }

    /* Absolute root, no trailing separator. */
    if (_fullpath(gRoot, argv[1], sizeof(gRoot)) == NULL) {
        printf("cannot resolve root dir %s\n", argv[1]);
        return 2;
    }
    const size_t rootLen = strlen(gRoot);
    if (rootLen > 0 && (gRoot[rootLen - 1] == '\\' || gRoot[rootLen - 1] == '/')) {
        gRoot[rootLen - 1] = '\0';
    }

    char indexPath[PATH_MAX_CHARS];
    _snprintf(indexPath, sizeof(indexPath), "%s\\index.html", gRoot);
    if (GetFileAttributesA(indexPath) == INVALID_FILE_ATTRIBUTES) {
        printf("root dir has no index.html: %s\n", gRoot);
        return 2;
    }

    const int port = (argc > 2) ? atoi(argv[2]) : 8080;
    _snprintf(gWsUrl, sizeof(gWsUrl), "%s", (argc > 3) ? argv[3] : "ws://127.0.0.1:6761");

    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("WSAStartup failed\n");
        return 2;
    }

    SOCKET listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener == INVALID_SOCKET) {
        printf("socket failed\n");
        return 2;
    }
    /* SO_EXCLUSIVEADDRUSE, not SO_REUSEADDR: on Windows the latter lets a second
     * process bind the same port and silently steal connections, so a conflict
     * would look like a healthy server. Exclusive ownership makes the second
     * bind fail outright, which is how the host learns to pick another port. */
    BOOL exclusive = TRUE;
    if (setsockopt(listener, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, (const char*)&exclusive, sizeof(exclusive)) == SOCKET_ERROR) {
        BOOL reuse = TRUE;
        setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((u_short)port);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    if (bind(listener, (const struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        /* stdout, not stderr: the host that launches us reads stdout, and this
         * message is how it tells "port taken" from "started fine". */
        printf("bind 127.0.0.1:%d failed (wsa error %d)\n", port, WSAGetLastError());
        return 3;
    }
    if (listen(listener, 16) == SOCKET_ERROR) {
        printf("listen failed (wsa error %d)\n", WSAGetLastError());
        return 3;
    }

    int boundPort = 0;
    int addrLen = sizeof(addr);
    if (getsockname(listener, (struct sockaddr*)&addr, &addrLen) == 0) {
        boundPort = ntohs(addr.sin_port);
    }
    printf("BFWEB listening %d %s\n", boundPort, gRoot);
    fflush(stdout);
    fprintf(stderr, "bfweb-server: http://127.0.0.1:%d/ (root %s, ws %s)\n", boundPort, gRoot, gWsUrl);

    for (;;) {
        SOCKET client = accept(listener, NULL, NULL);
        if (client == INVALID_SOCKET) {
            break;
        }
        HANDLE thread = CreateThread(NULL, 0, clientThread, (void*)(intptr_t)client, 0, NULL);
        if (thread == NULL) {
            closesocket(client);
            continue;
        }
        CloseHandle(thread);   /* detached */
    }

    closesocket(listener);
    WSACleanup();
    return 0;
}
