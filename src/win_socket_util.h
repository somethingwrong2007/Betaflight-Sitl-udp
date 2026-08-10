#ifndef SRC_WIN_SOCKET_UTIL_H
#define SRC_WIN_SOCKET_UTIL_H

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>

// Sockets created by Winsock are inheritable by default. When a firmware
// reboot spawns a child process for the auto-restart, the child would
// otherwise inherit the parent's listening/UDP sockets, leaving duplicate
// listeners on the same port that no thread accepts from. Clear the inherit
// flag on every socket so only explicitly passed std handles are inherited.
static inline void socketNoInherit(SOCKET s)
{
    if (s != INVALID_SOCKET) {
        SetHandleInformation((HANDLE)s, HANDLE_FLAG_INHERIT, 0);
    }
}

#endif

#endif
