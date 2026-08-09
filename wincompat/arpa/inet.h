#ifndef BF_SITL_WINCOMPAT_ARPA_INET_H
#define BF_SITL_WINCOMPAT_ARPA_INET_H

#include <winsock2.h>
#include <ws2tcpip.h>

/* Windows headers define BAUD_* macros that collide with Betaflight's
 * baudRate_e enum in io/serial.h. Undo them after winsock2.h has been loaded. */
#ifdef BAUD_50
#undef BAUD_50
#endif
#ifdef BAUD_75
#undef BAUD_75
#endif
#ifdef BAUD_110
#undef BAUD_110
#endif
#ifdef BAUD_134
#undef BAUD_134
#endif
#ifdef BAUD_150
#undef BAUD_150
#endif
#ifdef BAUD_300
#undef BAUD_300
#endif
#ifdef BAUD_600
#undef BAUD_600
#endif
#ifdef BAUD_1200
#undef BAUD_1200
#endif
#ifdef BAUD_1800
#undef BAUD_1800
#endif
#ifdef BAUD_2400
#undef BAUD_2400
#endif
#ifdef BAUD_4800
#undef BAUD_4800
#endif
#ifdef BAUD_7200
#undef BAUD_7200
#endif
#ifdef BAUD_9600
#undef BAUD_9600
#endif
#ifdef BAUD_14400
#undef BAUD_14400
#endif
#ifdef BAUD_19200
#undef BAUD_19200
#endif
#ifdef BAUD_38400
#undef BAUD_38400
#endif
#ifdef BAUD_56000
#undef BAUD_56000
#endif
#ifdef BAUD_57600
#undef BAUD_57600
#endif
#ifdef BAUD_115200
#undef BAUD_115200
#endif
#ifdef BAUD_128000
#undef BAUD_128000
#endif
#ifdef BAUD_256000
#undef BAUD_256000
#endif

#endif
