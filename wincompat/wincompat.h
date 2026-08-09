#ifndef BF_SITL_WINCOMPAT_H
#define BF_SITL_WINCOMPAT_H

#include <string.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

struct tm *gmtime_r(const time_t *timep, struct tm *result);
char *strsep(char **stringp, const char *delim);
char *strcasestr(const char *haystack, const char *needle);

#ifdef __cplusplus
}
#endif

#endif
