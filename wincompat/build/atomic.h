/*
 * x86/SITL shadow of Betaflight's ARM-only build/atomic.h.
 *
 * The official header uses Cortex-M BASEPRI intrinsics (__ASM), which do not
 * compile on x86. LOCAL mode enables USE_DSHOT so the simulated motor RPM can
 * flow through dshot.c; this header provides the same ATOMIC_* surface as
 * no-ops (critical sections are meaningless in a simulator).
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

// BASEPRI manipulation: no-ops on x86.
static inline void __set_BASEPRI_nb(uint32_t basePri)
{
    (void)basePri;
}

static inline void __set_BASEPRI_MAX_nb(uint32_t basePri)
{
    (void)basePri;
}

static inline uint8_t __get_BASEPRI(void)
{
    return 0;
}

static inline void __basepriRestoreMem(uint8_t *val)
{
    (void)val;
}

static inline uint8_t __basepriSetMemRetVal(uint8_t prio)
{
    (void)prio;
    return 1;
}

static inline void __basepriRestore(uint8_t *val)
{
    (void)val;
}

static inline uint8_t __basepriSetRetVal(uint8_t prio)
{
    (void)prio;
    return 1;
}

#define ATOMIC_BLOCK(prio) for ( uint8_t __basepri_save __attribute__ ((__cleanup__ (__basepriRestoreMem), __unused__)) = __get_BASEPRI(), \
                                     __ToDo = __basepriSetMemRetVal(prio); __ToDo ; __ToDo = 0 )

#define ATOMIC_BLOCK_NB(prio) for ( uint8_t __basepri_save __attribute__ ((__cleanup__ (__basepriRestore), __unused__)) = __get_BASEPRI(), \
                                    __ToDo = __basepriSetRetVal(prio); __ToDo ; __ToDo = 0 )

#ifndef __UNIQL
# define __UNIQL_CONCAT2(x,y) x ## y
# define __UNIQL_CONCAT(x,y) __UNIQL_CONCAT2(x,y)
# define __UNIQL(x) __UNIQL_CONCAT(x,__LINE__)
#endif

#define ATOMIC_BARRIER_ENTER(dataPtr, refStr)                              \
    __asm__ volatile ("" : "+m" (*(dataPtr)))

#define ATOMIC_BARRIER_LEAVE(dataPtr, refStr)                              \
    __asm__ volatile ("" : "m" (*(dataPtr)))

#define ATOMIC_BARRIER(data)                                            \
    __extension__ void  __UNIQL(__barrierEnd)(typeof(data) **__d) {     \
         ATOMIC_BARRIER_LEAVE(*__d, #data);                             \
    }                                                                   \
    typeof(data) __attribute__((__cleanup__(__UNIQL(__barrierEnd)))) *__UNIQL(__barrier) = &data; \
    ATOMIC_BARRIER_ENTER(__UNIQL(__barrier), #data);                    \
    do {} while(0)                                                      \
/**/

#define ATOMIC_OR(ptr, val) __sync_fetch_and_or(ptr, val)
#define ATOMIC_AND(ptr, val) __sync_fetch_and_and(ptr, val)
