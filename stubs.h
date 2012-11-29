#ifndef STUBS_H
#define STUBS_H

#include <cstdlib>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

void _exit(int status);
pid_t _getpid(void);
void _kill(pid_t id);

void * __dso_handle = 0;
__extension__ typedef int __guard __attribute__((mode (__DI__)));

int __cxa_guard_acquire(__guard *);
void __cxa_guard_release (__guard *);
void __cxa_guard_abort (__guard *);

#ifdef __cplusplus
}
#endif

#endif
