#ifndef STUBS_H
#define STUBS_H

#include <stdlib.h>

///* The ABI requires a 64-bit type.  */
__extension__ typedef int __guard __attribute__((mode (__DI__)));


extern "C" {
int __cxa_guard_acquire(__guard *);
void __cxa_guard_release (__guard *);
void __cxa_guard_abort (__guard *);
void *__dso_handle = NULL;

void _exit(int status);
pid_t _getpid(void);
void _kill(pid_t id);

}

void * operator new(size_t size); 
void operator delete(void * ptr);

#endif
