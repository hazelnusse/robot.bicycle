#include <new>
#include <stdio.h>
#include "ch.h"
#include "stubs.h"

void _exit(int status __attribute__((unused)))
{
  chSysHalt();
  while(1){}
}

pid_t _getpid(void)
{
  return 1;
}

void _kill(pid_t id __attribute__((unused)))
{

}

void * operator new(size_t size) 
{ 
  return chHeapAlloc(NULL, size);
} 

void * operator new(size_t size, const std::nothrow_t &) 
{ 
  return chHeapAlloc(NULL, size);
} 

void * operator new[](size_t size) 
{ 
  return chHeapAlloc(NULL, size);
} 

void * operator new[](size_t size, const std::nothrow_t &) 
{ 
  return chHeapAlloc(NULL, size);
} 

void operator delete(void * ptr) 
{ 
  chHeapFree(ptr); 
} 

void operator delete(void * ptr, const std::nothrow_t &) 
{ 
  chHeapFree(ptr); 
} 

void operator delete[](void * ptr) 
{ 
  chHeapFree(ptr); 
} 

void operator delete[](void * ptr, const std::nothrow_t &) 
{ 
  chHeapFree(ptr); 
} 

/* guard variables */
int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);};
void __cxa_guard_release (__guard *g) {*(char *)g = 1;};
void __cxa_guard_abort (__guard * g __attribute__((unused))) {};
