#ifndef BLINKTHREAD_H
#define BLINKTHREAD_H
#include "ch.h"

void FileSystemBlinkThread(void * arg) __attribute__((noreturn));
void SampleAndControlBlinkThread(void *arg) __attribute__((noreturn));

#endif // BLINKTHREAD_H
