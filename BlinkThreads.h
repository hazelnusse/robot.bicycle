#ifndef BLINKTHREAD_H
#define BLINKTHREAD_H
#include "ch.h"

msg_t FileSystemBlinkThread(void * arg);
msg_t SampleAndControlBlinkThread(void *arg);

#endif // BLINKTHREAD_H
