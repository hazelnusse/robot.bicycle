#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#ifdef __cplusplus
extern "C" {
#endif

void VectorE0(void);   // EXTI Line[15:10] interrupts, for steer index
void Vector108(void);  // TIM5 Global interrupt, for encoder speed measurement

#ifdef __cplusplus
}
#endif

#endif // INTERRUPTS_H
