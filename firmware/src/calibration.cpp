#include <array>

#include "hal.h"
#include "chprintf.h"
#include "shell.h"

#include "constants.h"
#include "encoder.h"
#include "vector_table.h"

#include "calibration.h"

namespace calibration {

// Integer array to store the value of the fork encoder when the index line
// is triggered.
std::array<int16_t, 16> encoder_counts;

// Volatile shared integer to count the number of times the interrupt is
// triggered.
volatile uint8_t number_of_edges_detected;

// Initially has the value from contants.h, but can be changed by the
// fork_encoder_calibration function if recalibration is needed without
// changing the hard coded value (obtained from a calibration experiment) that
// is in constants.h.
int32_t fork_encoder_index_offset = constants::fork_encoder_index_offset;

// An interface class to the encoder on the fork motor.
hardware::Encoder fork_encoder(STM32_TIM3,
                               constants::fork_counts_per_revolution);

// CalibrationISR is called on a rising/falling edge of the steer index
// Need to save the steer encoder count and determine the direction, which can
// be obtained from STM32_TIM3->CNT and STM32_TIM3->CR1[4]  (DIR bit)
CH_IRQ_HANDLER(fork_encoder_calibration_ISR)
{
  EXTI->PR = (1 << 11);   // clear the pending bit.
  if (number_of_edges_detected < encoder_counts.size()) {
    encoder_counts[number_of_edges_detected] = fork_encoder.get_count();// STM32_TIM3->CNT;
    ++number_of_edges_detected;
  } else {
    EXTI->IMR = 0;
  }
}

CH_IRQ_HANDLER(fork_encoder_home_ISR)
{
  fork_encoder.set_count(fork_encoder_index_offset);
  EXTI->PR = (1 << 11);   // clear the pending bit.
  number_of_edges_detected = 0;
}

void fork_encoder_calibration(BaseSequentialStream * chp, int, char **)
{
  float mean[2];
  for (int j = 0; j < 2; j++) {
    char response[4];
    chprintf(chp, "Is the fork locked? [y/n]\r\n");
    if (shellGetLine(chp, response, sizeof(response))) return;

    if ((response[0] == 'n') | (response[0] == 'N')) return;

    fork_encoder.set_count(0);
    number_of_edges_detected = 0;               // zero out number of times interrupt has triggered
    for (auto &count: encoder_counts)
      count = 0;

    chprintf(chp, "Unlock the fork and move it back and forth %u times\r\n",
             encoder_counts.size()/4);

    VectorTable v;
    irq_vector_t vec40 = v.get_ISR(EXTI15_10_IRQn);
    v.set_ISR(EXTI15_10_IRQn, fork_encoder_calibration_ISR);
    uint16_t tmp = SYSCFG->EXTICR[2];
    SYSCFG->EXTICR[2] = SYSCFG_EXTICR3_EXTI11_PF;
    nvicEnableVector(EXTI15_10_IRQn, CORTEX_PRIORITY_MASK(7));
    EXTI->IMR = (1 << 11);
    EXTI->RTSR = (1 << 11);
    EXTI->FTSR = (1 << 11);

    // sit and spin until the fork has moved back and forth enough times to
    // fill up the encoder_counts array
    while (number_of_edges_detected < encoder_counts.size()) { }

    EXTI->IMR = 0;
    EXTI->RTSR = 0;
    EXTI->FTSR = 0;
    SYSCFG->EXTICR[2] = tmp;
    nvicDisableVector(EXTI15_10_IRQn);
    v.set_ISR(EXTI15_10_IRQn, vec40);

    float sum = 0.0f;
    for (const auto & count : encoder_counts) {
      chprintf(chp, "%d\r\n", count);
      sum += count;
    }

    mean[j] = sum / encoder_counts.size();
    chprintf(chp, "Offset mean: %f\r\n", mean[j]);

    if (j == 0) {
      chprintf(chp, "Reverse the fork fixture and lock the fork straight.\r\n");
    }
  }
  float mean_both_runs = (mean[0] + mean[1])/2.0f;
  chprintf(chp, "Mean: %f\r\n", mean_both_runs);
  fork_encoder_index_offset = round(mean_both_runs);
  chprintf(chp, "Steer offset set to (as integer): %d\r\n", fork_encoder_index_offset);
}

void fork_encoder_home(BaseSequentialStream * chp, int, char **)
{
  chprintf(chp, "Move fork past index position.\r\n");
  VectorTable v;
  irq_vector_t vec40 = v.get_ISR(EXTI15_10_IRQn);
  v.set_ISR(EXTI15_10_IRQn, fork_encoder_home_ISR);
  uint16_t tmp = SYSCFG->EXTICR[2];
  SYSCFG->EXTICR[2] = SYSCFG_EXTICR3_EXTI11_PF;
  nvicEnableVector(EXTI15_10_IRQn, CORTEX_PRIORITY_MASK(7));
  EXTI->IMR = (1 << 11);
  EXTI->RTSR = (1 << 11);
  EXTI->FTSR = (1 << 11);

  number_of_edges_detected = 1;
  while (number_of_edges_detected) { } // sit and spin until fork_encoder_home_ISR sets number_of_edges_detected to zero

  EXTI->IMR = 0;
  EXTI->RTSR = 0;
  EXTI->FTSR = 0;
  SYSCFG->EXTICR[2] = tmp;
  nvicDisableVector(EXTI15_10_IRQn);
  v.set_ISR(EXTI15_10_IRQn, vec40);
  
  chprintf(chp, "Fork has been successfully homed.\r\n");
}

}
