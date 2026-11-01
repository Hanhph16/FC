#include "globals.h"

///////////////////////////////////////////////////////////////////////////////////////////
//Cau hinh timer:
//  TIM2 CH1 (PA0)           : input capture doc xung PPM tu receiver RC.
//  TIM4 CH1..CH4 (PB6..PB9) : tao xung PWM 1000-2000us cho 4 ESC.
//(Da bo TIM3 / PB0 - truoc day dung cho servo tha hang RDC.)
///////////////////////////////////////////////////////////////////////////////////////////
void timer_setup(void) {
  // ---------- Receiver: input capture ----------
  Timer2.attachCompare1Interrupt(handler_channel_1);
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
  TIMER2_BASE->CCMR2 = 0;
  TIMER2_BASE->CCER = TIMER_CCER_CC1E;
  TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;                  //Bat canh len.
  TIMER2_BASE->PSC = 71;                                  //72MHz / 72 = 1MHz -> 1 tick = 1us.
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;

  // ---------- ESC: PWM ----------
  TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  TIMER4_BASE->CR2 = 0;
  TIMER4_BASE->SMCR = 0;
  TIMER4_BASE->DIER = 0;
  TIMER4_BASE->EGR = 0;
  TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE | (0b110 << 12) | TIMER_CCMR1_OC2PE;
  TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE | (0b110 << 12) | TIMER_CCMR2_OC4PE;
  TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER4_BASE->PSC = 71;
  TIMER4_BASE->ARR = 5000;                                //Chu ky 5000us.
  TIMER4_BASE->DCR = 0;
  TIMER4_BASE->CCR1 = 1000;
  TIMER4_BASE->CCR2 = 1000;
  TIMER4_BASE->CCR3 = 1000;
  TIMER4_BASE->CCR4 = 1000;
  pinMode(PB6, PWM);
  pinMode(PB7, PWM);
  pinMode(PB8, PWM);
  pinMode(PB9, PWM);
}
