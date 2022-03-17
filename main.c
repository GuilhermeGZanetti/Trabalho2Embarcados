/** ***************************************************************************
 * @file    main.c
 * @brief   Simple LED Blink Demo for EFM32GG_STK3700
 * @version 1.0
******************************************************************************/

#include <stdint.h>
/*
 * Including this file, it is possible to define which processor using command line
 * E.g. -DEFM32GG995F1024
 * The alternative is to include the processor specific file directly
 * #include "efm32gg995f1024.h"
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "em_device.h"
#include "led.h"
#include "pwm.h"
#include "gpio.h"
#include "adc.h"

/*****************************************************************************
 * @brief  Main function
 *
 * @note   Using default clock configuration
 *         HFCLK = HFRCO
 *         HFCORECLK = HFCLK
 *         HFPERCLK  = HFCLK
 */

void Delay(uint32_t delay) {
    volatile uint32_t counter;
    int i;

    for(i=0;i<delay;i++) {
        counter = 10000;
        while( counter ) counter--;
    }
}

int main(void) {
    /* Configure LEDs */
    LED_Init(LED1);
    //Configure inputs
    GPIO_Init(GPIOD, BIT(0), 0);

    // Configure ADC
    ADC_Init(500000);
    ADC_ConfigChannel(ADC_CH0, 0); //ADC_SINGLECTRL_REF_VDD

    // Configure LED PWM
    PWM_Init(TIMER3,PWM_LOC1,PWM_PARAMS_ENABLECHANNEL2);

    // Enable IRQs
    //__enable_irq();

    while(1){
        int value = 0;                
        
        value = ADC_Read(ADC_CH0);

        PWM_Write(TIMER3,2,value*26);    

        Delay(1);   
    }   
}