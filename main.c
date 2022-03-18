/** ***************************************************************************
 * @file    main.c
 * @brief   Trabalho 2 de Sistemas Embarcados
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
#include "pwm.h"
#include "led.h"
#include "gpio.h"
#include "adc.h"
#include "lcd.h"
#include "clock_efm32gg2.h"

#define E_ANDANDO 1 
#define E_MEDINDO 2
#define E_ESQ1 3
#define E_ESQ2 4
#define E_DIR1 5
#define E_DIR2 6
#define E_DIR3 7
#define E_DIR4 8

/*****************************************************************************
 * @brief  SysTick interrupt handler
 *
 * @note   Called every 1/DIVIDER seconds (1 ms)
 */

//{
#define SYSTICKDIVIDER 1000
#define SOFTDIVIDER 1000
#define DELAYVAL 2

static uint32_t ticks = 0;
void SysTick_Handler(void) {
    static int counter = 0;             // must be static
    ticks++;
    
    if( counter != 0 ) {
        counter--;
    } else {
        counter = SOFTDIVIDER-1;
        LED_Toggle(LED1);
    }
}

/*****************************************************************************
 * @brief  Delay function based on SysTick
 *****************************************************************************/
void
Delay(uint32_t v) {
uint32_t lim = ticks+v;       // Missing processing of overflow here
    while ( ticks < lim ) {}
}

int andando(float *ME, float *MD);
void alteraValorLEDs(float ME, float MD);

int main(void) {
    //Configure inputs
    GPIO_Init(GPIOD, BIT(0), 0);
    GPIO_Init(GPIOD, BIT(1), 0);
    GPIO_Init(GPIOD, BIT(2), 0);
    //Configure Outputs
    GPIO_Init(GPIOD, 0, BIT(7));
    GPIO_Init(GPIOC, 0, BIT(0));

    // Set clock source to external crystal: 48 MHz
    (void) SystemCoreClockSet(CLOCK_HFXO,1,1);
    SysTick_Config(SystemCoreClock/SYSTICKDIVIDER);

    //Configure LCD
    LCD_Init();

    LCD_SetAll();
    Delay(DELAYVAL);
    LCD_ClearAll();
    Delay(DELAYVAL);

    // Configure ADC
    ADC_Init(500000);
    ADC_ConfigChannel(ADC_CH0, 0); //ADC_SINGLECTRL_REF_VDD
    ADC_ConfigChannel(ADC_CH1, 0); //ADC_SINGLECTRL_REF_
    ADC_ConfigChannel(ADC_CH2, 0); //ADC_SINGLECTRL_REF

    // Configure LED PWM
    PWM_Init(TIMER0,PWM_LOC4,PWM_PARAMS_ENABLECHANNEL1);
    PWM_Init(TIMER1,PWM_LOC4,PWM_PARAMS_ENABLECHANNEL1);

    // Enable IRQs
    __enable_irq();

    //Máquina de estados
    int estado = E_ANDANDO;

    float ME = 1.0, MD = 0.01;

    while(1){
        switch(estado){
            case E_ANDANDO:
                //Executa instrução andando
                estado = andando(&ME, &MD);
                alteraValorLEDs(ME, MD);
                break;
            case E_MEDINDO:
                //Executa instrução medindo
                ME = 0;
                MD = 0;
                break;
            case E_ESQ1:
                //Executa instrução esquerda 1
                ME = 0;
                MD = 1;
                break;
            case E_ESQ2:
                //Executa instrução esquerda 2
                ME = 0;
                MD = 1;
                break;
            case E_DIR1:
                //Executa instrução direita 1
                ME = 1;
                MD = 0;
                break;
            case E_DIR2:
                //Executa instrução direita 2
                ME = 1;
                MD = 0;
                break;
            case E_DIR3:
                //Executa instrução direita 3
                ME = 1;
                MD = 0;
                break;
            case E_DIR4:
                //Executa instrução direita 4
                ME = 1;
                MD = 0;
                break;
            default:
                //Executa instrução default
                ME = 1;
                MD = 0;
                break;
        } 
    }     
}

int andando(float *ME, float *MD){
    int valueReadEsq = ADC_Read(ADC_CH0);
    int valueReadDir = ADC_Read(ADC_CH1);
    int valueReadFr = ADC_Read(ADC_CH2);
    char buffer[50], buffer2[50];

    itoa(valueReadEsq, buffer, 10);
    strcat(buffer, "-");
    itoa(valueReadDir, buffer2, 10);
    strcat(buffer, buffer2);

    //LCD_WriteString(buffer);

    if(valueReadDir>valueReadEsq){
        *MD = 1.0;
        //Find ME value between 0.5 and 1
        float diference = valueReadEsq / valueReadDir;
        //Convert from 0-1 to 0.5 to 1
        diference = 0.5 + diference/2.0;
        *ME = diference;

    } else {
        *ME = 1.0;
        //Find ME value between 0.5 and 1
        float diference = valueReadDir / valueReadEsq;
        //Convert from 0-1 to 0.5 to 1
        diference = 0.5 + diference/2.0;
        *MD = diference;
    }

    itoa(valueReadFr, buffer, 10);
    LCD_WriteString(buffer);
    

    return E_ANDANDO;
}

void alteraValorLEDs(float ME, float MD){
    //Convert ME and MD from range 0-1 to 0-65535
    ME = ME * 65535;
    MD = MD * 65535;

    //Escrever ME no PD7
    PWM_Write(TIMER1,1,ME);    
    //Escrever MD no PC0
    PWM_Write(TIMER0,1,MD);
}