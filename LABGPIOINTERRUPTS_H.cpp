/*
 * LABGPIOINTERRUPTS_H.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Gaurav Yadav
 */

#include "LABGPIOINTERRUPTS_H.hpp"

void (*LabGPIOInterrupts::loookup_matrix[2][32])(void);

LabGPIOInterrupts::LabGPIOInterrupts(){}

LabGPIOInterrupts::~LabGPIOInterrupts(){}

void LabGPIOInterrupts::init(){
        NVIC_EnableIRQ(EINT3_IRQn);
}

void LabGPIOInterrupts::handle_interrupt(void)
{
    u0_dbg_printf("in handler\n");
    uint8_t pin;
    volatile uint8_t i;
    /* Read all the ports' rising and falling isr status */
        uint32_t p0_rising  = LPC_GPIOINT->IO0IntStatR;
        uint32_t p0_falling = LPC_GPIOINT->IO0IntStatF;
        uint32_t p2_rising  = LPC_GPIOINT->IO2IntStatR;
        uint32_t p2_falling = LPC_GPIOINT->IO2IntStatF;

        for(i=0;i<=31;i++)
        {
            if((LPC_GPIOINT->IO0IntStatR & (1<<i)) ||  (LPC_GPIOINT->IO0IntStatF & (1<<i)) )
                pin=i;
            if ((LPC_GPIOINT->IO2IntStatR & (1<<i)) ||  (LPC_GPIOINT->IO2IntStatF & (1<<i)) )
                pin=i;
        }
        if (p0_rising || p0_falling) {
            LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
            uart0_puts("clearing 0");
            loookup_matrix[0][pin]();
        }
        if (p2_rising || p2_falling) {
            LPC_GPIOINT->IO2IntClr = 0xFFFFFFFF;
            uart0_puts("clearing 2");
            loookup_matrix[2][pin]();
        }
        u0_dbg_printf("exit handler\n");
}



bool LabGPIOInterrupts::attachInterruptHandler(uint8_t port, uint32_t pin, void (*pin_isr)(void), InterruptCondition_E condition)
{
    uart0_puts("In attachIH");
    loookup_matrix[port][pin] = pin_isr;

    if(port==0)
    {
        if(condition==falling_edge)
            LPC_GPIOINT->IO0IntEnF |= (1 << pin);
        else if(condition==rising_edge)
            LPC_GPIOINT->IO0IntEnR |= (1 << pin);
        else{
            LPC_GPIOINT->IO0IntEnR |= (1 << pin);
            LPC_GPIOINT->IO0IntEnF |= (1 << pin);
            }
    }
    else if(port==2)
    {
          if(condition==falling_edge)
              LPC_GPIOINT->IO2IntEnF |= (1 << pin);
          else if(condition==rising_edge)
              LPC_GPIOINT->IO2IntEnR |= (1 << pin);
          else{
              LPC_GPIOINT->IO2IntEnR |= (1 << pin);
              LPC_GPIOINT->IO2IntEnF |= (1 << pin);
              }
      }
    uart0_puts("Exit attachIH");
    return true;
}


