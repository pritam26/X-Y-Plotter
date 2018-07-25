/*
 * LABGPIOINTERRUPTS_H.hpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Gaurav Yadav
 */

#ifndef LABGPIOINTERRUPTS_H_HPP_
#define LABGPIOINTERRUPTS_H_HPP_

#include "singleton_template.hpp"
#include <stdio.h>          // printf()
#include "FreeRTOS.h"
#include "task.h"           // vRunTimeStatIsrEntry() and vRunTimeStatIsrExit()
#include "LPC17xx.h"        // IRQn_Type
#include "uart0_min.h"      // Uart0 initialization
#include "printf_lib.h"     // u0_dbg_printf()
#include "lpc_sys.h"        // sys_reboot()
#include "fault_registers.h"
#include "string.h"


typedef enum {
  rising_edge,
  falling_edge,
  both_edges,
} InterruptCondition_E;

class LabGPIOInterrupts
{
private:

  static  void (*loookup_matrix[2][32])(void);  // why static needed?

  //  static LabGPIOInterrupts* m_pInstance;;

public:
    /**
     * LabGPIOInterrupts should be a singleton, meaning, only one instance can exist at a time.
     * Look up how to implement this.
     */
   LabGPIOInterrupts();
   //static LabGPIOInterrupts* getInstance();
    /**
     * 1) Should setup register "externalIRQHandler" as the EINT3 ISR.
     * 2) Should configure NVIC to notice EINT3 IRQs.
     */
    void init();
    /**
     * This handler should place a function pointer within the lookup table for the externalIRQHandler to find.
     *
     * @param[in] port         specify the GPIO port
     * @param[in] pin          specify the GPIO pin to assign an ISR to
     * @param[in] pin_isr      function to run when the interrupt event occurs
     * @param[in] condition    condition for the interrupt to occur on. RISING, FALLING or BOTH edges.
     * @return should return true if valid ports, pins, isrs were supplied and pin isr insertion was sucessful
     */
    bool attachInterruptHandler(uint8_t port, uint32_t pin, void (*pin_isr)(void), InterruptCondition_E condition);
    /**
     * After the init function has run, this will be executed whenever a proper
     * EINT3 external GPIO interrupt occurs. This function figure out which pin
     * has been interrupted and run the ccorrespondingISR for it using the lookup table.
     *
     * VERY IMPORTANT! Be sure to clear the interrupt flag that caused this
     * interrupt, or this function will be called again and again and again, ad infinitum.
     *
     * Also, NOTE that your code needs to be able to handle two GPIO interrupts occurring
     * at the same time.
     */
    static void handle_interrupt(void);
    ~LabGPIOInterrupts();
};


#endif /* LABGPIOINTERRUPTS_H_HPP_ */
