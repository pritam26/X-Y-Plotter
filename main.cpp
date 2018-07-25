/*
 * main_pritam_lappy.cpp
 *
 *  Created on: May 18, 2018
 *      Author: Gaurav Yadav
 */

/*
 * main_plotter.cpp
 *
 *  Created on: May 12, 2018
 *      Author: Gaurav Yadav
 */

#include "FreeRTOS.h"
#include "task.h"
#include "LPC17xx.h"
#include "stdint.h"
#include <stdio.h>
#include "queue.h"
#include "uart0_min.h"
#include "semphr.h"
#include<iostream>
#include <strings.h>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <iomanip>
#include <utilities.h>
using namespace std;
#include "gpio.hpp"
#include "GcodeParser.hpp"
#include "Motor.hpp"
#include "math.h"
#include "event_groups.h"
#include "storage.hpp"
#include "LABGPIOINTERRUPTS_H/LABGPIOINTERRUPTS_H.hpp"

SemaphoreHandle_t switch_read;  // create queue handle

LabGPIOInterrupts lab_interrupt;  // create global instance


QueueHandle_t Global_Queue_Handle=0;
EventGroupHandle_t sw_watchdog;
const uint32_t prod_task_id =(1<<0);
const uint32_t cons_task_id=(1<<1);
const uint32_t task_all_bits = ( prod_task_id | cons_task_id );
//char gcode[275]={0}; for A
char gcode[2500]={0};
GPIO m1_dir(P1_28); // motor1 direction pin
GPIO m2_dir(P1_23); //motor 2 direction pin
//LabGPIOInterrupts lab_interrupt;

TaskHandle_t xHandle; // handle for parse gcode read task


void c_eint3_handler(void)
{
    lab_interrupt.handle_interrupt();
}

void Home( void * pvParameters )
{

    while(1)
    {


        if(xSemaphoreTake(switch_read,99999))
        {

            vTaskSuspendAll();
        }

  // vTaskDelete(NULL);
    }

}
void switch_ISR(void)
{

    long task_woken=0;
        xSemaphoreGiveFromISR(switch_read,&task_woken);
        if(task_woken){

            portYIELD_FROM_ISR(task_woken);
        }
      }

///

void Parser(void* pv_parameters)
{
    string gcode_string(gcode);
    stringstream gcode_stream(gcode_string);
    GCODEPARSER parser;
    while(1)
    {

        string line; // each line is stored in this string
        command move;

        while (getline(gcode_stream, line))  // will take the whole line
        {
            printf("\nParsing and sending %s\n",line.c_str());
            move = parser.parse(line);


            printf("Sending gcode lines\n");


            xQueueSend(Global_Queue_Handle,&(move),portMAX_DELAY);
            xEventGroupSetBits(sw_watchdog,prod_task_id);

        }

    }
}

void motor(void* pv_parameters)
{
    motorMove motor;
    command data_receive;
   // GPIO m1_dir(P1_28);
    m1_dir.setAsOutput();
   // GPIO m2_dir(P1_29);
    m2_dir.setAsOutput();
    float steps_x;
    float steps_y;
    float steps_d = 0;
    int l_switch = 0;
    float dist = 0;//, dist_y = 0;
    Pen move_Pen;
    while(1)
    {
        float prev_x, motion_x;
        float prev_y, motion_y;

        // Receives the structure from the parser task
        if(xQueueReceive( Global_Queue_Handle, &( data_receive ), portMAX_DELAY ))
        {

            vTaskSuspend(xHandle);

            printf("gcode received\n");
            string gcode__pwm(data_receive.beg);
            if(gcode__pwm == "G1")
            {
                printf("Motion:%s\n",data_receive.beg);
                printf("Received G1 and need to run motor linearly\n");
                printf("X_value:%f\n",data_receive.x);
                printf("Y_value:%f\n",data_receive.y);
                printf("Feed value:%f\n",data_receive.feedrate);


                motion_x= data_receive.x- prev_x;
                motion_y = data_receive.y- prev_y;
                dist = sqrt(((data_receive.x - prev_x)*(data_receive.x - prev_x)) + ((data_receive.y - prev_y)*(data_receive.y - prev_y)));
                /**
                 * This  below 2 line calculates the steps for the linear motion
                 * Assigns the steps to x and y motor
                 */
                steps_x= abs(dist/0.16);
                steps_y=abs(dist/0.16);
                /**
                 * This code below in if statement test for the diagonal movement
                 * Calculates the steps for the motor for diagonal movement.
                 */

                if(abs(motion_x) == abs(motion_y))
                {
                    steps_d= abs(dist/0.113);
                    l_switch = 1;
                }
                printf(" x step is %f",steps_x);
                printf(" y step is %f\n",steps_y);
                printf(" x prev %f",prev_x);
                printf(" y prev %f\n",prev_y);
                if(0 == l_switch)
                {
                    if(motion_x < 0 )  // move left
                    {
                        motor.leftMove(steps_x, m1_dir, m2_dir);

                    }
                    else if (motion_x > 0)    //move right
                    {
                        motor.rightMove(steps_x, m1_dir, m2_dir);
//                        m1_dir.setHigh();
//                        m2_dir.setHigh();
//                        printf("direction is right\n");
//                        for(float i=0; i<steps;i++)
//                        {
//                            LPC_GPIO2->FIOSET = (1<<1);
//                            LPC_GPIO2->FIOSET = (1<<0);
//                            if(i==624)
//                                printf("yes i is 624");
//                            delay_us(1000);
//                            LPC_GPIO2->FIOCLR=(1<<1);
//                            LPC_GPIO2->FIOCLR=(1<<0);
//                            delay_us(1000);
//                        }
//
                    }
                    else if(motion_y > 0)                   // up
                    {
                         motor.upMove(steps_y, m1_dir, m2_dir);
//                        m1_dir.setLow();
//                        m2_dir.setHigh();
//                        printf("direction is up\n");
//                        for(float i=0; i<steps_y;i++)
//                        {
//                            LPC_GPIO2->FIOSET = (1<<1);
//                            LPC_GPIO2->FIOSET = (1<<0);
//                            if(i==624)
//                                printf("yes i is 624");
//                            delay_us(1000);
//                            LPC_GPIO2->FIOCLR=(1<<1);
//                            LPC_GPIO2->FIOCLR=(1<<0);
//                            delay_us(1000);
//                        }
                    }

                     else if(motion_y<0)   //down
                     {
                         motor.downMove(steps_y, m1_dir, m2_dir);
//                        m1_dir.setHigh();
//                        m2_dir.setLow();
//                        printf("direction is down\n");
//                        for(float i=0; i<steps_y;i++)
//                        {
//                            LPC_GPIO2->FIOSET = (1<<1);
//                            LPC_GPIO2->FIOSET = (1<<0);
//                            if(i==624)
//                                printf("yes i is 624");
//                            delay_us(1000);
//                            LPC_GPIO2->FIOCLR=(1<<1);
//                            LPC_GPIO2->FIOCLR=(1<<0);
//                            delay_us(1000);
//                        }
                     }

                }
                else if(l_switch == 1)
                {
                     if((abs(motion_x) == abs(motion_y)) && (motion_x > 0) && (motion_y > 0))   // top right
                     {
                         motor.topRight(steps_d, m2_dir);
                     }

                     else if((abs(motion_x) == abs(motion_y)) && (motion_x < 0) && (motion_y > 0))   // top left
                     {
                         motor.topLeft(steps_d, m1_dir);
                     }

                     else if((abs(motion_x) == abs(motion_y)) && (motion_x < 0) && (motion_y < 0))   // bottom left
                     {
                         motor.bottomLeft(steps_d, m2_dir);
                     }

                     else if((abs(motion_x) == abs(motion_y)) && (motion_x > 0) && (motion_y < 0))   // bottom right
                     {
                         motor.bottomRight(steps_d, m1_dir);
                     }
                     l_switch = 0;
                }

                 prev_x=data_receive.x;
                 prev_y=data_receive.y;

            }
            else if (gcode__pwm == "G4")
            {
                printf("Motion:%s\n",data_receive.beg);
                printf("P_Value is %f\n", data_receive.p);
            }
            else if (gcode__pwm == "M300")
            {
                vTaskSuspend(xHandle);

                if(data_receive.s == 30.00)
                {
                    move_Pen.Pen_Down();

                }

                if(data_receive.s == 50.00)
                {
                    move_Pen.Pen_Up();
                }
//                printf("Motion:%s\n",data_receive.beg);
//                printf("S_Value is %f\n", data_receive.s);
            }

            xEventGroupSetBits(sw_watchdog,cons_task_id);
            vTaskResume(xHandle);
        }
    }


}

void soft_watchdog(void *p)
{
    motorMove home;
    while(1)
    {
        uint32_t result = xEventGroupWaitBits(sw_watchdog,task_all_bits,pdTRUE,pdTRUE,2000);

        if((result & task_all_bits)== task_all_bits)
        {
            printf("Both the task working fine\n");
        }
        else
        {

            if(!((result & cons_task_id)||(result & prod_task_id)))
            {
               printf("gcode is finished and now move towards the home position\n");

              // home.movehome(); // move the plotter towards home position
            }

        }

    }
}

int main(void)
{

    /**
         * Initialization of P2.0 and P2.2 as output for generating the steps.
         * This will be given as input to the step pin of the motor.
     */

//    LPC_PINCON->PINSEL4 &= ~(3<<0);  // set
//    LPC_PINCON->PINSEL4 &= ~(3<<2);
    LPC_PINCON->PINSEL4 &= ~(3<<0);  // set
    LPC_PINCON->PINSEL4 &= ~(3<<4);

//    LPC_GPIO2->FIODIR |=  (1<<0);
//    LPC_GPIO2->FIODIR |=  (1<<1);
        LPC_GPIO2->FIODIR |=  (1<<0);
        LPC_GPIO2->FIODIR |=  (1<<2);
    sw_watchdog = xEventGroupCreate();

///interrupt init start
    lab_interrupt.init(); // Init the interrupt
    vSemaphoreCreateBinary(switch_read);// for interrupt
    isr_register(EINT3_IRQn, c_eint3_handler); // for delegating the the interrupt ha
    lab_interrupt.attachInterruptHandler(0, 29, switch_ISR, rising_edge);
    ////Interrupt init

    Storage::read("1:edited rectanlge_aks.gcode",gcode,sizeof(gcode),850);
     printf("%s\n gcode is ",gcode);

    const uint32_t STACK_SIZE = 2048;
    Global_Queue_Handle= xQueueCreate(1,sizeof(command));
    //printf("hello in main");

    xTaskCreate(Parser, "Reading_Gcode", STACK_SIZE ,NULL,1,&xHandle);
    xTaskCreate(motor, "PWM_output", STACK_SIZE ,NULL,2,NULL);
    xTaskCreate(Home, "Home", STACK_SIZE ,NULL,1,NULL); //

    //create task for watchdog
       xTaskCreate(soft_watchdog, "software_watchdog", STACK_SIZE ,NULL,PRIORITY_HIGH,NULL);



    vTaskStartScheduler();
    return 0;

}



