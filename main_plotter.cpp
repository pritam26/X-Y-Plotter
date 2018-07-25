/*
 * main_plotter.cpp
 *
 *  Created on: May 12, 2018
 *      Author: Gaurav Yadav
 */

#if 0
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


QueueHandle_t Global_Queue_Handle=0;
QueueHandle_t Global_Queue_Handle_M1=0;
QueueHandle_t Global_Queue_Handle_M2=0;
TaskHandle_t xHandle;
const char gcode[200] = "G1 X-50.00 Y0.00 F3500.00    \n"
                       "M300 S50.00 (pen up)   \n"
                       "G4 P150 (wait 150ms) \n"
                        "G1 X-25.00 Y0.00 F3500.00  \n"
                        "M300 S30.00 (pen down)   \n"
                        "G1 X-25.00 Y-50.00 F3500.00  \n"
                        "M300 S50.00 (pen up)   \n";
string gcode_string(gcode);
stringstream gcode_stream(gcode_string);

void Read_Gcode(void* pv_parameters)
{
    GCODEPARSER parser;
    while(1)
    {

        string line; // each line is stored in this string
        command move;

        float steps_x;
        float steps_y;

        while (getline(gcode_stream, line))  // will take the whole line
        {
            float prev_x,motion_x;
            float prev_y,motion_y;

            printf("\nParsing and sending %s\n",line.c_str());
            move = parser.parse(line);
            printf("Motion:%s\n",move.beg);
            printf("X_value:%f\n",move.x);
            printf("Y_value:%f\n",move.y);
            printf("Feed value:%f\n",move.feedrate);

            motion_x= move.x- prev_x;
            motion_y = move.y- prev_y;
            if(motion_x<=0)
            {
                printf(" movement should be left\n");

            }

            steps_x= abs(motion_x/0.16);
            steps_y=abs(motion_y/0.16);
            printf(" x step is %f",steps_x);
            printf(" y step is %f",steps_y);
            //printf("Sending gcode lines\n");


           // xQueueSend(Global_Queue_Handle,&(move),portMAX_DELAY);
            xQueueSend(Global_Queue_Handle_M1,&(steps_x),portMAX_DELAY);
            xQueueSend(Global_Queue_Handle_M1,&(steps_y),portMAX_DELAY);

        }

    }
}

void Motor1 (void* pv_parameters)
{
    while(1)
    {

    }

}

void Motor2 (void* pv_parameters)
{
    while(1)
    {

    }
}

#if 0
void PWM_output(void* pv_parameters)
{
    command data_recieve;
    GPIO m1_dir(P1_28);
        m1_dir.setAsOutput();
        GPIO m2_dir(P1_29);
        m2_dir.setAsOutput();
        float steps;
        float steps_y;
    while(1)
    {
        float prev_x,motion_x;
        float prev_y,motion_y;

        if(xQueueReceive( Global_Queue_Handle, &( data_recieve ), portMAX_DELAY ))
        {

            vTaskSuspend(xHandle);

            printf("gcode received\n");
            string gcode__pwm(data_recieve.beg);
            if(gcode__pwm == "G1")
            {
                printf("Motion:%s\n",data_recieve.beg);
                printf("Received G1 and need to run motor linearly\n");
                printf("X_value:%f\n",data_recieve.x);
                printf("Y_value:%f\n",data_recieve.y);
                printf("Feed value:%f\n",data_recieve.feedrate);


                motion_x= data_recieve.x- prev_x;
                motion_y = data_recieve.y- prev_y;

                steps= abs(motion_x/0.16);
                steps_y=abs(motion_y/0.16);
                printf(" x step is %f",steps);
                printf(" y step is %f",steps_y);

                if(motion_x <= 0 )  // move left
                {
                    m1_dir.setLow();
                    m2_dir.setLow();
                    printf("direction is left\n");
                    for(float i=0; i<steps;i++)
                    {
                        LPC_GPIO2->FIOSET = (1<<1);
                        LPC_GPIO2->FIOSET = (1<<0);
                      /*  if(i==624)
                            printf("yes i is 624");*/
                        delay_us(1000);
                        LPC_GPIO2->FIOCLR=(1<<1);
                        LPC_GPIO2->FIOCLR=(1<<0);
                        delay_us(1000);
                    }


                }
                else if (motion_x > 0)    //move right
                {
                    m1_dir.setHigh();
                    m2_dir.setHigh();
                    printf("direction is right\n");
                    for(float i=0; i<steps;i++)
                    {
                        LPC_GPIO2->FIOSET = (1<<1);
                        LPC_GPIO2->FIOSET = (1<<0);
                        /*if(i==624)
                            printf("yes i is 624");*/
                        delay_us(1000);
                        LPC_GPIO2->FIOCLR=(1<<1);
                        LPC_GPIO2->FIOCLR=(1<<0);
                        delay_us(1000);
                    }

                }
                 if(motion_y > 0)                   // move up
                {
                    m1_dir.setLow();
                    m2_dir.setHigh();
                    printf("direction is up\n");
                    for(float i=0; i<steps_y;i++)
                    {
                        LPC_GPIO2->FIOSET = (1<<1);
                        LPC_GPIO2->FIOSET = (1<<0);
                       /* if(i==624)
                            printf("yes i is 624");*/
                        delay_us(1000);
                        LPC_GPIO2->FIOCLR=(1<<1);
                        LPC_GPIO2->FIOCLR=(1<<0);
                        delay_us(1000);
                    }
                }

                else if(motion_y<0)   //move down
                {
                    m1_dir.setHigh();
                    m2_dir.setLow();
                    printf("direction is down\n");
                    for(float i=0; i<steps_y;i++)
                    {
                        LPC_GPIO2->FIOSET = (1<<1);
                        LPC_GPIO2->FIOSET = (1<<0);
                     /*   if(i==624)
                            printf("yes i is 624");*/
                        delay_us(1000);
                        LPC_GPIO2->FIOCLR=(1<<1);
                        LPC_GPIO2->FIOCLR=(1<<0);
                        delay_us(1000);
                    }
                }


                prev_x=data_recieve.x;
                prev_y=data_recieve.y;

            }


            else if (gcode__pwm == "G4")
            {
                printf("Motion:%s\n",data_recieve.beg);
                printf("P_Value is %f\n", data_recieve.p);
            }
            else if (gcode__pwm == "M300")
            {
                printf("Motion:%s\n",data_recieve.beg);
                printf("S_Value is %f\n", data_recieve.s);
            }


            vTaskResume(xHandle);
        }
    }


}
#endif

int main(void)
{


    LPC_PINCON->PINSEL4 &= ~(3<<0);  // set
      LPC_PINCON->PINSEL4 &= ~(3<<2);

      LPC_GPIO2->FIODIR |=  (1<<0);
      LPC_GPIO2->FIODIR |=  (1<<1);

const uint32_t STACK_SIZE = 2048;
//Global_Queue_Handle= xQueueCreate(1,sizeof(command));
Global_Queue_Handle_M1= xQueueCreate(1,sizeof(float));
Global_Queue_Handle_M2= xQueueCreate(1,sizeof(float));
//printf("hello in main");

   xTaskCreate(Read_Gcode, "Reading_Gcode", STACK_SIZE ,NULL,1,&xHandle);
  // xTaskCreate(PWM_output, "PWM_output", STACK_SIZE ,NULL,2,NULL);
   xTaskCreate(Motor1, "Motor1", STACK_SIZE ,NULL,2,NULL);
   xTaskCreate(Motor2, "Motor2", STACK_SIZE ,NULL,2,NULL);


 vTaskStartScheduler();
  return 0;

}
#endif

#if 0
#include "gpio.hpp"

int main(void)
{
  int steps= 625;
  /*GPIO mypin(P1_28);
  mypin.setAsOutput();
  mypin.setHigh();
 */
 // GPIO mymotor1(P1_28);
  LPC_PINCON->PINSEL4 &= ~(3<<0);
  LPC_PINCON->PINSEL4 &= ~(3<<2);

  LPC_GPIO2->FIODIR |=  (1<<0);
  LPC_GPIO2->FIODIR |=  (1<<1);

//  mymotor1.setAsOutput();
    for (int i = 0; i < steps; i++) {
        // Use the pin as output pin
        LPC_GPIO2->FIOSET = (1 << 0);     // Pin will now be at 3.3v
        LPC_GPIO2->FIOSET = (1 << 1);
          delay_us(500);   //1.7
          LPC_GPIO2->FIOCLR = (1 << 0);        // Pin will now be at 0.0v
          LPC_GPIO2->FIOCLR = (1 << 1);
         delay_us(500);       // 0.3
    }
  return 0;

}

#endif

#if 0
#include "FreeRTOS.h"
#include "task.h"
#include "LPC17xx.h"
#include "stdint.h"
#include <stdio.h>
#include <lpc_pwm.hpp>
#include <GPIOLED_assig/LabGPIO.hpp>
#include "utilities.h"

LabGPIO_1 Dir1(29);
LabGPIO_1 Dir2(28);


int main(void)
{

    //  Dir.setHigh();
    PWM motor1(PWM::pwm1, 1000);
    PWM motor2(PWM::pwm2, 500);
    Dir1.setAsOutput();
    Dir2.setAsOutput();
   // Dir1.setHigh();
    Dir2.setHigh();
    //PWM servo(PWM::pwm3,50);

    //motor2.set(50);
    motor2.set(50);
    //Dir.setHigh();

delay_ms(600);






    //vTaskStartScheduler();
    return 0;

}
#endif





