/*
 * pwmMotor.cpp
 *
 *  Created on: May 16, 2018
 *      Author: prita
 */

#include "Motor.hpp"
#include "utilities.h"
#include "gpio.hpp"

void motorMove::movehome(GPIO& m1_dir, GPIO& m2_dir)
{
       m1_dir.setLow();
       m2_dir.setLow();
       while(1)
       {
           LPC_GPIO2->FIOSET = (1<<2);
           LPC_GPIO2->FIOSET = (1<<0);
           delay_us(1000);
           LPC_GPIO2->FIOCLR=(1<<2);
           LPC_GPIO2->FIOCLR=(1<<0);
           delay_us(1000);
       }
}


void motorMove::leftMove(float steps, GPIO& m1_dir, GPIO& m2_dir)
{
    m1_dir.setLow();
    m2_dir.setLow();
    printf("direction is left\n");
    for(float i=0; i<steps;i++)
    {
        LPC_GPIO2->FIOSET = (1<<2);
        LPC_GPIO2->FIOSET = (1<<0);
        delay_us(1000);
        LPC_GPIO2->FIOCLR=(1<<2);
        LPC_GPIO2->FIOCLR=(1<<0);
        delay_us(1000);
    }
}

void motorMove::rightMove(float steps, GPIO& m1_dir, GPIO& m2_dir)
{
        m1_dir.setHigh();
        m2_dir.setHigh();
        printf("direction is right\n");
        for(float i=0; i<steps;i++)
        {
            LPC_GPIO2->FIOSET = (1<<2);
            LPC_GPIO2->FIOSET = (1<<0);
            delay_us(1000);
            LPC_GPIO2->FIOCLR=(1<<2);
            LPC_GPIO2->FIOCLR=(1<<0);
            delay_us(1000);
        }
}


void motorMove::upMove(float steps_y, GPIO& m1_dir, GPIO& m2_dir)
{
        m1_dir.setLow();
        m2_dir.setHigh();
        printf("direction is up\n");
        for(float i=0; i<steps_y;i++)
        {
            LPC_GPIO2->FIOSET = (1<<2);
            LPC_GPIO2->FIOSET = (1<<0);
            delay_us(1000);
            LPC_GPIO2->FIOCLR=(1<<2);
            LPC_GPIO2->FIOCLR=(1<<0);
            delay_us(1000);
        }
}


void motorMove::downMove(float steps_y, GPIO& m1_dir, GPIO& m2_dir)
{
        m1_dir.setHigh();
        m2_dir.setLow();
        printf("direction is down\n");
        for(float i=0; i<steps_y;i++)
        {
            LPC_GPIO2->FIOSET = (1<<2);
            LPC_GPIO2->FIOSET = (1<<0);
            delay_us(1000);
            LPC_GPIO2->FIOCLR=(1<<2);
            LPC_GPIO2->FIOCLR=(1<<0);
            delay_us(1000);
        }
}

void motorMove::topRight(float steps_d, GPIO& m2_dir)
{
    m2_dir.setHigh();
    printf("direction is top right\n");
    for(float i=0; i<steps_d;i++)
    {
        LPC_GPIO2->FIOSET = (1<<2);
        delay_us(1000);
        LPC_GPIO2->FIOCLR=(1<<2);
        delay_us(1000);
    }
}

void motorMove::topLeft(float steps_d, GPIO& m1_dir)
{
    m1_dir.setLow();
    printf("direction is top left\n");
    for(float i=0; i<steps_d;i++)
    {
        LPC_GPIO2->FIOSET = (1<<0);
        delay_us(1000);
        LPC_GPIO2->FIOCLR=(1<<0);
        delay_us(1000);
    }
}

void motorMove::bottomRight(float steps_d, GPIO& m1_dir)
{
    m1_dir.setHigh();
    printf("direction is bottom right\n");
    for(float i=0; i<steps_d;i++)
    {
        LPC_GPIO2->FIOSET = (1<<0);
        delay_us(1000);
        LPC_GPIO2->FIOCLR=(1<<0);
        delay_us(1000);
    }
}

void motorMove::bottomLeft(float steps_d, GPIO& m2_dir)
{
    m2_dir.setLow();
    printf("direction is bottom left\n");
    for(float i=0; i<steps_d;i++)
    {
        LPC_GPIO2->FIOSET = (1<<2);
        delay_us(1000);
        LPC_GPIO2->FIOCLR=(1<<2);
        delay_us(1000);
    }
}

Pen::Pen(){}

void Pen::Pen_Down(){

//    PWM servo(PWM::pwm2, 50);
//        servo.set(7.5);
//        delay_ms(1000);

    PWM servo(PWM::pwm2, 50);
    servo.set(7.5);
    delay_ms(1000);

}

void Pen::Pen_Up(){
//    PWM servo(PWM::pwm2, 50);
//        servo.set(5);
//        delay_ms(1000);

    PWM servo(PWM::pwm2, 50);
    servo.set(5);
    delay_ms(1000);

}

motorMove::~motorMove()
{}

Pen::~Pen(){}
