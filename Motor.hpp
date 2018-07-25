/*
 * pwmMotor.hpp
 *
 *  Created on: May 16, 2018
 *      Author: prita
 */

#ifndef PWMMOTOR_HPP_
#define PWMMOTOR_HPP_

#include "FreeRTOS.h"
#include "task.h"
#include "LPC17xx.h"
#include "stdint.h"
#include <stdio.h>
#include "gpio.hpp"
#include "lpc_pwm.hpp"

class motorMove
{
    public:

        void movehome(GPIO&, GPIO&);

        void leftMove(float, GPIO&, GPIO&);

        void rightMove(float, GPIO&, GPIO&);

        void upMove(float, GPIO&, GPIO&);

        void downMove(float, GPIO&, GPIO&);

        void topRight(float, GPIO&);

        void topLeft(float, GPIO&);

        void bottomRight(float, GPIO&);

        void bottomLeft(float, GPIO&);

        ~motorMove();
};

class Pen{

public:
    Pen();
    ~Pen();

    void Pen_Up(); //servo Right
    void Pen_Down();// servo neutral

};

#endif /* PWMMOTOR_HPP_ */
