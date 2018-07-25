/*
 * GcodeParser.hpp
 *
 *  Created on: May 12, 2018
 *      Author: Gaurav Yadav
 */

#ifndef GCODEPARSER_HPP_
#define GCODEPARSER_HPP_

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

typedef struct
{

        char beg[8]; //motion type stored in string;
        float x;
        float y;
        float feedrate;
        float p;    // for G4
        float s;    // for M300
} command;

class GCODEPARSER {
public:
    GCODEPARSER();
    ~GCODEPARSER();

    // will receive a line and parse it to extract the desired code and values
    command parse(string& line);

};




#endif /* GCODEPARSER_HPP_ */
