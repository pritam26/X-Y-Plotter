/*
 * GcodeParser.cpp
 *
 *  Created on: May 12, 2018
 *      Author: Gaurav Yadav
 */


#include "GcodeParser.hpp"



GCODEPARSER::GCODEPARSER() {
}

GCODEPARSER::~GCODEPARSER() {
}

command GCODEPARSER::parse(string& line)
{
            command motion;

                if (4 == sscanf(line.c_str(), "%s X%f Y%f F%f", motion.beg, &motion.x, &motion.y, &motion.feedrate))
                {
                    printf("motion is %s\n and x y f value is %f %f %f\n",motion.beg, motion.x,motion.y, motion.feedrate);
                }
                else if(2==sscanf(line.c_str(), "%s S%f", motion.beg, &motion.s))
                {
                    printf("motion is %s\n and S value is %f\n",motion.beg,motion.s);
                }
                else if(2==sscanf(line.c_str(), "%s P%f", motion.beg, &motion.p))
                {
                    printf("motion is %s\n and P value is %f\n",motion.beg,motion.p);
                }

                return motion;
}






