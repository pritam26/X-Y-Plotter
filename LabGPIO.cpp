/*
 * LabGPIO.cpp
 *
 *  Created on: Feb 15, 2018
 *      Author: prita
 */

/*
 * LabGPIO_0.cpp
 *
 *  Created on: Feb 15, 2018
 *      Author: prita
 */

#include "LabGPIO.hpp"

LabGPIO::LabGPIO(uint8_t port, uint8_t pin)
{
    portNum = port;
    pinNum = pin;

    if(portNum == 0)
    {
        if(pinNum < 16)
            LPC_PINCON->PINSEL0 &= ~(3<<pinNum);
        else
            LPC_PINCON->PINSEL1 &= ~(3<<pinNum);
    }
    else if(portNum == 1)
    {
        if(pinNum < 16)
            LPC_PINCON->PINSEL2 &= ~(3<<pinNum);
        else
            LPC_PINCON->PINSEL3 &= ~(3<<pinNum);
    }
    else
    {
        LPC_PINCON->PINSEL4 &= ~(3<<pinNum);
    }
}

void LabGPIO::setAsInput(void)
{
    if(portNum == 0)
    {
        LPC_GPIO0->FIODIR &= ~(1<<pinNum);
    }
    else if(portNum == 1)
    {
        LPC_GPIO1->FIODIR &= ~(1<<pinNum);
    }
    else
    {
        LPC_GPIO2->FIODIR &= ~(1<<pinNum);
    }
}

void LabGPIO::setAsOutput(void)
{
    if(portNum == 0)
    {
        LPC_GPIO0->FIODIR |= (1<<pinNum);
    }
    else if(portNum == 1)
    {
        LPC_GPIO1->FIODIR |= (1<<pinNum);
    }
    else
    {
        LPC_GPIO2->FIODIR |= (1<<pinNum);
    }
}

void LabGPIO::setDirection(bool output)
{
    if(output)
        setAsOutput();
    else
        setAsInput();
}

void LabGPIO::setHigh(void)
{
    if(portNum == 0)
    {
        LPC_GPIO0->FIOSET = (1<<pinNum);
    }
    else if (portNum == 1)
    {
        LPC_GPIO1->FIOSET = (1<<pinNum);
    }
    else
    {
        LPC_GPIO2->FIOSET = (1<<pinNum);
    }
}

void LabGPIO::setLow(void)
{
    if(portNum == 0)
    {
        LPC_GPIO0->FIOCLR = (1<<pinNum);
    }
    else if(portNum == 1)
    {
        LPC_GPIO1->FIOCLR = (1<<pinNum);
    }
    else
    {
        LPC_GPIO2->FIOCLR = (1<<pinNum);
    }
}

void LabGPIO::set(bool high)
{
    if(high) {setHigh();}
    else {setLow();}
}

bool LabGPIO::getLevel(void) const
{
    if (portNum == 0)
    {
        return !!(LPC_GPIO0->FIOPIN & (1<<pinNum));
    }
    else if (portNum == 1)
    {
        return !!(LPC_GPIO1->FIOPIN & (1<<pinNum));
    }
    else
    {
        return !!(LPC_GPIO2->FIOPIN & (1<<pinNum));
    }
}

LabGPIO::~LabGPIO(){}



