// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu
 
#include <iostream>
#include <pigpio.h>
#include "CRotaryEncoder.h"

int re_decoder::getNrOfPulsesLeftEnc()
{
   return m_nrOfPulsesLeft;
}

int re_decoder::getNrOfPulsesRightEnc()
{
   return m_nrOfPulsesRight;
}
   
void re_decoder::_pulse_left(int gpio, int level, uint32_t tick)
{
   if (gpio == m_mygpioA) 
   {
	   m_levA = level;
   } 
   else 
   {
	   m_levB = level;
   }

   if (gpio != m_lastGpio) /* debounce */
   {
      m_lastGpio = gpio;

      if ((gpio == m_mygpioA) && (level == 1))
      {
         if (m_levB)
         {
             m_nrOfPulsesLeft += 1;
         }
      }
      else if ((gpio == m_mygpioB) && (level == 1))
      {
         if (m_levA)
         {
             m_nrOfPulsesLeft -= 1;
         }
      }
   }
}

void re_decoder::_pulse_right(int gpio, int level, uint32_t tick)
{
   if (gpio == m_mygpioA) 
   {
	   m_levA = level;
   } 
   else 
   {
	   m_levB = level;
   }

   if (gpio != m_lastGpio) /* debounce */
   {
      m_lastGpio = gpio;

      if ((gpio == m_mygpioA) && (level == 1))
      {
         if (m_levB)
         {
             m_nrOfPulsesRight -= 1;
         }
      }
      else if ((gpio == m_mygpioB) && (level == 1))
      {
         if (m_levA)
         {
             m_nrOfPulsesRight += 1;
         }
      }
   }
}

re_decoder::re_decoder()
{
   m_nrOfPulsesLeft = 0;
   m_nrOfPulsesRight = 0;
}

void re_decoder::re_init_left(int gpioA, int gpioB)
{
   m_mygpioA = gpioA;
   m_mygpioB = gpioB;

   spdlog::info("Call from init_left_enc, mygpioA = {}, mygpioB = {}", m_mygpioA, m_mygpioB);

   m_levA=0;
   m_levB=0;

   m_lastGpio = -1;

   gpioSetMode(gpioA, PI_INPUT);
   gpioSetMode(gpioB, PI_INPUT);

   gpioSetPullUpDown(gpioA, PI_PUD_DOWN);
   gpioSetPullUpDown(gpioB, PI_PUD_DOWN);

   // monitor encoder level changes
   gpioSetAlertFuncEx(gpioA, &re_decoder::static_internal_pulse_left, this);
   gpioSetAlertFuncEx(gpioB, &re_decoder::static_internal_pulse_left, this);
}

void re_decoder::re_init_right(int gpioA, int gpioB)
{
   m_mygpioA = gpioA;
   m_mygpioB = gpioB;

   spdlog::info("Call from init_right_enc, mygpioA = {}, mygpioB = {}", m_mygpioA, m_mygpioB);

   m_levA=0;
   m_levB=0;

   m_lastGpio = -1;

   gpioSetMode(gpioA, PI_INPUT);
   gpioSetMode(gpioB, PI_INPUT);

   gpioSetPullUpDown(gpioA, PI_PUD_DOWN);
   gpioSetPullUpDown(gpioB, PI_PUD_DOWN);

   // monitor encoder level changes
   gpioSetAlertFuncEx(gpioA, &re_decoder::static_internal_pulse_right, this);
   gpioSetAlertFuncEx(gpioB, &re_decoder::static_internal_pulse_right, this);
}

void re_decoder::re_cancel(void)
{
   gpioSetAlertFuncEx(m_mygpioA, 0, this);
   gpioSetAlertFuncEx(m_mygpioB, 0, this);
}
