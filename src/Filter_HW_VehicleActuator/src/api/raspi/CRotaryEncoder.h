// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CROTARY_ENCODER_H
#define CROTARY_ENCODER_H

#include <stdint.h>
#include <CyC_TYPES.h>

typedef void (*re_decoderCB_t)(int);

class re_decoder
{
private:
   int m_mygpioA, m_mygpioB, m_levA, m_levB, m_lastGpio;
    
   int m_nrOfPulsesLeft, m_nrOfPulsesRight;

   void _pulse_left(int gpio, int level, uint32_t tick);
   void _pulse_right(int gpio, int level, uint32_t tick);
   
   static void static_internal_pulse_left(int gpio, int level, uint32_t tick, void *userdata)
   {
       static_cast<re_decoder*>(userdata)->_pulse_left(gpio, level, tick);
   }
   
   static void static_internal_pulse_right(int gpio, int level, uint32_t tick, void *userdata)
   {
       static_cast<re_decoder*>(userdata)->_pulse_right(gpio, level, tick);
   }

public:
   // This function establishes a rotary encoder on gpioA and gpioB
   // When the encoder is turned the callback function is called
   void re_init_left(int gpioA, int gpioB);
   void re_init_right(int gpioA, int gpioB);
   
   int getNrOfPulsesLeftEnc();
   int getNrOfPulsesRightEnc();
   
   re_decoder();

   // This function releases the resources used by the decoder
   void re_cancel(void);
};

#endif //CROTARY_ENCODER_H
