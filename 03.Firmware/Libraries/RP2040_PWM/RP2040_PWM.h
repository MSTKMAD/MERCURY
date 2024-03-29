/****************************************************************************************************************************
  RP2040_PWM.h
  For RP2040 boards
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/RP2040_PWM
  Licensed under MIT license

  Now even you use all these new 16 ISR-based timers,with their maximum interval practically unlimited (limited only by
  unsigned long miliseconds), you just consume only one RP2040 STM32 timer and avoid conflicting with other cores' tasks.
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.

  Version: 1.3.1

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K.Hoang      21/09/2021 Initial coding for RP2040 using ArduinoCore-mbed or arduino-pico core
  1.0.1   K.Hoang      24/09/2021 Fix bug generating wrong frequency
  1.0.2   K.Hoang      04/10/2021 Fix bug not changing frequency dynamically
  1.0.3   K.Hoang      05/10/2021 Not reprogram if same PWM frequency. Add PIO strict `lib_compat_mode`
  1.0.4   K Hoang      22/10/2021 Fix platform in library.json for PIO
  1.0.5   K Hoang      06/01/2022 Permit changing dutyCycle and keep same frequency on-the-fly
  1.1.0   K Hoang      24/02/2022 Permit PWM output for both channels of PWM slice. Use float instead of double
  1.1.1   K Hoang      06/03/2022 Fix compiler warnings. Display informational warning when debug level > 3
  1.2.0   K Hoang      16/04/2022 Add manual setPWM function to use in wafeform creation
  1.3.0   K Hoang      16/04/2022 Add setPWM_Int function for optional uint32_t dutycycle = real_dutycycle * 1000
  1.3.1   K Hoang      11/09/2022 Add minimal example `PWM_Basic`
*****************************************************************************************************************************/

#pragma once

#ifndef RP2040_PWM_H
#define RP2040_PWM_H

#if ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && defined(ARDUINO_ARCH_MBED)
  #if defined(USING_MBED_RP2040_PWM)
    #undef USING_MBED_RP2040_PWM
  #endif  
  #define USING_MBED_RP2040_PWM       true
  
  #if(_PWM_LOGLEVEL_>3)
    #warning USING_MBED_RP2040_PWM in RP2040_PWM.h
  #endif
  
#elif ( defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) ||  \
        defined(ARDUINO_ADAFRUIT_ITSYBITSY_RP2040) || defined(ARDUINO_ADAFRUIT_QTPY_RP2040) || defined(ARDUINO_ADAFRUIT_STEMMAFRIEND_RP2040) ||  \
        defined(ARDUINO_ADAFRUIT_STEMMAFRIEND_RP2040) || defined(ARDUINO_ADAFRUIT_TRINKEYQT_RP2040) || defined(ARDUINO_ADAFRUIT_MACROPAD_RP2040) ||  \
        defined(ARDUINO_ADAFRUIT_KB2040_RP2040) || defined(ARDUINO_ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_CYTRON_MAKER_NANO_RP2040) ||  \
        defined(ARDUINO_CYTRON_MAKER_PI_RP2040) || defined(ARDUINO_SPARKFUN_PROMICRO_RP2040) || defined(ARDUINO_CHALLENGER_2040_WIFI_RP2040) ||  \
        defined(ARDUINO_CHALLENGER_2040_LTE_RP2040) || defined(ARDUINO_CHALLENGER_NB_2040_WIFI_RP2040) || defined(ARDUINO_ILABS_2040_RPICO32_RP2040) ||  \
        defined(ARDUINO_MELOPERO_SHAKE_RP2040) || defined(ARDUINO_SOLDERPARTY_RP2040_STAMP) || defined(ARDUINO_UPESY_RP2040_DEVKIT) ||  \
        defined(ARDUINO_WIZNET_5100S_EVB_PICO) || defined(ARDUINO_GENERIC_RP2040) ) && !defined(ARDUINO_ARCH_MBED) 
  #if defined(USING_RP2040_PWM)
    #undef USING_RP2040_PWM
  #endif  
  #define USING_RP2040_PWM            true
  
  #if(_PWM_LOGLEVEL_>3)
    #warning USING_RP2040_PWM in RP2040_PWM.h
  #endif
#else
  #error This code is intended to run on the RP2040 mbed_nano, mbed_rp2040 or arduino-pico platform! Please check your Tools->Board setting.
#endif

#ifndef RP2040_PWM_VERSION
  #define RP2040_PWM_VERSION           "RP2040_PWM v1.3.1"
  
  #define RP2040_PWM_VERSION_MAJOR     1
  #define RP2040_PWM_VERSION_MINOR     3
  #define RP2040_PWM_VERSION_PATCH     1

  #define RP2040_PWM_VERSION_INT       1003001
#endif

#include <math.h>
#include <float.h>
#include "hardware/pwm.h"

#include "PWM_Generic_Debug.h"

#define MAX_PWM_FREQUENCY        (62500000.0f)
#define MIN_PWM_FREQUENCY        (7.5f)

// New from v1.1.0
///////////////////////

#if !defined(NUM_PWM_SLICES)
  #define NUM_PWM_SLICES      8
#endif

typedef struct 
{
  float freq;
  float channelA_div;
  float channelB_div;
  bool  channelA_Active;
  bool  channelB_Active;
} PWM_slice;

static PWM_slice PWM_slice_data[NUM_PWM_SLICES] =
{
  { 0, 0, 0, false, false },
  { 0, 0, 0, false, false },
  { 0, 0, 0, false, false },
  { 0, 0, 0, false, false },
  { 0, 0, 0, false, false },
  { 0, 0, 0, false, false },
  { 0, 0, 0, false, false },
  { 0, 0, 0, false, false }
};

// Not using float for waveform creating
typedef struct 
{
  uint16_t channelA_div;
  uint16_t channelB_div;
  bool  channelA_Active;
  bool  channelB_Active;
  bool  initialized;
} PWM_slice_manual;

// Default to 0% PWM
static PWM_slice_manual PWM_slice_manual_data[NUM_PWM_SLICES] =
{
  { 0, 0, false, false, false },
  { 0, 0, false, false, false },
  { 0, 0, false, false, false },
  { 0, 0, false, false, false },
  { 0, 0, false, false, false },
  { 0, 0, false, false, false },
  { 0, 0, false, false, false },
  { 0, 0, false, false, false }
};
///////////////////////

///////////////////////////////////////////////////////////////////

class RP2040_PWM
{ 
  public:
  
  RP2040_PWM(const uint8_t& pin, const float& frequency, const float& dutycycle, bool phaseCorrect = false)
  {
#if defined(F_CPU)
    freq_CPU = F_CPU;
#else
    freq_CPU = 125000000;
#endif

    _pin          = pin;
    _frequency    = frequency;
    _dutycycle    = dutycycle * 1000;
    
    _phaseCorrect = phaseCorrect;
     
    if (!calc_TOP_and_DIV(frequency))
    {
      _frequency  = 0;
    }
    else
    {
      _frequency    = frequency;
    }
             
    _enabled      = false;
  }
  
  ///////////////////////////////////////////
  
  ~RP2040_PWM();
  
  ///////////////////////////////////////////
  
  bool setPWM()
  {
    return setPWM_Int(_pin, _frequency, _dutycycle, _phaseCorrect);
  }
  
  ///////////////////////////////////////////
  
  // To be called only after previous complete setPWM_manual with top and div params
  // by checking PWM_slice_manual_data[_slice_num].initialized == true;
  bool setPWM_manual(const uint8_t& pin, uint16_t& level)
  {         
    _pin = pin;
    
    // Limit level <= _PWM_config.top
    if (level > _PWM_config.top)
      level = _PWM_config.top;
    
    gpio_set_function(_pin, GPIO_FUNC_PWM);
    
    _slice_num = pwm_gpio_to_slice_num(_pin);
    
    if (!PWM_slice_manual_data[_slice_num].initialized)
    {
      PWM_LOGERROR1("Error, not initialized for PWM pin = ", _pin);
      
      return false;
    }
    
    pwm_set_gpio_level(_pin, level);
           
    // From v1.1.0
    ////////////////////////////////
    
    if ( (pwm_gpio_to_channel(_pin)) == PWM_CHAN_A)
    {
      PWM_slice_manual_data[_slice_num].channelA_div    = level;
      PWM_slice_manual_data[_slice_num].channelA_Active = true;
      
      // If B is active, set the data now
      if (PWM_slice_manual_data[_slice_num].channelB_Active)
      {
        pwm_set_chan_level(_slice_num, PWM_CHAN_B, PWM_slice_manual_data[_slice_num].channelB_div);
      }
    }
    else if ( (pwm_gpio_to_channel(_pin)) == PWM_CHAN_B)
    {
      PWM_slice_manual_data[_slice_num].channelB_div     = level;
      PWM_slice_manual_data[_slice_num].channelB_Active  = true;
      
      // If A is active, set the data now
      if (PWM_slice_manual_data[_slice_num].channelA_Active)
      {
        pwm_set_chan_level(_slice_num, PWM_CHAN_A, PWM_slice_manual_data[_slice_num].channelA_div);
      }
    }
    else
    {
      PWM_LOGERROR1("Error, not correct PWM pin = ", _pin);
      
      return false;
    }
    
    pwm_set_enabled(_slice_num, true);
      
    PWM_LOGINFO3("pin = ", _pin, ", PWM_CHAN =", pwm_gpio_to_channel(_pin));
    
    ////////////////////////////////
    
    _enabled = true;
    
    PWM_LOGINFO7("PWM enabled, slice =", _slice_num, ", top =", _PWM_config.top,
                 ", div =", _PWM_config.div, ", level =", level);

    return true;  
  }
  
  ///////////////////////////////////////////
  
  bool setPWM_manual(const uint8_t& pin, const uint16_t& top, const uint8_t& div, 
                     uint16_t& level, bool phaseCorrect = false)
  {   
    _pin = pin;
    
    _PWM_config.top = top;
    _PWM_config.div = div;

    // Limit level <= top
    if (level > top)
      level = top;
    
    gpio_set_function(_pin, GPIO_FUNC_PWM);
    
    _slice_num = pwm_gpio_to_slice_num(_pin);
    
    pwm_config config = pwm_get_default_config();
           
    // Set phaseCorrect
    pwm_set_phase_correct(_slice_num, phaseCorrect);
       
    pwm_config_set_clkdiv_int(&config, _PWM_config.div);
    pwm_config_set_wrap(&config, _PWM_config.top);
    
    // auto start running once configured
    pwm_init(_slice_num, &config, true);
    pwm_set_gpio_level(_pin, level);
    
    // Store and flag so that simpler setPWM_manual() can be called without top and div
    PWM_slice_manual_data[_slice_num].initialized = true;
           
    // From v1.1.0
    ////////////////////////////////
    // Update PWM_slice_manual_data[]
    
    if ( (pwm_gpio_to_channel(_pin)) == PWM_CHAN_A)
    {
      PWM_slice_manual_data[_slice_num].channelA_div    = level;
      PWM_slice_manual_data[_slice_num].channelA_Active = true;
      
      // If B is active, set the data now
      if (PWM_slice_manual_data[_slice_num].channelB_Active)
      {
        pwm_set_chan_level(_slice_num, PWM_CHAN_B, PWM_slice_manual_data[_slice_num].channelB_div);
      }
    }
    else if ( (pwm_gpio_to_channel(_pin)) == PWM_CHAN_B)
    {
      PWM_slice_manual_data[_slice_num].channelB_div     = level;
      PWM_slice_manual_data[_slice_num].channelB_Active  = true;
      
      // If A is active, set the data now
      if (PWM_slice_manual_data[_slice_num].channelA_Active)
      {
        pwm_set_chan_level(_slice_num, PWM_CHAN_A, PWM_slice_manual_data[_slice_num].channelA_div);
      }
    }
    else
    {
      PWM_LOGERROR1("Error, not correct PWM pin = ", _pin);
      
      return false;
    }
    
    pwm_set_enabled(_slice_num, true);
      
    PWM_LOGINFO3("pin = ", _pin, ", PWM_CHAN =", pwm_gpio_to_channel(_pin));
    
    ////////////////////////////////
    
    _enabled = true;
    
    PWM_LOGINFO7("PWM enabled, slice =", _slice_num, ", top =", _PWM_config.top,
                 ", div =", _PWM_config.div, ", level =", level);

    return true;  
  }
  
  ///////////////////////////////////////////
  
  // dutycycle from 0-100,000 for 0%-100% to make use of 16-bit top register
  // dutycycle = real_dutycycle * 1000 for better accuracy
  bool setPWM_Int(const uint8_t& pin, const float& frequency, const uint32_t& dutycycle, bool phaseCorrect = false)
  {
    bool newFreq      = false;
    bool newDutyCycle = false;
    
    if ( (frequency <= MAX_PWM_FREQUENCY) && (frequency >= MIN_PWM_FREQUENCY) )
    {   
      _pin = pin;
      
      if (_frequency != frequency)
      {
        if (!calc_TOP_and_DIV(frequency))
        {
          _frequency  = 0;
        }
        else
        {
          _frequency  = frequency;
          _dutycycle  = dutycycle;
          
          newFreq     = true;
          
          PWM_LOGINFO3("Changing PWM frequency to", frequency, "and dutyCycle =", (float) _dutycycle / 1000);
        }
      }
      else if (_enabled)
      {
        if (_dutycycle != dutycycle)
        {
          _dutycycle   = dutycycle;         
          newDutyCycle = true;
          
          PWM_LOGINFO3("Changing PWM DutyCycle to", (float) _dutycycle / 1000, "and keeping frequency =", _frequency);

        }
        else
        {
          PWM_LOGINFO3("No change, same PWM frequency =", frequency, "and dutyCycle =", (float) _dutycycle / 1000);
        }
      }
      
      if ( (!_enabled) || newFreq || newDutyCycle )
      {
        gpio_set_function(_pin, GPIO_FUNC_PWM);
        
        _slice_num = pwm_gpio_to_slice_num(_pin);
        
        pwm_config config = pwm_get_default_config();
               
        // Set phaseCorrect
        pwm_set_phase_correct(_slice_num, phaseCorrect);
           
        pwm_config_set_clkdiv_int(&config, _PWM_config.div);
        pwm_config_set_wrap(&config, _PWM_config.top);
        
        // auto start running once configured
        pwm_init(_slice_num, &config, true);
        
        // To avoid uint32_t overflow and still keep accuracy as _dutycycle max = 100,000 > 65536 of uint16_t
        pwm_set_gpio_level(_pin, ( _PWM_config.top * (_dutycycle / 2) ) / 50000 );
               
        // From v1.1.0
        ////////////////////////////////
        // Update PWM_slice_data[]
        PWM_slice_data[_slice_num].freq = _frequency;
        
        if ( (pwm_gpio_to_channel(_pin)) == PWM_CHAN_A)
        {
          PWM_slice_data[_slice_num].channelA_div     = ( _PWM_config.top * (_dutycycle / 2) ) / 50000;
          PWM_slice_data[_slice_num].channelA_Active  = true;
          
          // If B is active, set the data now
          if (PWM_slice_data[_slice_num].channelB_Active)
          {
            pwm_set_chan_level(_slice_num, PWM_CHAN_B, PWM_slice_data[_slice_num].channelB_div);
          }
        }
        else if ( (pwm_gpio_to_channel(_pin)) == PWM_CHAN_B)
        {
          PWM_slice_data[_slice_num].channelB_div     = ( _PWM_config.top * (_dutycycle / 2) ) / 50000;
          PWM_slice_data[_slice_num].channelB_Active  = true;
          
          // If A is active, set the data now
          if (PWM_slice_data[_slice_num].channelA_Active)
          {
            pwm_set_chan_level(_slice_num, PWM_CHAN_A, PWM_slice_data[_slice_num].channelA_div);
          }
        }
        else
        {
          PWM_LOGERROR1("Error, not correct PWM pin = ", _pin);
          
          return false;
        }
        
        pwm_set_enabled(_slice_num, true);
          
        PWM_LOGINFO3("pin = ", _pin, ", PWM_CHAN =", pwm_gpio_to_channel(_pin));
        
        ////////////////////////////////
        
        _enabled = true;
        
        PWM_LOGINFO3("PWM enabled, slice = ", _slice_num, ", _frequency = ", _frequency);
      }
    
      return true;
    }
    else
      return false;
  }
  
  ///////////////////////////////////////////
  
  bool setPWM(const uint8_t& pin, const float& frequency, const float& dutycycle, bool phaseCorrect = false)
  {
    return setPWM_Int(pin, frequency, dutycycle * 1000, phaseCorrect);
  }
  
  ///////////////////////////////////////////

  bool setPWM_Period(const uint8_t& pin, const float& period_us, const float& dutycycle, bool phaseCorrect = false)
  {
    return setPWM_Int(pin, 1000000.0f / period_us, dutycycle * 1000, phaseCorrect);
  }
  
  ///////////////////////////////////////////
  
  void enablePWM()
  {
    pwm_set_enabled(_slice_num, true);
    _enabled = true;
  }
  
  ///////////////////////////////////////////
  
  void disablePWM()
  {
    pwm_set_enabled(_slice_num, false);
    _enabled = false;
  }
  
  ///////////////////////////////////////////
  
  inline uint32_t get_TOP()
  {
    return _PWM_config.top;
  }
  
  ///////////////////////////////////////////
  
  inline uint32_t get_DIV()
  {
    return _PWM_config.div;
  }
  
  ///////////////////////////////////////////
  
  inline float getActualFreq()
  {
    return _actualFrequency;
  }
  
  ///////////////////////////////////////////
  
  inline uint32_t get_freq_CPU()
  {
    return freq_CPU;
  }
  
  ///////////////////////////////////////////////////////////////////
  
  private:
  
  pwm_config  _PWM_config;
  uint32_t    freq_CPU;
  
  float       _actualFrequency;
  float       _frequency;
  
  // dutycycle from 0-100,000 for 0%-100% to make use of 16-bit top register
  // dutycycle = real_dutycycle * 1000 for better accuracy
  uint32_t    _dutycycle;
  //////////
  
  uint8_t     _pin;
  uint8_t     _slice_num;  
  bool        _phaseCorrect;
  bool        _enabled;
  
  ///////////////////////////////////////////
  
  // https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf, page 549
  // https://raspberrypi.github.io/pico-sdk-doxygen/group__hardware__pwm.html
  
  ///////////////////////////////////////////
  
  bool calc_TOP_and_DIV(const float& freq)
  {           
    if (freq >= 2000.0)
    {
      _PWM_config.div = 1;
    }
    else if (freq >= 200.0) 
    {
      _PWM_config.div = 10;
    }
    else if (freq >= 20.0) 
    {
      _PWM_config.div = 100;
    }
    else if (freq >= 10.0) 
    {
      _PWM_config.div = 200;
    }
    else if (freq >= MIN_PWM_FREQUENCY)
    {
      _PWM_config.div = 255;
    }
    else
    {
      PWM_LOGERROR1("Error, freq must be >=", MIN_PWM_FREQUENCY);
      
      return false;
    }
    
    // Formula => PWM_Freq = ( F_CPU ) / [ ( TOP + 1 ) * ( DIV + DIV_FRAC/16) ]
    _PWM_config.top = ( freq_CPU / freq / _PWM_config.div ) - 1;
    
    _actualFrequency = ( freq_CPU  ) / ( (_PWM_config.top + 1) * _PWM_config.div );
    
    PWM_LOGINFO3("_PWM_config.top =", _PWM_config.top, ", _actualFrequency =", _actualFrequency);
    
    return true; 
  }
};

///////////////////////////////////////////

#endif    // RP2040_PWM_H

