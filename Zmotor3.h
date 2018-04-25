/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _ZMOTOR3_H
#define _ZMOTOR3_H


#include "PinExtender.h"
#include <ZMCP23017.h>
#include <ZPCA9685.h>
    

#define MCP23017_ADDR_BASE 0x20
#define PCA9685_ADDR_BASE 0x40
#define MOTOR3_IO_0  (MCP23017_GPB1 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_IO_1  (MCP23017_GPB3 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_IO_2  (MCP23017_GPB5 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_IO_3  (MCP23017_GPB7 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_IO_4  (MCP23017_GPA1 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_IO_5  (MCP23017_GPA3 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_IO_6  (MCP23017_GPA5 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_IO_7  (MCP23017_GPA7 | MCP23017_ADDR_BASE<<16)

#define MOTOR3_EN_0  (MCP23017_GPB0 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_EN_1  (MCP23017_GPB2 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_EN_2  (MCP23017_GPB4 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_EN_3  (MCP23017_GPB6 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_EN_4  (MCP23017_GPA0 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_EN_5  (MCP23017_GPA2 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_EN_6  (MCP23017_GPA4 | MCP23017_ADDR_BASE<<16)
#define MOTOR3_EN_7  (MCP23017_GPA6 | MCP23017_ADDR_BASE<<16)

#define MOTOR3_PWM_0  (PCA9685_LED3 | PCA9685_ADDR_BASE<<16)
#define MOTOR3_PWM_1  (PCA9685_LED2 | PCA9685_ADDR_BASE<<16)
#define MOTOR3_PWM_2  (PCA9685_LED1 | PCA9685_ADDR_BASE<<16)
#define MOTOR3_PWM_3  (PCA9685_LED0 | PCA9685_ADDR_BASE<<16)
#define MOTOR3_PWM_4  (PCA9685_LED4 | PCA9685_ADDR_BASE<<16)
#define MOTOR3_PWM_5  (PCA9685_LED5 | PCA9685_ADDR_BASE<<16)
#define MOTOR3_PWM_6  (PCA9685_LED6  | PCA9685_ADDR_BASE<<16)
#define MOTOR3_PWM_7  (PCA9685_LED7  | PCA9685_ADDR_BASE<<16)

#define MOTOR3_P8_PWM MOTOR3_PWM_0 
#define MOTOR3_P8_IO MOTOR3_IO_0 
#define MOTOR3_P8_EN MOTOR3_EN_0 

#define MOTOR3_P12_PWM MOTOR3_PWM_1 
#define MOTOR3_P12_IO MOTOR3_IO_1
#define MOTOR3_P12_EN MOTOR3_EN_1


#define MOTOR3_P7_PWM MOTOR3_PWM_2 
#define MOTOR3_P7_IO MOTOR3_IO_2
#define MOTOR3_P7_EN MOTOR3_EN_2

#define MOTOR3_P11_PWM MOTOR3_PWM_3
#define MOTOR3_P11_IO MOTOR3_IO_3
#define MOTOR3_P11_EN MOTOR3_EN_3
   
#define MOTOR3_P6_PWM MOTOR3_PWM_4
#define MOTOR3_P6_IO MOTOR3_IO_4
#define MOTOR3_P6_EN MOTOR3_EN_4

#define MOTOR3_P10_PWM MOTOR3_PWM_5 
#define MOTOR3_P10_IO MOTOR3_IO_5
#define MOTOR3_P10_EN MOTOR3_EN_5

#define MOTOR3_P9_PWM MOTOR3_PWM_6
#define MOTOR3_P9_IO MOTOR3_IO_6
#define MOTOR3_P9_EN MOTOR3_EN_6

#define MOTOR3_P13_PWM MOTOR3_PWM_7 
#define MOTOR3_P13_IO MOTOR3_IO_7
#define MOTOR3_P13_EN MOTOR3_EN_7


/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with ZPCA9685 PWM chip
*/
/**************************************************************************/
class Zmotor3 : public PinExtender  {
 public:
 Zmotor3();
  void begin(TwoWire *MyWire,uint8_t addr1);
   void begin(TwoWire *i2c,uint8_t addrio,uint8_t addrpwm);
  void begin(uint8_t addr,uint8_t addr2);
  void begin(uint8_t addr);
  void begin(void);
  void SWRST (void);
bool check();

  void pinMode(uint32_t p, uint8_t d);
  void digitalWrite(uint32_t p, uint8_t d);
  uint8_t digitalRead(uint32_t p);
  /*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
 void analogWrite( uint32_t ulPin, uint32_t ulValue ) ;

  void analogWriteResolution(int res);

  
 uint32_t getPin(uint32_t ulPin);
  void reset(void);
 
  uint32_t analogRead( uint32_t pin );

  void setPWMFreq(float freq);
  protected:
       bool acceptlocal(uint32_t p);
 private: 
 ZPCA9685  pwm;
 ZMCP23017  io;
 
  
};


#endif
