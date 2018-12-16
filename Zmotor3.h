/** @file Zmotor3.h

 @par dependency :
   this library use the folowing ones :   ZMCP23017, ZPCA9685, PinExtender,Rosserial_Arduino_Library
   you can find it on https://github.com/zoubworldArduino/
  
 
   @par description
   This lib support a board called Motor3 based on MCP23017 and PCA9685 with L298 as power stage.
   This board offers 8 motor channels 5-35V up to 2A DC.
   Each channel have 2 pin, one where we can apply LOW or HIGH level thanks to digitalWrite(), and one where we can apply PWM value thanks to analogWrite()
   The pin can be identify by the generic name like #PIN_MOTOR2_IO_0 #PIN_MOTOR3_PWM_0, in this case you have to do instanceBoard.digitalWrite(PIN_MOTOR3_IO_0,LOW)
   The pin can be identify by the instance name like pin=instanceBoard.getpin(#PIN_MOTOR2_IO_0) or pin=instanceBoard.getpinIo(MOTOR2_0) or  or pin=instanceBoard.getpinPwm(MOTOR2_0),
   in this case you have to do digitalWrite(pin,LOW); from arduino API, this offer a compatibility with any library, but before you should link your board to arduino API by calling setPinExtender(&instanceBoard);
   if you have 2 board, then do it :instanceBoard.setPinExtender(&instanceBoard_2);
   
   generic name is used inside the instance of board, and the instance pin name allow to use generic arduino IPA, it content on it the I2C addresse of the device and the channel.
   note that instanceBoard.getpin() must be called after instanceBoard creation and initialisation with begin(), the pin number is a 32bit number.
   
   The cmd(channel, pwm) function allow to manage easly the motor control, channel is between  0 and 15 like on skillprint of the board.
   The pwm value is between -4096 and 4095, 0 give no power.
   
   The name #PIN_MOTOR3_IO_0 can be replace by MOTOR3_IO[0].
   The name #PIN_MOTOR3_PWM_0 can be replace by MOTOR3_PWM[0].
   The name #PIN_MOTOR3_PWM_0 can be replace by MOTOR3_EN[0].
   

*/

#ifndef _ZMOTOR3_H
#define _ZMOTOR3_H


#include "PinExtender.h"
#include <ZMCP23017.h>
#include <ZPCA9685.h>
    
/** @name I2C base address
*/
//@{
#define MCP23017_ADDR_BASE 0x20
#define PCA9685_ADDR_BASE 0x40
//@}

/** @name generic pin name accoding to the channel number : MOTOR3_[pin]_[channel]
*/
//@{
#define PIN_MOTOR3_IO_0  (MCP23017_GPB1 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_IO_1  (MCP23017_GPB3 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_IO_2  (MCP23017_GPB5 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_IO_3  (MCP23017_GPB7 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_IO_4  (MCP23017_GPA1 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_IO_5  (MCP23017_GPA3 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_IO_6  (MCP23017_GPA5 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_IO_7  (MCP23017_GPA7 | MCP23017_ADDR_BASE<<16)

#define PIN_MOTOR3_EN_0  (MCP23017_GPB0 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_EN_1  (MCP23017_GPB2 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_EN_2  (MCP23017_GPB4 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_EN_3  (MCP23017_GPB6 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_EN_4  (MCP23017_GPA0 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_EN_5  (MCP23017_GPA2 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_EN_6  (MCP23017_GPA4 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR3_EN_7  (MCP23017_GPA6 | MCP23017_ADDR_BASE<<16)

#define PIN_MOTOR3_PWM_0  (PCA9685_LED3 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR3_PWM_1  (PCA9685_LED2 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR3_PWM_2  (PCA9685_LED1 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR3_PWM_3  (PCA9685_LED0 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR3_PWM_4  (PCA9685_LED4 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR3_PWM_5  (PCA9685_LED5 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR3_PWM_6  (PCA9685_LED6  | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR3_PWM_7  (PCA9685_LED7  | PCA9685_ADDR_BASE<<16)
//@}

/** @name generic pin name accoding the the board connector : MOTOR3_[connector]_[pin]
*/
//@{
#define MOTOR3_P8_PWM PIN_MOTOR3_PWM_0 
#define MOTOR3_P8_IO PIN_MOTOR3_IO_0 
#define MOTOR3_P8_EN PIN_MOTOR3_EN_0 

#define MOTOR3_P12_PWM PIN_MOTOR3_PWM_1 
#define MOTOR3_P12_IO PIN_MOTOR3_IO_1
#define MOTOR3_P12_EN PIN_MOTOR3_EN_1


#define MOTOR3_P7_PWM PIN_MOTOR3_PWM_2 
#define MOTOR3_P7_IO PIN_MOTOR3_IO_2
#define MOTOR3_P7_EN PIN_MOTOR3_EN_2

#define MOTOR3_P11_PWM PIN_MOTOR3_PWM_3
#define MOTOR3_P11_IO PIN_MOTOR3_IO_3
#define MOTOR3_P11_EN PIN_MOTOR3_EN_3
   
#define MOTOR3_P6_PWM PIN_MOTOR3_PWM_4
#define MOTOR3_P6_IO PIN_MOTOR3_IO_4
#define MOTOR3_P6_EN PIN_MOTOR3_EN_4

#define MOTOR3_P10_PWM PIN_MOTOR3_PWM_5 
#define MOTOR3_P10_IO PIN_MOTOR3_IO_5
#define MOTOR3_P10_EN PIN_MOTOR3_EN_5

#define MOTOR3_P9_PWM PIN_MOTOR3_PWM_6
#define MOTOR3_P9_IO PIN_MOTOR3_IO_6
#define MOTOR3_P9_EN PIN_MOTOR3_EN_6

#define MOTOR3_P13_PWM PIN_MOTOR3_PWM_7 
#define MOTOR3_P13_IO PIN_MOTOR3_IO_7
#define MOTOR3_P13_EN PIN_MOTOR3_EN_7
//@}

/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with ZPCA9685 PWM chip
*/
/**************************************************************************/
class Zmotor3 : public PinExtender  {
 public:
 Zmotor3();
 
  /** initialise the board,
  the Wire interface must be initialize before. see wire.begin()
  */
  void begin(
  TwoWire *MyWire,//!< the Wire interface like &Wire for board that handle several one.
  uint8_t addr1//!< the I2C address custom bit,the #PCA9685_ADDR_BASE, #MCP23017_ADDR_BASE will be added to target each chip.
  );
  /** initialise the board,
  the Wire interface must be initialize before. see wire.begin()
  */
   void begin(TwoWire *i2c,//!< the Wire interface like &Wire for board that handle several one.
   uint8_t addrio,//!< the I2C address of MCP23017
   uint8_t addrpwm//!< the I2C address of PCA9685
   );
  /** initialise the board,
  the Wire interface must be initialize before. see wire.begin()
  */
  void begin(uint8_t addr,//!< the I2C address of MCP23017
  uint8_t addr2//!< the I2C address of PCA9685
  );
  /** initialise the board,
  the Wire interface must be initialize before. see wire.begin()
  */
  void begin(uint8_t addr//!< the I2C address custom bit,the #PCA9685_ADDR_BASE, #MCP23017_ADDR_BASE will be added to target each chip.
  );
  /** initialise the board,
  the Wire interface must be initialize before. see wire.begin()
  default Wire and addresses will be used : #PCA9685_ADDR_BASE, #MCP23017_ADDR_BASE 
  */
  
  void begin(void);
  /** Test the Hardware to be sure that the connection is good.
  else it return false.
  @return true : if the communication work well and it look like that it is the good chip behing I2C interface
  */
  bool test();
 /** Reset the board like a power up.
  
  
  */
  void SWRST (void);
  /**  check the board
  @deprecated
  */
bool check();
 /** same function as Arduino API : pinMode()
 */
  void pinMode(uint32_t p, uint8_t d);
  /** same function as Arduino API : digitalWrite()
 */
 
  void digitalWrite(uint32_t p, uint8_t d);
  /** same function as Arduino API : digitalRead()
 */
  uint8_t digitalRead(uint32_t p);
  /**  same function as Arduino API : analogWrite()
 
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
 void analogWrite( uint32_t ulPin, uint32_t ulValue ) ;
/** same function as Arduino API : analogWriteResolution()

*/
  void analogWriteResolution(int res //!< possible value 8 10 12
  );

  
  /** return the pin number of the current instance for the generic pin ulPin
  
  This allow to use Aduino API see PinExtender documentation
  */
 uint32_t getPin(uint32_t ulPin//!<the generic pin name like #PIN_MOTOR2_IO_0 #PIN_PIN_MOTOR3_PWM_0
 );
 /** Reset the board like a power up.
  
  
  */
  void reset(void);
 /** same function as Arduino API : analogRead()

*/
  uint32_t analogRead( uint32_t pin );
 /** set up the frequency of the PWM
 note max is about 1500 Hz

*/
  void setPWMFreq(float freq//!< Frequency in Hz
  );
  protected:
  /** test if the pin number existe on the current board.
  */
       bool acceptlocal(uint32_t p);
 private: 
 /** instance of ZPCA9685 driver
 */
 ZPCA9685  pwm;
 /** instance of ZMCP23017 driver
 */
 
 ZMCP23017  io;
 
  
};


#endif
