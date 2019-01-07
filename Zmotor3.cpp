/** @file Zmotor3.cpp

Inspired from Adafruit 16-channel PWM & Servo driver library



*/
#include "zmotor3.h"

#include <Wire.h>

#include "PinExtender.h"


uint32_t MOTOR3_PWM[]={PIN_MOTOR3_PWM_0,PIN_MOTOR3_PWM_1,PIN_MOTOR3_PWM_2,PIN_MOTOR3_PWM_3,PIN_MOTOR3_PWM_4,PIN_MOTOR3_PWM_5,PIN_MOTOR3_PWM_6,PIN_MOTOR3_PWM_7};
uint32_t MOTOR3_IO[]={PIN_MOTOR3_IO_0,PIN_MOTOR3_IO_1,PIN_MOTOR3_IO_2,PIN_MOTOR3_IO_3,PIN_MOTOR3_IO_4,PIN_MOTOR3_IO_5,PIN_MOTOR3_IO_6,PIN_MOTOR3_IO_7};


/**************************************************************************/
/*! 
    @brief  Instantiates a new Zmotor3 PWM driver chip with the I2C address on the Wire interface. On Due we use Wire1 since its the interface on the 'default' I2C pins.
    @param  addr The 7-bit I2C address to locate this chip, default is 0x40
*/
/**************************************************************************/
Zmotor3::Zmotor3() : PinExtender(),  io(),  pwm() 
 {
	 
}
/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/

  void Zmotor3::begin(TwoWire *i2c,uint8_t addr)
  {
	  begin(i2c, addr|MCP23017_ADDR_BASE, addr|PCA9685_ADDR_BASE);
  }
 void Zmotor3::begin(TwoWire *i2c,uint8_t addrio,uint8_t addrpwm)
  {
	  io.begin(i2c, addrio);
      pwm.begin(i2c, addrpwm);
    for (int i=0;i<16;i++)
     io.pinMode(io.getPin(i), OUTPUT);
    for (int i=0;i<16;i++)
     io.digitalWrite(io.getPin(i), LOW);
      
}
/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/

  void Zmotor3::begin(uint8_t addr)
  {
    begin(&Wire, addr);
}
/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/

  void Zmotor3::begin(void)
  {
	  begin( 0x00);
  }
  



/**************************************************************************/
/*! 
    @brief  Sends a reset command to the Zmotor3 chip over I2C
*/
/**************************************************************************/
void Zmotor3::reset(void) {
 // io.reset();
  pwm.reset();
  
}
/**************************************************************************/
/*! 
    @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
    @param  freq Floating point frequency that we will attempt to match
*/
/**************************************************************************/
void Zmotor3::setPWMFreq(float freq) {
	pwm.setPWMFreq( freq);
}


bool Zmotor3::acceptlocal(uint32_t p)
{
  if (pwm.acceptlocal( p))
  return true;
  if (io.acceptlocal( p))
  return true; 
  return false;
  
}

     
/** dummy function
*/
uint8_t Zmotor3::digitalRead(uint32_t ulPin)
{
	 if (pwm.acceptlocal( ulPin))
  return pwm.digitalRead(ulPin);
else
  if (io.acceptlocal( ulPin))
  return  io.digitalRead(ulPin);
	else if (_next)
		return _next->digitalRead( ulPin);	
return 0;	
}
/**
 * Sets the pin mode to either INPUT or OUTPUT but for all, and input doesn't exist
 */
void Zmotor3::pinMode(uint32_t ulPin, uint8_t mode) {
	
		 if (pwm.acceptlocal( ulPin))
   pwm.pinMode(ulPin,mode);
else
  if (io.acceptlocal( ulPin))
    io.pinMode(ulPin,mode);
	else if (_next)
		 _next->pinMode( ulPin,mode);	
	
	
}
void Zmotor3::analogWriteResolution(int res)
{
	pwm.analogWriteResolution(res);
	io.analogWriteResolution(res);
  if (_next)
		return _next->analogWriteResolution( res);		
}
 void Zmotor3::analogWrite( uint32_t ulPin, uint32_t ulValue ) 
 {
	 if (pwm.acceptlocal( ulPin))
   pwm.analogWrite(ulPin,ulValue);
else
  if (io.acceptlocal( ulPin))
    io.analogWrite(ulPin,ulValue);
	else if (_next)
		 _next->analogWrite( ulPin,ulValue);		
	
 }
 bool Zmotor3::test()
  {
    bool b=pwm.test();
    b&=io.test();
  return b;
  }

 uint32_t Zmotor3::getPin(uint32_t ulPin)
 {
	 if ((ulPin & (PCA9685_ADDR_BASE<<16))==(PCA9685_ADDR_BASE<<16))
	 return pwm.getPin( ulPin);
 	 if ((ulPin & (MCP23017_ADDR_BASE<<16))==(MCP23017_ADDR_BASE<<16))
	 return io.getPin( ulPin); 
     return NO_CHANNEL;
 }
 /*
 uint32_t Zmotor3::pin2channel(uint32_t ulPin)
 {
	 if (pwm.acceptlocal( ulPin))
	 return ulPin &0xff | PCA9685_ADDR_BASE<<16;
 	 if (io.acceptlocal( ulPin))
	 return ulPin &0xff | MCP23017_ADDR_BASE<<16;
 
 
 return NO_CHANNEL;
 }*/
 uint32_t Zmotor3::analogRead( uint32_t pin )
{ 
  return 0;
}
void Zmotor3::digitalWrite(uint32_t ulPin, uint8_t ulValue)
{
	if (pwm.acceptlocal( ulPin))
   pwm.digitalWrite(ulPin,ulValue);
else
  if (io.acceptlocal( ulPin))
    io.digitalWrite(ulPin,ulValue);
	else if (_next)
		 _next->digitalWrite( ulPin,ulValue);		
}