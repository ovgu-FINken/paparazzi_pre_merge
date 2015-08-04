#include "modules/rgb_sensor/tcs34725_i2c.h"

#ifndef TCS34725_I2C_DEV
#define TCS34725_I2C_DEV i2c2
#endif

uint16_t red, green, blue;
struct i2c_transaction tcs34725_i2c_trans;
bool_t sensor_ok;

void tcs34725_write_reg(uint8_t reg, uint8_t val)
{
  tcs34725_i2c_trans.buf[0] = TCS34725_COMMAND_BIT | reg;
  tcs34725_i2c_trans.buf[1] = val;
  i2c_transmit(&TCS34725_I2C_DEV,&tcs34725_i2c_trans,TCS34725_ADDRESS<<1,2);
}

uint8_t tcs34725_read_reg(uint8_t reg)
{
  tcs34725_i2c_trans.buf[0] = TCS34725_COMMAND_BIT | reg;
  i2c_transmit(&TCS34725_I2C_DEV,&tcs34725_i2c_trans,TCS34725_ADDRESS<<1,1);
  tcs34725_i2c_trans.buf[0] = 0;
  tcs34725_i2c_trans.buf[1] = 0;
  i2c_receive(&TCS34725_I2C_DEV,&tcs34725_i2c_trans,(TCS34725_ADDRESS<<1)|1,1);
  return tcs34725_i2c_trans.buf[0];
}

void tcs34725_enable(void)
{
  tcs34725_write_reg(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  tcs34725_write_reg(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
}

void tcs34725_i2c_init(void)
{
  tcs34725_i2c_trans.buf[0] = 0;
  tcs34725_i2c_trans.buf[1] = 0;
  tcs34725_i2c_trans.status =  I2CTransDone;
  if(tcs34725_read_reg(TCS34725_ID) == 0x44)
  {
    sensor_ok = true;
  }
  else
  {
    sensor_ok = false;
  }
  tcs34725_enable();
}

void tcs34725_read_periodic(void)
{
  uint16_t redH, redL, greenH, greenL, blueH, blueL;

  if(!sensor_ok)
  {
    return;
  }

  redL = tcs34725_read_reg(TCS34725_RDATAL);
  redH = tcs34725_read_reg(TCS34725_RDATAH);
  greenL = tcs34725_read_reg(TCS34725_GDATAL);
  greenH = tcs34725_read_reg(TCS34725_GDATAH);
  blueL = tcs34725_read_reg(TCS34725_BDATAL);
  blueH = tcs34725_read_reg(TCS34725_BDATAH);

  red = (redH << 8) | redL;
  blue = (blueH << 8) | blueL;
  green = (greenH << 8) | greenL;
}

