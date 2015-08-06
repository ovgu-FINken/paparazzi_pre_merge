#include <led.h>

#include <libopencm3/stm32/i2c.h>

static inline void LED1_ON(void)
{
  LED_ON(4);
}

static inline void LED1_OFF(void)
{
  LED_OFF(4);
}

static inline void LED2_ON(void)
{
  LED_ON(5);
}

static inline void LED2_OFF(void)
{
  LED_OFF(5);
}

static inline void LED_ERROR(uint8_t base, uint8_t nr)
{
  LED2_ON();
  for (int i = 0; i < (base + nr); i++) {
    LED1_ON();
    LED1_OFF();
  }
  LED2_OFF();
}

static inline void LED_SHOW_ACTIVE_BITS(uint32_t i2c)
{
  uint16_t CR1 = I2C_CR1(i2c);
  uint16_t SR1 = I2C_SR1(i2c);
  uint16_t SR2 = I2C_SR2(i2c);
  // Note: reading SR1 and then SR2 will clear ADDR bits

  LED1_ON();

  // 1 Start
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_SB, SR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 2 Addr
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_ADDR, SR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 3 BTF
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_BTF, SR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 4 ERROR
  if ((SR1 & I2C_SR1_ERR_MASK) != 0x0000) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // Anything?
  if ((SR1 + SR2) != 0x0000) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  LED1_OFF();


  LED1_ON();

  // 1 Start
  if (BIT_X_IS_SET_IN_REG(I2C_CR1_START, CR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 2 Stop
  if (BIT_X_IS_SET_IN_REG(I2C_CR1_STOP, CR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 3 Busy
  if (BIT_X_IS_SET_IN_REG(I2C_SR2_BUSY, SR2)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 4 Tra
  if (BIT_X_IS_SET_IN_REG(I2C_SR2_TRA, SR2)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 5 Master
  if (BIT_X_IS_SET_IN_REG(I2C_SR2_MSL, SR2)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();
  LED1_OFF();

  //#define I2C_DEBUG_LED_CONTROL
#ifdef I2C_DEBUG_LED_CONTROL


  LED1_ON();

  // 1 Anything CR?
  if ((CR1) != 0x0000) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 2 PE
  if (BIT_X_IS_SET_IN_REG(I2C_CR1_PE, CR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 3 SWRESET
  if (BIT_X_IS_SET_IN_REG(I2C_CR1_SWRST, CR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  LED1_OFF();
#endif

}
