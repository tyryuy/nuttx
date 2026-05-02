#include <nuttx/config.h>

#include <syslog.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lsm6dsl.h>

#include "../include/board.h"
#include "nrf52_gpio.h"
#include "nrf52_i2c.h"
#include "xiao-nrf52840.h"

#define LSM6DS3TRC_I2C_ADDR  0x6a

static int nrf52_lsm6ds3_readreg(FAR struct i2c_master_s *i2c,
                                 uint8_t addr, uint8_t regaddr,
                                 FAR uint8_t *regval)
{
  struct i2c_config_s config;
  int ret;

  config.frequency = CONFIG_LSM6DSL_I2C_FREQUENCY;
  config.address   = addr;
  config.addrlen   = 7;

  ret = i2c_write(i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      return ret;
    }

  return i2c_read(i2c, &config, regval, sizeof(*regval));
}

int nrf52_lsm6ds3_initialize(void)
{
  struct i2c_master_s *i2c;
  uint8_t regval;
  int ret;

  /* Power on IMU */

  syslog(LOG_INFO, "IMUDBG: powering LSM6DS3TR-C\n");
  nrf52_gpio_config(BOARD_IMU_PWR_PIN);
  nrf52_gpio_write(BOARD_IMU_PWR_PIN, true);
  usleep(10000);

  i2c = nrf52_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: failed to initialize I2C0\n");
      return -ENODEV;
    }

  ret = nrf52_lsm6ds3_readreg(i2c, LSM6DS3TRC_I2C_ADDR, LSM6DSL_WHO_AM_I,
                              &regval);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "IMUDBG: WHO_AM_I read failed addr=0x%02x reg=0x%02x ret=%d\n",
             LSM6DS3TRC_I2C_ADDR, LSM6DSL_WHO_AM_I, ret);
    }
  else
    {
      syslog(LOG_INFO,
             "IMUDBG: WHO_AM_I addr=0x%02x reg=0x%02x value=0x%02x\n",
             LSM6DS3TRC_I2C_ADDR, LSM6DSL_WHO_AM_I, regval);
    }

  ret = lsm6dsl_sensor_register("/dev/accel0", i2c, LSM6DS3TRC_I2C_ADDR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to register LSM6DS3TR-C: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "LSM6DS3TR-C registered as /dev/accel0\n");
  return OK;
}
