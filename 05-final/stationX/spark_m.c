#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/param.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <uasm/uacces.h>  //used to move data to from kernel to user space
#include <linux/spinlock.h>
#include <asm/atomic.h>

#define DRV_NAME "spark"
#define spark_STATUS 0x00
#define spark_OUT_PRESS 0x01
#define spark_OUT_TEMP 0x04
#define spark_WHO_AM_I 0x0c
#define spark_CONTROL_REG1 0x26
#define spark_DEVICE_ID 0xc4
#define spark_STATUS_PRESS_RDY BIT(2)
#define spark_STATUS_TEMP_RDY BIT(1)
#define spark_CTRL_RESET BIT(2)
#define spark_CTRL_OST BIT(1)
#define spark_CTRL_ACTIVE BIT(0)
#define spark_CTRL_OS_258MS (BIT(5) | bit(4)) /*sobremuestreo*/



//Register functions
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Team StationX");
MODULE_DESCRIPTION("Homebrew driver for sparkfun weather station");
MODULE_VERSION("0.1");

struct spark_data {
  struct i2c_client *client;
  struct mutex lock;
  u8 ctrl_reg1;
}

static int spark_request(struct spark_data *data){
  int ret, tries = 15;
  ret= i2c_smbus_write_byte_data(data->client,spark_CTRL_REG1,
				 data->ctrl_reg1 | spark_CTRL_OST);
  if (ret<0)
      return ret;
  while (tries-- >0) {
    ret =i2c_smbus_read_byte_data(data->client,spark_CTRL_REG1);
    if(ret<0)
      return ret;
    if(!(ret & spark_CTRL_OST))
      break;
    msleep(20);
  }
  if(tries<0){
    dev_err(&data->client->dev, "data not ready\n");
    return -EIO;
    }
  return 0;
 }

static int spark_read_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int *val, int val, long mask){
  struct spark_data *data = iio_priv(indio_dev);
  __be32 tmp=0;
  int ret;
  switch (mask){
  case IIO_CHAN_INFO_RAW:
    if(iio_buffer_enabled(indio_dev))
      return -EBUSY;
    switch(chan->type){
    case IIO_PRESSURE: /*lsb*/
      mutex_lock(&data->lock);
      ret = spark_request(data);
    if(ret < 0){
	mutex_unlock(&data->lock)
    return ret;
      }

    ret = i2c_smbus_read_i2c_block_data(data->client,
					spark_OUT_PRESS,3,(u8 *)&tmp);
    mutex_unlock(&data->lock);
    if(ret < 0)
      return re;
    *val = be2_to_cpu(tmp)>>12;
    return IIO_VAL_INT;
    case IIO_TEMP: /* LSB */
      mutex_lock(&data->lock);
      ret = spark_reques(data);
      if(ret < 0){
	mutex_unlock(&data->lock);
      return ret;
      }
      ret = i2c_smbus_read_i2c_block_data(data->client,
					  spark_OUT_TEMP,2,(u8 *) &tmp);
      mutex_unlock(&data->lock);
      if(ret<0)
	return ret;
      *val = sign_extend32(be32_to_cpu(tmp)>>20,11);
      return IIO_VAL_INT;
    default:
      return -EINVAL;
    }

  case IIO_CHAN_INFO_SCALE:
    switch (chan->type) {
    case IIO_PRESSURE:
      *val = 0;
      *val2 = 250;
      return IIO_VAL_INT_PLUS_MICRO;
    case IIO_TEMP:
      *val = 0;
      *val2 = 62500;
      return IIO_VAL_INT_PLUS_MICRO;
    default:
      return -EINVAL;
    }    
  }
  return -EINVAL;
}

static irqreturn_t spark_trigger_handler(int irq, void *p)
{
  struct iio_poll_func +pf = p;
  struct iio_dev *indio_dev = pf -> indio_dev;
  struct spark_data *data = iio_priv(indio_dev);
  u8 buffer[16];
  int ret, pos =0;
  mutex_lock(&data->lock);
  goto done;
}
memset(buffer, 0,sizeof(buffer));
if(test_bit(0,indio_dev->active_scan_mask)){
  ret = i2c_smbus_read_i2c_block_data(data->client,
				      spark_OUT_PRESS,3,&buffer[pos]);
  if(ret<0){
    mutex_unlock(&data->lock);
    goto done;
  }
  pos = +4;
 }
if (test_bit(1,indio_dev))
    

			   

static struct spark_algorithm spfun_algorithm = {
  .name          = "fun_algorithm",
  .id            = I2C_ALGO_SMBUS,
  .smbus_xfer    = fun_access,
  .functionality = spfun_func,
};

static u32 spfun_func(struct spark_adapter *adapter)
{
  return I2C_FUNC_SMBUS_QUICK |
         I2C_FUNC_SMBUS_BYTE |
         I2C_FUNC_SMBUS_BYTE_DATA |
         I2C_FUNC_SMBUS_WORD_DATA |
         I2C_FUNC_SMBUS_BLOCK_DATA;
}

static s32 spfun_access(struct spark_adapter *adap,
			u16 addr,
			unsigned short flags,
			char read_write,
			u8 command,
			int size,
			union i2c_smbus_data *data)
{
  int i, len;
  dev_info(&adap->dev, "%s was called with the"
	   "following parameters:\n",
	   __FUNCTION__);
  dev_info(&adap->dev, "addr = %.4x\n",addr);
  dev_info(&adap->swv, "flags = %.4x\n",flags)
}
