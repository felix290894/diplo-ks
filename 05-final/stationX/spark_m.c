#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/delay.h>

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
#define spark_CTRL_OS_258MS (BIT(5) | BIT(4)) /*sobre muestreo*/



//Register functions
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Team StationX");
MODULE_DESCRIPTION("Homebrew driver for sparkfun weather station");
MODULE_VERSION("0.1");

struct spark_data {
  struct i2c_client *client;
  struct mutex lock;
  u8 ctrl_reg1;
};

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
if (test_bit(1,indio_dev->active_scan_mask)){
  ret = i2c_smbus_read_i2c_block_data(data->client,
				      spark_OUT_TEMP,2,&buffer[pos]);
  if(ret<0){
    mutex_unlock(&data->lock)
      goto done;
  }
 }
mutex_unlock(&data->lock);
iio_push_to_buffer_with_timestamp(indio_dev,buffer,
				  iio_get_time_ns(indio_dev));
done:
iio_trigger_notify_done(indio_dev->trig);
return IRQ_HANDLED;
}

static const struct iio_chan_spec spark_channels[] = {
  {
    .type = IIO_PRESSURE,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
    BIT(IIO_CHAN_INFO_SCALE),
    .scan_index = 0,
    .scan_type = {
      .sign = 'u',
      .realbits = 20,
      .storagebits = 32,
      .shift = 12
      .endianness = IIO_BE,
    }
  },
  {
    .type = IIO_TEMP,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
    BIT(IIO_CHAN_INFO_SCALE),
    .scan_index = 1,
    .scan_type = {
      .sign = 's',
      .realbits = 12,
      .storagebits = 16,
      .shift = 4,
      .endianness = IIO_BE,
  }
  },
  {
    .type = IIO_TEMP,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
    BIT(IIO_CHAN_INFO_SCALE),
    .scan_index_separate = 1,
    .scan_type = {
      .sign = 's',
      .realbits = 12,
      .storagebits = 16,
      .shift = 4,
      .endianness = IIO_BE,
    }
    {,
     IIO_CHAN_SOFT_TIMESTAMP(2),
    },
    static const struct iio_info_spark_info = {
      .read_raw = &spark_read_raw,
      .driver_module = THIS_MODULE,
    };
    static int spark_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
    {
      struct spark_data *data;
      struct iio_dev *indio_dev;
      int ret;
      ret = i2c_smbus_read_byte_data(client,spark_WHO_AM_I);
      if(ret < 0)
	return ret;
      if (ret != spark_DEVICE_ID)
	return -ENODEV;
      data = iio_priv(indio_dev);
      data->client = client;
      mutex_init(&data->lock);
      i2c_set_clientdata(client,indio_dev);
      indio_dev->info = &spark_info;
      indio_dev->name = id->name;
      indio_dev->dev.parent = &client->dev;
      indio_dev->modes = INDIO_DIRECT_MODE;
      indio_dev->channels = spark_channels;
      indio_dev_<num_channels = ARRAY_SIZE(spark_channels);
      /*reset*/
      i2c_smbus_write_byte_data(client, spark_CTRL_REG1,
				spark_CTRL_RESET);
      msleep(50);
      data->ctrl_reg1 = spark_CTRL_OS_258MS;
      ret = i2c_smbus_write_byte_data(client, spark_CTRL_REG1,
				      data->ctrl_reg1);
      if(ret<0)
	return ret;
      ret = iio_triggered_buffer_setup_setup(indio_dev, NULL,
					    spark_trigger_handler,NULL);
      if(ret<0)
	return ret;
      ret = iio_device_register(indio_dev);
      if(ret<0)
	goto buffer_cleanup;
      return 0;
    buffer_cleanup:
      iio_triggered_buffer_cleanup(indio_dev);
      return ret;
    }

    static int spark_standby(struct spark_data *data)
    {
      return i2c_smbus_write_byte_data(data->client, spark_CTRL_REG1,
				       data->ctrl_reg1 & ~spark_CTRL_ACTIVE);
    }
    static int spark_remove(struct i2c_client *client)
    {
      struct iio_dev *indio_dev = i2c_get_clientdata(client);
      iio_device_unregister(indio_dev);
      iio_triggered_buffer_cleanup(indio_dev);
      spark_standby(iio_priv(indio_dev));
      return 0;
    }

#ifdef CONFIG_PM_SLEEP
    static int spark_suspend(struct device *dev)
    {
      return spark_standby(iio_priv(i2c_get_clientdata(
						       to_i2c_client(dev))));
    }
    static int spark_resume (struct device *dev)
    {
      struct spark_data *data = iio_priv(i2c_get_clientdata(
							    to_i2c_client(dev)));
      return i2c_smbus_write_byte_data(data->client, spark_CTRL_REG1,
				       data->ctrl_reg1);
    }
    static SIMPLE_DEV_PM_OPS(spark_pm_ops,spark_suspend, spark_resume);
#define spark_PM_OPS (&spark_pm_ops)
    #else
    #define spark_PM_OPS NULL
    #endif
    static const struct i2c_device_id spark_id[]={
      {"spark",0},
      {}
    },
    MODULE_DEVICE_TABLE(i2c,spark_id);
    static struct i2c_driver spark_driver = {
      .driver = {
	.name = "spark",
	.pm = spark_PM_OPS,
      },
      .probe = spark_probe,
      .remove = spark_remove,
      .id_table = spark_id,
    };

static struct spark_algorithm spfun_algorithm = {
  .name          = "fun_algorithm",
  .id            = I2C_ALGO_SMBUS,
  .smbus_xfer    = fun_access,
  .functionality = spfun_func,
};

    module_i2c_driver(spark_driver);
    /*
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
  }*/
