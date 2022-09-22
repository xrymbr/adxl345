#include <linux/input.h>	/* BUS_SPI */
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/pm.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "adxl34x.h"



struct adxl345 {
	unsigned int irq;
	struct gpio_desc *apxl_int1_gpio;
};



static int major;
static struct class *apxl345class;

static struct adxl345 *adxl345;
static struct spi_device *apxl345_spi;


/* 驱动提供的设备信息 */
static const struct adxl34x_platform_data adxl345_default_init = {
	.tap_threshold = 35,
	.tap_duration = 3,
	.tap_latency = 20,
	.tap_window = 20,
	.tap_axis_control = ADXL_TAP_X_EN | ADXL_TAP_Y_EN | ADXL_TAP_Z_EN,
	.act_axis_control = 0xFF,
	.activity_threshold = 6,
	.inactivity_threshold = 4,
	.inactivity_time = 3,
	.free_fall_threshold = 8,
	.free_fall_time = 0x20,
	.data_rate = 8,
	.data_range = ADXL_FULL_RES,

	.ev_type = EV_ABS,
	.ev_code_x = ABS_X,	/* EV_REL */
	.ev_code_y = ABS_Y,	/* EV_REL */
	.ev_code_z = ABS_Z,	/* EV_REL */

	.ev_code_tap = {BTN_TOUCH, BTN_TOUCH, BTN_TOUCH}, /* EV_KEY {x,y,z} */
	.power_mode = ADXL_AUTO_SLEEP | ADXL_LINK,
	.fifo_mode = ADXL_FIFO_STREAM,
	.watermark = 0,
};





/* 字符设备驱动 */
static int apxl345_open(struct inode *inode, struct file *file)
{
	return 0;
}



static struct file_operations apxl345ops = {
    .owner = THIS_MODULE,
	.open  = apxl345_open,	
};



static int adxl345_spi_write(struct device *dev, unsigned char reg, unsigned char val)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char buf[2];

	buf[0] = ADXL34X_WRITECMD(reg);
	buf[1] = val;

	return spi_write(spi, buf, sizeof(buf));
}
static int adxl345_spi_read(struct device *dev, unsigned char reg)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char cmd;

	cmd = ADXL34X_READCMD(reg);

	return spi_w8r8(spi, cmd);
}
static int adxl345_spi_read_block(struct device *dev, unsigned char reg, int count, void *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	ssize_t status;

	reg = ADXL34X_READMB_CMD(reg);
	status = spi_write_then_read(spi, &reg, 1, buf, count);	
	
	return (status < 0) ? status : 0;
	
}



/* adxl345 spi设备传输函数 */
static const struct adxl34x_bus_ops adxl345_spi_bops = {
	.bustype	= BUS_SPI,
	.write		= adxl345_spi_write,
	.read		= adxl345_spi_read,
	.read_block	= adxl345_spi_read_block,
};




/* 数据处理系统 */
static void adxl34x_get_triple(struct adxl34x *ac, struct axis_triple *axis)
{
	short buf[3];

	ac->bops->read_block(ac->dev, DATAX0, DATAZ1 - DATAX0 + 1, buf);

	mutex_lock(&ac->mutex);
	ac->saved.x = (s16) le16_to_cpu(buf[0]);
	axis->x = ac->saved.x;

	ac->saved.y = (s16) le16_to_cpu(buf[1]);
	axis->y = ac->saved.y;

	ac->saved.z = (s16) le16_to_cpu(buf[2]);
	axis->z = ac->saved.z;
	mutex_unlock(&ac->mutex);
}

static void adxl34x_service_ev_fifo(struct adxl34x *ac)
{
	struct adxl34x_platform_data *pdata = &ac->pdata;
	struct axis_triple axis;

	adxl34x_get_triple(ac, &axis);

	input_event(ac->input, pdata->ev_type, pdata->ev_code_x,
		    axis.x - ac->swcal.x);
	input_event(ac->input, pdata->ev_type, pdata->ev_code_y,
		    axis.y - ac->swcal.y);
	input_event(ac->input, pdata->ev_type, pdata->ev_code_z,
		    axis.z - ac->swcal.z);
}

static void adxl34x_report_key_single(struct input_dev *input, int key)
{
	input_report_key(input, key, true);
	input_sync(input);
	input_report_key(input, key, false);
}

static void adxl34x_send_key_events(struct adxl34x *ac,
		struct adxl34x_platform_data *pdata, int status, int press)
{
	int i;

	for (i = ADXL_X_AXIS; i <= ADXL_Z_AXIS; i++) {
		if (status & (1 << (ADXL_Z_AXIS - i)))
			input_report_key(ac->input,
					 pdata->ev_code_tap[i], press);
	}
}

static void adxl34x_do_tap(struct adxl34x *ac,
		struct adxl34x_platform_data *pdata, int status)
{
	adxl34x_send_key_events(ac, pdata, status, true);
	input_sync(ac->input);
	adxl34x_send_key_events(ac, pdata, status, false);
}

static irqreturn_t adxl345_irq(int irq, void *handle)
{
	struct adxl34x *ac = handle;
	struct adxl34x_platform_data *pdata = &ac->pdata;
	int int_stat, tap_stat, samples, orient, orient_code;

	/*
	 * ACT_TAP_STATUS should be read before clearing the interrupt
	 * Avoid reading ACT_TAP_STATUS in case TAP detection is disabled
	 */

	if (pdata->tap_axis_control & (TAP_X_EN | TAP_Y_EN | TAP_Z_EN))
		tap_stat = AC_READ(ac, ACT_TAP_STATUS);
	else
		tap_stat = 0;

	int_stat = AC_READ(ac, INT_SOURCE);

	if (int_stat & FREE_FALL)
		adxl34x_report_key_single(ac->input, pdata->ev_code_ff);

	if (int_stat & OVERRUN)
		dev_dbg(ac->dev, "OVERRUN\n");

	if (int_stat & (SINGLE_TAP | DOUBLE_TAP)) {
		adxl34x_do_tap(ac, pdata, tap_stat);

		if (int_stat & DOUBLE_TAP)
			adxl34x_do_tap(ac, pdata, tap_stat);
	}

	if (pdata->ev_code_act_inactivity) {
		if (int_stat & ACTIVITY)
			input_report_key(ac->input,
					 pdata->ev_code_act_inactivity, 1);
		if (int_stat & INACTIVITY)
			input_report_key(ac->input,
					 pdata->ev_code_act_inactivity, 0);
	}

	/*
	 * ORIENTATION SENSING ADXL346 only
	 */
	if (pdata->orientation_enable) {
		orient = AC_READ(ac, ORIENT);
		if ((pdata->orientation_enable & ADXL_EN_ORIENTATION_2D) &&
		    (orient & ADXL346_2D_VALID)) {

			orient_code = ADXL346_2D_ORIENT(orient);
			/* Report orientation only when it changes */
			if (ac->orient2d_saved != orient_code) {
				ac->orient2d_saved = orient_code;
				adxl34x_report_key_single(ac->input,
					pdata->ev_codes_orient_2d[orient_code]);
			}
		}

		if ((pdata->orientation_enable & ADXL_EN_ORIENTATION_3D) &&
		    (orient & ADXL346_3D_VALID)) {

			orient_code = ADXL346_3D_ORIENT(orient) - 1;
			/* Report orientation only when it changes */
			if (ac->orient3d_saved != orient_code) {
				ac->orient3d_saved = orient_code;
				adxl34x_report_key_single(ac->input,
					pdata->ev_codes_orient_3d[orient_code]);
			}
		}
	}

	if (int_stat & (DATA_READY | WATERMARK)) {

		if (pdata->fifo_mode)
			samples = ENTRIES(AC_READ(ac, FIFO_STATUS)) + 1;
		else
			samples = 1;

		for (; samples > 0; samples--) {
			adxl34x_service_ev_fifo(ac);
			/*
			 * To ensure that the FIFO has
			 * completely popped, there must be at least 5 us between
			 * the end of reading the data registers, signified by the
			 * transition to register 0x38 from 0x37 or the CS pin
			 * going high, and the start of new reads of the FIFO or
			 * reading the FIFO_STATUS register. For SPI operation at
			 * 1.5 MHz or lower, the register addressing portion of the
			 * transmission is sufficient delay to ensure the FIFO has
			 * completely popped. It is necessary for SPI operation
			 * greater than 1.5 MHz to de-assert the CS pin to ensure a
			 * total of 5 us, which is at most 3.4 us at 5 MHz
			 * operation.
			 */
			if (ac->fifo_delay && (samples > 1))
				udelay(3);
		}
	}

	input_sync(ac->input);

	return IRQ_HANDLED;
}




/* 使能系统 */
static void __adxl345_disable(struct adxl34x *adxl)
{
	/*
	 * A '0' places the ADXL34x into standby mode
	 * with minimum power consumption.
	 */
	AC_WRITE(adxl, POWER_CTL, 0);
}

static void __adxl345_enable(struct adxl34x *adxl)
{
	AC_WRITE(adxl, POWER_CTL, adxl->pdata.power_mode | PCTL_MEASURE);  /* POWER_CTL bit3 4 5  */
}





/* 文件系统 */

static ssize_t adxl34x_disable_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ac->disabled);
}

static ssize_t adxl34x_disable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	unsigned int val;
	int error;

	error = kstrtouint(buf, 10, &val);
	if (error)
		return error;

	mutex_lock(&ac->mutex);

	if (!ac->suspended && ac->opened) {
		if (val) {
			if (!ac->disabled)
				__adxl345_disable(ac);
		} else {
			if (ac->disabled)
				__adxl345_enable(ac);
		}
	}

	ac->disabled = !!val;

	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(disable, 0664, adxl34x_disable_show, adxl34x_disable_store);

static ssize_t adxl34x_calibrate_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	ssize_t count;

	mutex_lock(&ac->mutex);
	count = sprintf(buf, "%d,%d,%d\n",
			ac->hwcal.x * 4 + ac->swcal.x,
			ac->hwcal.y * 4 + ac->swcal.y,
			ac->hwcal.z * 4 + ac->swcal.z);
	mutex_unlock(&ac->mutex);

	return count;
}

static ssize_t adxl34x_calibrate_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);

	/*
	 * Hardware offset calibration has a resolution of 15.6 mg/LSB.
	 * We use HW calibration and handle the remaining bits in SW. (4mg/LSB)
	 */

	mutex_lock(&ac->mutex);
	ac->hwcal.x -= (ac->saved.x / 4);
	ac->swcal.x = ac->saved.x % 4;

	ac->hwcal.y -= (ac->saved.y / 4);
	ac->swcal.y = ac->saved.y % 4;

	ac->hwcal.z -= (ac->saved.z / 4);
	ac->swcal.z = ac->saved.z % 4;

	AC_WRITE(ac, OFSX, (s8) ac->hwcal.x);
	AC_WRITE(ac, OFSY, (s8) ac->hwcal.y);
	AC_WRITE(ac, OFSZ, (s8) ac->hwcal.z);
	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(calibrate, 0664,
		   adxl34x_calibrate_show, adxl34x_calibrate_store);

static ssize_t adxl34x_rate_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", RATE(ac->pdata.data_rate));
}

static ssize_t adxl34x_rate_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	unsigned char val;
	int error;

	error = kstrtou8(buf, 10, &val);
	if (error)
		return error;

	mutex_lock(&ac->mutex);

	ac->pdata.data_rate = RATE(val);
	AC_WRITE(ac, BW_RATE,
		 ac->pdata.data_rate |
			(ac->pdata.low_power_mode ? LOW_POWER : 0));

	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(rate, 0664, adxl34x_rate_show, adxl34x_rate_store);

static ssize_t adxl34x_autosleep_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n",
		ac->pdata.power_mode & (PCTL_AUTO_SLEEP | PCTL_LINK) ? 1 : 0);
}

static ssize_t adxl34x_autosleep_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	unsigned int val;
	int error;

	error = kstrtouint(buf, 10, &val);
	if (error)
		return error;

	mutex_lock(&ac->mutex);

	if (val)
		ac->pdata.power_mode |= (PCTL_AUTO_SLEEP | PCTL_LINK);
	else
		ac->pdata.power_mode &= ~(PCTL_AUTO_SLEEP | PCTL_LINK);

	if (!ac->disabled && !ac->suspended && ac->opened)
		AC_WRITE(ac, POWER_CTL, ac->pdata.power_mode | PCTL_MEASURE);

	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(autosleep, 0664,
		   adxl34x_autosleep_show, adxl34x_autosleep_store);

static ssize_t adxl34x_position_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	ssize_t count;

	mutex_lock(&ac->mutex);
	count = sprintf(buf, "(%d, %d, %d)\n",
			ac->saved.x, ac->saved.y, ac->saved.z);
	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(position, S_IRUGO, adxl34x_position_show, NULL);

#ifdef ADXL_DEBUG
static ssize_t adxl34x_write_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	unsigned int val;
	int error;

	/*
	 * This allows basic ADXL register write access for debug purposes.
	 */
	error = kstrtouint(buf, 16, &val);
	if (error)
		return error;

	mutex_lock(&ac->mutex);
	AC_WRITE(ac, val >> 8, val & 0xFF);
	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(write, 0664, NULL, adxl34x_write_store);
#endif

static struct attribute *adxl34x_attributes[] = {
	&dev_attr_disable.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_rate.attr,
	&dev_attr_autosleep.attr,
	&dev_attr_position.attr,
#ifdef ADXL_DEBUG
	&dev_attr_write.attr,
#endif
	NULL
};

static const struct attribute_group adxl34x_attr_group = {
	.attrs = adxl34x_attributes,
};








/* input开关系统 */
static int adxl345_input_open(struct input_dev *input)
{
	struct adxl34x *adxl = input_get_drvdata(input);

	mutex_lock(&adxl->mutex);

	if (!adxl->suspended && !adxl->disabled)
		__adxl345_enable(adxl);

	adxl->opened = true;

	mutex_unlock(&adxl->mutex);

	return 0;
}

static void adxl345_input_close(struct input_dev *input)
{
	struct adxl34x *adxl = input_get_drvdata(input);

	mutex_lock(&adxl->mutex);

	if (!adxl->suspended && !adxl->disabled)
		__adxl345_disable(adxl);

	adxl->opened = false;

	mutex_unlock(&adxl->mutex);
}


/* probe函数 */
static struct adxl34x *adxl345_probe(struct device *dev, int irq, bool fifo_delay_default, const struct adxl34x_bus_ops *bops)
{
	struct adxl34x *adxl;
	struct input_dev *input_dev;
	const struct adxl34x_platform_data *pdata;
	unsigned char revid;
	int err, range;


	
	if (!irq) {
		dev_err(dev, "no IRQ?\n");
		err = -ENODEV;
		goto err_out;	
	}

	adxl = kzalloc(sizeof(*adxl), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!adxl || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	adxl->fifo_delay = fifo_delay_default;
 
 	/* 如果device没有提供设备信息则使用驱动提供的默认信息 */ 
	pdata = dev_get_platdata(dev); 
	if (!pdata) {
		dev_dbg(dev, "No platform data: Using default initialization\n");
		pdata = &adxl345_default_init;
	}

	adxl->pdata = *pdata;
	pdata = &adxl->pdata;

	adxl->input = input_dev;
	adxl->dev   = dev;
	adxl->irq	= irq;
	adxl->bops  = bops;

	mutex_init(&adxl->mutex);

	input_dev->name = "ADXL345 accelerometer";
	
	revid = AC_READ(adxl, DEVID);
    printk("revid %d\n", revid);

	adxl->model = 345;
	/*switch (revid) {
	case ID_ADXL345:
		adxl->model = 345;
		break;
	default:
		dev_err(dev, "Failed to probe %s\n", input_dev->name);
		err = -ENODEV;
		goto err_free_mem;
	}*/
	
	snprintf(adxl->phys, sizeof(adxl->phys), "%s/input0", dev_name(dev));

	
	input_dev->phys = adxl->phys;
	input_dev->dev.parent = dev;
	input_dev->id.product = adxl->model;
	input_dev->id.bustype = bops->bustype;
	input_dev->open = adxl345_input_open;
	input_dev->close = adxl345_input_close;

	input_set_drvdata(input_dev, adxl);

	__set_bit(adxl->pdata.ev_type, input_dev->evbit);

	/* EV_ABS */
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_Z, input_dev->absbit);

	if (pdata->data_range & FULL_RES)
		range = ADXL_FULLRES_MAX_VAL;	/* Signed 13-bit */
	else
		range = ADXL_FIXEDRES_MAX_VAL;	/* Signed 10-bit */

	input_set_abs_params(input_dev, ABS_X, -range, range, 3, 3);
	input_set_abs_params(input_dev, ABS_Y, -range, range, 3, 3);
	input_set_abs_params(input_dev, ABS_Z, -range, range, 3, 3);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(pdata->ev_code_tap[ADXL_X_AXIS], input_dev->keybit);
	__set_bit(pdata->ev_code_tap[ADXL_Y_AXIS], input_dev->keybit);
	__set_bit(pdata->ev_code_tap[ADXL_Z_AXIS], input_dev->keybit);

	if (pdata->ev_code_ff) {
		adxl->int_mask = FREE_FALL;
		__set_bit(pdata->ev_code_ff, input_dev->keybit);
	}
	
	if (pdata->ev_code_act_inactivity)
		__set_bit(pdata->ev_code_act_inactivity, input_dev->keybit);

	adxl->int_mask |= ACTIVITY | INACTIVITY;

	if (pdata->watermark) {
		adxl->int_mask |= WATERMARK;
		if (!FIFO_MODE(pdata->fifo_mode))
			adxl->pdata.fifo_mode |= FIFO_STREAM;
	} else {
		adxl->int_mask |= DATA_READY;
	}

	if (pdata->tap_axis_control & (TAP_X_EN | TAP_Y_EN | TAP_Z_EN))
		adxl->int_mask |= SINGLE_TAP | DOUBLE_TAP;

	if (FIFO_MODE(pdata->fifo_mode) == FIFO_BYPASS)
		adxl->fifo_delay = false;

	AC_WRITE(adxl, POWER_CTL, 0);
	
	/* 申请中断 */
	dump_stack();

	err = request_threaded_irq(adxl->irq, NULL, adxl345_irq, IRQF_TRIGGER_HIGH | IRQF_ONESHOT, dev_name(dev), adxl);
	if (err) {
	   dev_err(dev, "irq %d busy?\n", adxl->irq);
	   goto err_free_mem;
	}
	
	/* 在sys目录下创建相应节点 */
	err = sysfs_create_group(&dev->kobj, &adxl34x_attr_group);
	if (err)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr;

	/* 位移偏移类 */
	AC_WRITE(adxl, OFSX, pdata->x_axis_offset);
	adxl->hwcal.x = pdata->x_axis_offset;
	AC_WRITE(adxl, OFSY, pdata->y_axis_offset);
	adxl->hwcal.y = pdata->y_axis_offset;
	AC_WRITE(adxl, OFSZ, pdata->z_axis_offset);
	adxl->hwcal.z = pdata->z_axis_offset;

	/* 敲击中断与时间类 */
	AC_WRITE(adxl, THRESH_TAP, pdata->tap_threshold); /* 敲击中断为 2.1875g */
	AC_WRITE(adxl, DUR, pdata->tap_duration); /* 最大时间值 1875μs */
	AC_WRITE(adxl, LATENT, pdata->tap_latency);	/* 等待时间25ms */
	AC_WRITE(adxl, WINDOW, pdata->tap_window);	/* 延迟时间25ms */

	/* 活动与时间类 */
	AC_WRITE(adxl, THRESH_ACT, pdata->activity_threshold);	/* 保存检测活动的阈值 0.375g */
	AC_WRITE(adxl, THRESH_INACT, pdata->inactivity_threshold);	/* 保存检测静止的阈值 0.25g */
	AC_WRITE(adxl, TIME_INACT, pdata->inactivity_time);	/* 加速度时间量小于3sec(秒)则静止 */
	AC_WRITE(adxl, THRESH_FF, pdata->free_fall_threshold); /* 自由落体检测0.5g */
	AC_WRITE(adxl, TIME_FF, pdata->free_fall_time);	/* 小于THRESH_FF的最小时间160ms */
	AC_WRITE(adxl, TAP_AXES, pdata->tap_axis_control);	/* 使能三轴敲击检测 */

	AC_WRITE(adxl, ACT_INACT_CTL, pdata->act_axis_control);	/* 交流耦合及使能操作 */
	AC_WRITE(adxl, BW_RATE, RATE(adxl->pdata.data_rate) | (pdata->low_power_mode ? LOW_POWER : 0));	/* 正常功耗 12.5HZ输出速率 */

	AC_WRITE(adxl, DATA_FORMAT, pdata->data_range);	/* 全分辨率模式 */	
	AC_WRITE(adxl, FIFO_CTL, FIFO_MODE(pdata->fifo_mode) | SAMPLES(pdata->watermark));	/* 流模式 保存最后32个数据 */

	/* Map all INTs to INT1 */
	AC_WRITE(adxl, INT_MAP, 0);	/* 设置中断信号接收引脚INT1 */
	adxl->pdata.orientation_enable = 0;

	AC_WRITE(adxl, INT_ENABLE, adxl->int_mask | OVERRUN);	/* 使能相应功能 */

	adxl->pdata.power_mode &= (PCTL_AUTO_SLEEP | PCTL_LINK);

	return adxl;
		

err_remove_attr:
   sysfs_remove_group(&dev->kobj, &adxl34x_attr_group);
err_free_irq:
   free_irq(adxl->irq, adxl);
err_free_mem:
   input_free_device(input_dev);
   kfree(adxl);
err_out:
   return ERR_PTR(err);

}

static int adxl345_spi_probe(struct spi_device *spi)
{
	int ret;
	struct adxl34x *adxl;
		
	apxl345_spi = spi;

	/* 申请中断 */		
	adxl345 = kzalloc(sizeof(struct adxl345) * 1, GFP_KERNEL);
	if(!adxl345)
	{
		printk("kzalloc is err\n");
		return -ENODEV;
	}
	
	adxl345->apxl_int1_gpio = gpiod_get(&spi->dev, "int1", GPIOD_IN);
	if (IS_ERR(adxl345->apxl_int1_gpio)) 
	{
		printk("apxl_int1_gpio Failed to get GPIO interrupt\n");
		return -1;
	}
	
	gpiod_direction_input(adxl345->apxl_int1_gpio);
	spi->irq = gpiod_to_irq(adxl345->apxl_int1_gpio);
	if (spi->irq < 0) {
		printk("failed to get adxl345 irq\n");
		return spi->irq;
	}
	
    printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);

	/* 进行设备信息初始化 */
	adxl = adxl345_probe(&spi->dev, spi->irq, spi->max_speed_hz > MAX_FREQ_NO_FIFODELAY, &adxl345_spi_bops);
	if (IS_ERR(adxl))
		return PTR_ERR(adxl);
	
	spi_set_drvdata(spi, adxl);
    printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);

	return 0;
	
}







/* remove函数 */
static int adxl345_remove(struct adxl34x *adxl)
{
	sysfs_remove_group(&adxl->dev->kobj, &adxl34x_attr_group);
	free_irq(adxl->irq, adxl);
	input_unregister_device(adxl->input);
	dev_dbg(adxl->dev, "unregistered accelerometer\n");
	kfree(adxl);

	return 0;
}

static int adxl345_spi_remove(struct spi_device *spi)
{
	int ret;
	struct adxl34x *adxl = spi_get_drvdata(spi);

	ret = adxl345_remove(adxl);
	
	gpiod_put(adxl345->apxl_int1_gpio);
	kfree(adxl345);
	return 0;
}

/* 电源管理系统 */
static void adxl345_suspend(struct adxl34x *adxl)
{
	mutex_lock(&adxl->mutex);

	if (!adxl->suspended && !adxl->disabled && adxl->opened)
		__adxl345_disable(adxl);

	adxl->suspended = true;

	mutex_unlock(&adxl->mutex);
}

static void adxl345_resume(struct adxl34x *adxl)
{
	mutex_lock(&adxl->mutex);

	if (adxl->suspended && !adxl->disabled && adxl->opened)
		__adxl345_enable(adxl);

	adxl->suspended = false;

	mutex_unlock(&adxl->mutex);
}



static int __maybe_unused adxl345_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct adxl34x *adxl = spi_get_drvdata(spi);

	adxl345_suspend(adxl);

	return 0;
}

static int __maybe_unused adxl345_spi_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct adxl34x *adxl = spi_get_drvdata(spi);

	adxl345_resume(adxl);

	return 0;
}

static SIMPLE_DEV_PM_OPS(adxl345_spi_pm, adxl345_spi_suspend,
			 adxl345_spi_resume);



/* spi_driver结构体 */
static const struct of_device_id adxl345_dt_ids[] = {
	{ .compatible = "spiadxl345" },
	{ /* sentinel */ }
};

static struct spi_driver adxl345_spi_driver = {
	.driver = {
		.name = "spiadxl345",
		.owner = THIS_MODULE,
		.of_match_table = adxl345_dt_ids,		
		.pm = &adxl345_spi_pm,			
	},
	.probe = adxl345_spi_probe,
	.remove = adxl345_spi_remove
};



/* 出入口函数 */
static int __init adxl345_init(void)
{
	int ret;
	major = register_chrdev(0, "adxl345major", &apxl345ops);
	if (major < 0) 
	{
		printk(": could not get major number\n");
		return major;
	}
	
	apxl345class = class_create(THIS_MODULE, "apxl345class");
	if (IS_ERR(apxl345class)) 
	{
		ret = PTR_ERR(apxl345class);
		printk(": could not get mx1508_class\n");		
		return ret;
	}

	device_create(apxl345class, NULL, MKDEV(major, 0),  NULL, "lc_adxl345");
	return spi_register_driver(&adxl345_spi_driver);
}

static void __exit adxl345_exit(void)
{
	spi_unregister_driver(&adxl345_spi_driver);
	
	device_destroy(apxl345class, MKDEV(major, 0));
	class_destroy(apxl345class);
	unregister_chrdev(major, "adxl345major");
}


module_init(adxl345_init);
module_exit(adxl345_exit);

MODULE_LICENSE("GPL");







