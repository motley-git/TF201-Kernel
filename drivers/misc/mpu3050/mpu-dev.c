/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/signal.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/poll.h>

#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/workqueue.h>

#include "mpuirq.h"
#include "slaveirq.h"
#include "mlsl.h"
#include "mpu-i2c.h"
#include "mldl_cfg.h"
#include "mpu.h"

extern unsigned int factory_mode;
extern bool flagLoadConfig;
extern bool flagLoadAccelConfig;
struct delayed_work mpu_init_work;

/*enum ST_Result: defined for enable_self_test() function*/
enum ST_Result{
	Success,		/*Test Success*/
	InspectFault,		/*Enter SELF-TEST mode success, but fail to read data*/
	RegValFault,		/*Enter SELF-TEST mode fail*/
	Default };		/*Deafault Status*/

/* Platform data for the MPU */
struct mpu_private_data {
	struct miscdevice dev;
	struct i2c_client *client;
	struct mldl_cfg mldl_cfg;

	struct mutex mutex;
	wait_queue_head_t mpu_event_wait;
	struct completion completion;
	struct timer_list timeout;
	struct notifier_block nb;
	struct mpuirq_data mpu_pm_event;
	int response_timeout;	/* In seconds */
	unsigned long event;
	struct attribute_group attrs;
	int gyro_status;
	int accel_status;
	int compass_status;
	int pid;
};

struct i2c_client *this_client;

static int mpu_delay_init(void);

static ssize_t enable_load_accel_config(struct device *dev, struct device_attribute *devattr, char *buf)
{       struct i2c_client *client = to_i2c_client(dev);
        struct mpu_private_data *data = i2c_get_clientdata(client);
        //flagLoadAccelConfig = false;
        return sprintf(buf, "%d\n", flagLoadAccelConfig);
}

static ssize_t enable_load_mag_config(struct device *dev, struct device_attribute *devattr, char *buf)
{       struct i2c_client *client = to_i2c_client(dev);
        struct mpu_private_data *data = i2c_get_clientdata(client);
        flagLoadConfig = false;
        return sprintf(buf, "%d\n", flagLoadConfig);
}

static ssize_t read_compass_status(struct device *dev, struct device_attribute *devattr, char *buf)
{	struct i2c_client *client = to_i2c_client(dev);
	struct mpu_private_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", data->compass_status);
}

static ssize_t read_accel_status(struct device *dev, struct device_attribute *devattr, char *buf)
{	struct i2c_client *client = to_i2c_client(dev);
	struct mpu_private_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", data->accel_status);
}

static ssize_t read_gyro_status(struct device *dev, struct device_attribute *devattr, char *buf)
{	struct i2c_client *client = to_i2c_client(dev);
	struct mpu_private_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", data->gyro_status);
}

static ssize_t read_accel_raw(struct device *dev, struct device_attribute *devattr, char *buf)
{	struct i2c_client *client = to_i2c_client(dev);
	struct mpu_private_data *mpu = i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	unsigned char data[6];
	int X = 0, Y = 0, Z = 0;
	int res = 0;
	int project = 101;

	if(mldl_cfg->accel ){
		res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address,
				0x0F, 1, data);
		printk("KXTF9 WHO AM I: %d\n",data[0]);

		res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address,
				0x1B, 1, data);
		printk("KXTF9 CTRL_REG1: 0x%02lx\n",data[0]);
		res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address,
				0x1E, 1, data);
		printk("KXTF9 INT_CTRL_REG1: 0x%02lx\n",data[0]);
		if ((data[0] & 0x80 ) == 0)
			{
				printk("KXTF9 is standing by, change to OP \n");
				data[0] = 0xE0;
				inv_serial_single_write(client->adapter, mldl_cfg->pdata->accel.address,0x1B, data[0]);
			}

		res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address,
				0x06, 6, data);
		printk("KXTF9 raw data 1~6: %d, %d, %d, %d, %d, %d\n",data[0],data[1],data[2],data[3],data[4],data[5]);
		
		if(res)
			printk("%s: Read accel data register fail\n", __FUNCTION__);
		else{
			    X = ((data[1] << 4) | (data[0] >> 4));
				Y = ((data[3] << 4) | (data[2] >> 4));
				Z = ((data[5] << 4) | (data[4] >> 4));

				if (X & 0x800)
					X |= 0xFFFFF000;
				if (Y & 0x800)
					Y |= 0xFFFFF000;
				if (Z & 0x800)
					Z |= 0xFFFFF000;

				if (project == 101){
					printk("%s: ep101\n", __func__);
					X = X*(-1);
					Y = Y*(1);
					Z = Z*(1);
				}
		}
	}

	return sprintf(buf, "%d %d %d\n", X, Y, Z);
}

static ssize_t read_compass_raw(struct device *dev, struct device_attribute *devattr, char *buf)
{	struct i2c_client *client = to_i2c_client(dev);
	struct mpu_private_data *mpu = i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	unsigned char data[6];
	int res = 0;
	printk("%s+\n", __func__);
	if(mldl_cfg->compass){
		printk("%s: have compass\n", __func__);
		inv_serial_single_write(client->adapter, mldl_cfg->pdata->compass.address,
				0x1B, 0x92);
				inv_serial_single_write(client->adapter, mldl_cfg->pdata->compass.address,
				0x1D, 0x40);

		res = inv_serial_read(client->adapter, mldl_cfg->pdata->compass.address,
					0x10, 6, data);
		if(res){
			printk("%s: Read compass register fail\n", __FUNCTION__);
			return sprintf(buf, "0 0 0 0 0 0\n");
		}
		else{
			printk("%x %x %x %x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5]);
			return sprintf(buf, "%x %x %x %x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5]);
		}

	}else {
		printk("%s: no compass\n", __func__);
		return sprintf(buf, "0 0 0 0 0 0\n");
	}
}

static unsigned char Self_Test_Inspect(int inputX, int inputY, int inputZ, int Size)
{
	const int Self_Test_Spec[8] = {-1064, -736,    //  X-spec
	                                 636,  964,    //  Y-spec
					-464, -136,    //  Z-spec
					1536, 1864  }; // -Z-spec
	unsigned char ret = Default;
	int avgX=0, avgY=0, avgZ=0;

	if(Size)
	{
		avgX = (int)(inputX/Size);
		avgY = (int)(inputY/Size);
		avgZ = (int)(inputZ/Size);

		if((avgX > Self_Test_Spec[0]) && (avgX < Self_Test_Spec[1]) &&
		   (avgY > Self_Test_Spec[2]) && (avgY < Self_Test_Spec[3]))
		{
			if((avgZ > Self_Test_Spec[4]) && (avgZ < Self_Test_Spec[5]))
				ret = Success;
			else if((avgZ > Self_Test_Spec[6]) && (avgZ < Self_Test_Spec[7]))
				ret = Success;
			else
				ret = InspectFault;
		}
	}

	return ret;
}

static ssize_t enable_self_test(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mpu_private_data *mpu = i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	unsigned char data[6];
	int X = 0, Y = 0, Z = 0, i=0;
	char localbuf[50];
	int bufsize = 20;
	int localdatabuf[60]={0};
	int bufcount=0;
	int res = 0;
	int SumX=0, SumY=0, SumZ=0;
	unsigned char result = Default;
	unsigned char RegVal = 0x00;
	short WriteRegcnt = 0;

	if(mldl_cfg->accel){

		// Write 0xCA to address 0x3A continuously,
		// to prevent the case if SELF-TEST register is not ready sometimes.
		printk("KXTF9 is changing to Self-Test Mode from Normal Mode.\n");
		while((RegVal != 0xCA) && (WriteRegcnt < 5)){
			inv_serial_single_write(client->adapter, mldl_cfg->pdata->accel.address, 0x3A, 0xCA);

			// Read the status of Self-Test Register.
			res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address,
					0x3A, 1, data);
			printk("KXTF9 Self-Test Reg: 0x%02lx\n", data[0]);

			mdelay(100);
			RegVal = data[0];
			WriteRegcnt++;
		}

		if(WriteRegcnt >= 5)
			result = RegValFault;
		else
		{
			printk("KXTF9 Self-Test Start.\n");

			 // long delay, for filtering those data which not read in Self-test Mode.
			mdelay(250);

			for(i=0; i<bufsize; i++)
			{
				res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address, 0x06, 6, data);

				if(res)
					printk("%s: Read accel data register fail\n", __FUNCTION__);
				else{
					X = ((data[1] << 4) | (data[0] >> 4));
					Y = ((data[3] << 4) | (data[2] >> 4));
					Z = ((data[5] << 4) | (data[4] >> 4));

					if (X & 0x800)
						X |= 0xFFFFF000;
					if (Y & 0x800)
						Y |= 0xFFFFF000;
					if (Z & 0x800)
						Z |= 0xFFFFF000;

					X = X*(-1);

					localdatabuf[i*3+0] = X;
					localdatabuf[i*3+1] = Y;
					localdatabuf[i*3+2] = Z;
				}

				// print data content for debug.
				printk("%d %d %d\n", X, Y, Z);

				SumX+=X;
				SumY+=Y;
				SumZ+=Z;

				mdelay(50);
			}

			printk("SunX=%d, SumY=%d, SumZ=%d\n", SumX, SumY, SumZ);

			// inspect the self-test data between the custom spec.
			// if result = 0, match the spec.
			// if result = 1, DO NOT match the spec.
			result = Self_Test_Inspect(SumX, SumY, SumZ, bufsize);
		}
	}

	if(result == Success)
		return sprintf(buf, "accel Self-Test PASS!\n");
	else if(result == RegValFault)
		return sprintf(buf, "accel Self-Test FAIL!\nFail to write 0xCA into Self-Test Register.\n");
	else{
		bufcount += sprintf(localbuf, "accel Self-Test FAIL!\nData content shows as below:\n\n");
		strcat(buf, localbuf);

		for(i=0; i<bufsize; i++){
			bufcount += sprintf(localbuf, "%d %d %d\n", localdatabuf[i*3+0], localdatabuf[i*3+1], localdatabuf[i*3+2]);
			strcat(buf, localbuf);
		}

		return bufcount;
	}
}

static ssize_t disable_self_test(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mpu_private_data *mpu = i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	unsigned char data[6];
	int res = 0;

	// Write 0x00 to Self-Test register (0x3A), to change it into NORMAL mode.
	printk("KXTF9 is changing to Normal Mode from Self-Test.\n");
	inv_serial_single_write(client->adapter, mldl_cfg->pdata->accel.address,
				0x3A, 0x00);

	res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address,
				0x3A, 1, data);
	if(!data[0])
		printk("KXTF9 Self-Test Reg: 0x%02lx, Change to Normal Mode Successfully.\n", data[0]);
	else
		printk("KXTF9 Self-Test Reg: 0x%02lx, Change to Normal Mode FAIL.\n", data[0]);

	// Read relevant register status for further usage.
	res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address,
				0x3A, 1, data);
	printk("KXTF9 Self-Test Reg: 0x%02lx\n",data[0]);

	res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address,
				0x0C, 1, data);
	printk("KXTF9 DCST_RESP Reg: 0x%02lx\n",data[0]);

	res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address,
				0x1D, 1, data);
	printk("KXTF9 CTRL_REG3 Reg: 0x%02lx\n",data[0]);

	res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address,
				0x1B, 1, data);
	printk("KXTF9 CTRL_REG1 Reg: 0x%02lx\n",data[0]);

	return sprintf(buf, "Disable Self-Test Mode.\n");
}

DEVICE_ATTR(compass_raw, S_IRUGO, read_compass_raw, NULL);
DEVICE_ATTR(accel_raw, S_IRUGO, read_accel_raw, NULL);
DEVICE_ATTR(compass_status, S_IRUGO, read_compass_status, NULL);
DEVICE_ATTR(accel_status, S_IRUGO, read_accel_status, NULL);
DEVICE_ATTR(gyro_status, S_IRUGO, read_gyro_status, NULL);
DEVICE_ATTR(enLoadMagConfig, S_IRUGO, enable_load_mag_config, NULL);
DEVICE_ATTR(enLoadAccelConfig, S_IRUGO, enable_load_accel_config, NULL);
DEVICE_ATTR(enSelfTest, S_IRUGO, enable_self_test, NULL);
DEVICE_ATTR(disSelfTest, S_IRUGO, disable_self_test, NULL);

static struct attribute *mpu_3050_attr[] = {
	&dev_attr_gyro_status.attr,
	&dev_attr_accel_status.attr,
	&dev_attr_compass_status.attr,
	&dev_attr_accel_raw.attr,
	&dev_attr_compass_raw.attr,
	&dev_attr_enLoadMagConfig,
	&dev_attr_enLoadAccelConfig,
	&dev_attr_enSelfTest.attr,
	&dev_attr_disSelfTest.attr,
	NULL
};

static void mpu_pm_timeout(u_long data)
{
	struct mpu_private_data *mpu = (struct mpu_private_data *)data;
	struct i2c_client *client = mpu->client;
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	complete(&mpu->completion);
}

static int mpu_pm_notifier_callback(struct notifier_block *nb,
				    unsigned long event, void *unused)
{
	struct mpu_private_data *mpu =
	    container_of(nb, struct mpu_private_data, nb);
	struct i2c_client *client = mpu->client;
	struct timeval event_time;
	dev_dbg(&client->adapter->dev, "%s: %ld\n", __func__, event);

	/* Prevent the file handle from being closed before we initialize
	   the completion event */
	mutex_lock(&mpu->mutex);
	if (!(mpu->pid) ||
	    (event != PM_SUSPEND_PREPARE && event != PM_POST_SUSPEND)) {
		mutex_unlock(&mpu->mutex);
		return NOTIFY_OK;
	}

	if (event == PM_SUSPEND_PREPARE)
		mpu->event = MPU_PM_EVENT_SUSPEND_PREPARE;
	if (event == PM_POST_SUSPEND)
		mpu->event = MPU_PM_EVENT_POST_SUSPEND;

	do_gettimeofday(&event_time);
	mpu->mpu_pm_event.interruptcount++;
	mpu->mpu_pm_event.irqtime =
	    (((long long)event_time.tv_sec) << 32) + event_time.tv_usec;
	mpu->mpu_pm_event.data_type = MPUIRQ_DATA_TYPE_PM_EVENT;
	mpu->mpu_pm_event.data = mpu->event;


	if (mpu->response_timeout > 0) {
		mpu->timeout.expires = jiffies + mpu->response_timeout * HZ;
		add_timer(&mpu->timeout);
	}
	INIT_COMPLETION(mpu->completion);
	mutex_unlock(&mpu->mutex);

	wake_up_interruptible(&mpu->mpu_event_wait);
	wait_for_completion(&mpu->completion);
	del_timer_sync(&mpu->timeout);
	dev_dbg(&client->adapter->dev, "%s: %ld DONE\n", __func__, event);
	return NOTIFY_OK;
}

static int mpu_dev_open(struct inode *inode, struct file *file)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	int result;
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	dev_dbg(&client->adapter->dev, "current->pid %d\n", current->pid);
	mpu->pid = current->pid;
	/* we could do some checking on the flags supplied by "open" */
	/* i.e. O_NONBLOCK */
	/* -> set some flag to disable interruptible_sleep_on in mpu_read */

	/* Reset the sensors to the default */
	result = mutex_lock_interruptible(&mpu->mutex);
	if (result) {
		dev_err(&client->adapter->dev,
			"%s: mutex_lock_interruptible returned %d\n",
			__func__, result);
		return result;
	}
	mldl_cfg->requested_sensors = INV_THREE_AXIS_GYRO;
	if (mldl_cfg->accel && mldl_cfg->accel->resume)
		mldl_cfg->requested_sensors |= INV_THREE_AXIS_ACCEL;

	if (mldl_cfg->compass && mldl_cfg->compass->resume)
		mldl_cfg->requested_sensors |= INV_THREE_AXIS_COMPASS;

	if (mldl_cfg->pressure && mldl_cfg->pressure->resume)
		mldl_cfg->requested_sensors |= INV_THREE_AXIS_PRESSURE;
	mutex_unlock(&mpu->mutex);
	return 0;
}

/* close function - called when the "file" /dev/mpu is closed in userspace   */
static int mpu_release(struct inode *inode, struct file *file)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;
	int result = 0;

	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	mutex_lock(&mpu->mutex);
	mldl_cfg->requested_sensors = 0;
	result = inv_mpu_suspend(mldl_cfg, client->adapter,
				 accel_adapter, compass_adapter,
				 pressure_adapter, INV_ALL_SENSORS);
	mpu->pid = 0;
	mutex_unlock(&mpu->mutex);
	complete(&mpu->completion);
	dev_dbg(&client->adapter->dev, "mpu_release\n");
	return result;
}

/* read function called when from /dev/mpu is read.  Read from the FIFO */
static ssize_t mpu_read(struct file *file,
			char __user *buf, size_t count, loff_t *offset)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	size_t len = sizeof(mpu->mpu_pm_event) ;//+ sizeof(unsigned long);
	int err;

	if (!mpu->event && (!(file->f_flags & O_NONBLOCK)))
		wait_event_interruptible(mpu->mpu_event_wait, mpu->event);

	if (!mpu->event || NULL == buf
	    || count < sizeof(mpu->mpu_pm_event) )//+ sizeof(unsigned long))
		return 0;

	err = copy_to_user(buf, &mpu->mpu_pm_event, sizeof(mpu->mpu_pm_event));
	if (err != 0) {
		dev_err(&client->adapter->dev,
			"Copy to user returned %d\n", err);
		return -EFAULT;
	}
	mpu->event = 0;
	return len;
}

static unsigned int mpu_poll(struct file *file, struct poll_table_struct *poll)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	int mask = 0;

	poll_wait(file, &mpu->mpu_event_wait, poll);
	if (mpu->event)
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

static int
mpu_dev_ioctl_set_mpu_pdata(struct i2c_client *client, unsigned long arg)
{
	int ii;
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mpu_platform_data *pdata = mpu->mldl_cfg.pdata;
	struct mpu_platform_data local_pdata;

	if (copy_from_user(&local_pdata, (unsigned char __user *)arg,
			   sizeof(local_pdata)))
		return -EFAULT;

	pdata->int_config = local_pdata.int_config;
	for (ii = 0; ii < DIM(pdata->orientation); ii++)
		pdata->orientation[ii] = local_pdata.orientation[ii];
	pdata->level_shifter = local_pdata.level_shifter;

	pdata->accel.address = local_pdata.accel.address;
	for (ii = 0; ii < DIM(pdata->accel.orientation); ii++)
		pdata->accel.orientation[ii] =
		    local_pdata.accel.orientation[ii];

	pdata->compass.address = local_pdata.compass.address;
	for (ii = 0; ii < DIM(pdata->compass.orientation); ii++)
		pdata->compass.orientation[ii] =
		    local_pdata.compass.orientation[ii];

	pdata->pressure.address = local_pdata.pressure.address;
	for (ii = 0; ii < DIM(pdata->pressure.orientation); ii++)
		pdata->pressure.orientation[ii] =
		    local_pdata.pressure.orientation[ii];

	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	return INV_SUCCESS;
}

static int
mpu_dev_ioctl_set_mpu_config(struct i2c_client *client, unsigned long arg)
{
	int ii;
	int result = INV_SUCCESS;
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct mldl_cfg *temp_mldl_cfg;

	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	temp_mldl_cfg = kzalloc(sizeof(struct mldl_cfg), GFP_KERNEL);
	if (NULL == temp_mldl_cfg)
		return -ENOMEM;

	/*
	 * User space is not allowed to modify accel compass pressure or
	 * pdata structs, as well as silicon_revision product_id or trim
	 */
	if (copy_from_user(temp_mldl_cfg, (struct mldl_cfg __user *)arg,
			   offsetof(struct mldl_cfg, silicon_revision))) {
		result = -EFAULT;
		goto out;
	}

	if (mldl_cfg->gyro_is_suspended) {
		if (mldl_cfg->addr != temp_mldl_cfg->addr)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->int_config != temp_mldl_cfg->int_config)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->ext_sync != temp_mldl_cfg->ext_sync)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->full_scale != temp_mldl_cfg->full_scale)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->lpf != temp_mldl_cfg->lpf)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->clk_src != temp_mldl_cfg->clk_src)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->divider != temp_mldl_cfg->divider)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->dmp_enable != temp_mldl_cfg->dmp_enable)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->fifo_enable != temp_mldl_cfg->fifo_enable)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->dmp_cfg1 != temp_mldl_cfg->dmp_cfg1)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->dmp_cfg2 != temp_mldl_cfg->dmp_cfg2)
			mldl_cfg->gyro_needs_reset = TRUE;

		for (ii = 0; ii < MPU_NUM_AXES; ii++)
			if (mldl_cfg->offset_tc[ii] !=
			    temp_mldl_cfg->offset_tc[ii])
				mldl_cfg->gyro_needs_reset = TRUE;

		for (ii = 0; ii < MPU_NUM_AXES; ii++)
			if (mldl_cfg->offset[ii] != temp_mldl_cfg->offset[ii])
				mldl_cfg->gyro_needs_reset = TRUE;

		if (memcmp(mldl_cfg->ram, temp_mldl_cfg->ram,
			   MPU_MEM_NUM_RAM_BANKS * MPU_MEM_BANK_SIZE *
			   sizeof(unsigned char)))
			mldl_cfg->gyro_needs_reset = TRUE;
	}

	memcpy(mldl_cfg, temp_mldl_cfg,
	       offsetof(struct mldl_cfg, silicon_revision));

 out:
	kfree(temp_mldl_cfg);
	return result;
}

static int
mpu_dev_ioctl_get_mpu_config(struct i2c_client *client, unsigned long arg)
{
	/* Have to be careful as there are 3 pointers in the mldl_cfg
	 * structure */
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct mldl_cfg *local_mldl_cfg;
	int retval = 0;

	local_mldl_cfg = kzalloc(sizeof(struct mldl_cfg), GFP_KERNEL);
	if (NULL == local_mldl_cfg)
		return -ENOMEM;

	retval =
	    copy_from_user(local_mldl_cfg, (struct mldl_cfg __user *)arg,
			   sizeof(struct mldl_cfg));
	if (retval) {
		dev_err(&client->adapter->dev,
			"%s|%s:%d: EFAULT on arg\n",
			__FILE__, __func__, __LINE__);
		retval = -EFAULT;
		goto out;
	}

	/* Fill in the accel, compass, pressure and pdata pointers */
	if (mldl_cfg->accel) {
		retval = copy_to_user((void __user *)local_mldl_cfg->accel,
				      mldl_cfg->accel,
				      sizeof(*mldl_cfg->accel));
		if (retval) {
			dev_err(&client->adapter->dev,
				"%s|%s:%d: EFAULT on accel\n",
				__FILE__, __func__, __LINE__);
			retval = -EFAULT;
			goto out;
		}
	}

	if (mldl_cfg->compass) {
		retval = copy_to_user((void __user *)local_mldl_cfg->compass,
				      mldl_cfg->compass,
				      sizeof(*mldl_cfg->compass));
		if (retval) {
			dev_err(&client->adapter->dev,
				"%s|%s:%d: EFAULT on compass\n",
				__FILE__, __func__, __LINE__);
			retval = -EFAULT;
			goto out;
		}
	}

	if (mldl_cfg->pressure) {
		retval = copy_to_user((void __user *)local_mldl_cfg->pressure,
				      mldl_cfg->pressure,
				      sizeof(*mldl_cfg->pressure));
		if (retval) {
			dev_err(&client->adapter->dev,
				"%s|%s:%d: EFAULT on pressure\n",
				__FILE__, __func__, __LINE__);
			retval = -EFAULT;
			goto out;
		}
	}

	if (mldl_cfg->pdata) {
		retval = copy_to_user((void __user *)local_mldl_cfg->pdata,
				      mldl_cfg->pdata,
				      sizeof(*mldl_cfg->pdata));
		if (retval) {
			dev_err(&client->adapter->dev,
				"%s|%s:%d: EFAULT on pdata\n",
				__FILE__, __func__, __LINE__);
			retval = -EFAULT;
			goto out;
		}
	}

	/* Do not modify the accel, compass, pressure and pdata pointers */
	retval = copy_to_user((struct mldl_cfg __user *)arg,
			      mldl_cfg, offsetof(struct mldl_cfg, accel));

	if (retval)
		retval = -EFAULT;
 out:
	kfree(local_mldl_cfg);
	return retval;
}

/**
 * Pass a requested slave configuration to the slave sensor
 *
 * @param adapter the adaptor to use to communicate with the slave
 * @param mldl_cfg the mldl configuration structuer
 * @param slave pointer to the slave descriptor
 * @param usr_config The configuration to pass to the slave sensor
 *
 * @return 0 or non-zero error code
 */
static int slave_config(struct mldl_cfg *mldl_cfg,
			void *gyro_adapter,
			void *slave_adapter,
			struct ext_slave_descr *slave,
			struct ext_slave_platform_data *pdata,
			struct ext_slave_config __user *usr_config)
{
	int retval = INV_SUCCESS;
	struct ext_slave_config config;
	if ((!slave) || (!slave->config))
		return INV_ERROR_FEATURE_NOT_IMPLEMENTED;

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	if (config.len && config.data) {
		int *data;
		data = kzalloc(config.len, GFP_KERNEL);
		if (!data)
			return INV_ERROR_MEMORY_EXAUSTED;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = inv_mpu_slave_config(mldl_cfg, gyro_adapter, slave_adapter,
				      &config, slave, pdata);
	kfree(config.data);
	return retval;
}

/**
 * Get a requested slave configuration from the slave sensor
 *
 * @param adapter the adaptor to use to communicate with the slave
 * @param mldl_cfg the mldl configuration structuer
 * @param slave pointer to the slave descriptor
 * @param usr_config The configuration for the slave to fill out
 *
 * @return 0 or non-zero error code
 */
static int slave_get_config(struct mldl_cfg *mldl_cfg,
			    void *gyro_adapter,
			    void *slave_adapter,
			    struct ext_slave_descr *slave,
			    struct ext_slave_platform_data *pdata,
			    struct ext_slave_config __user *usr_config)
{
	int retval = INV_SUCCESS;
	struct ext_slave_config config;
	void *user_data;
	if (!(slave) || !(slave->get_config))
		return INV_ERROR_FEATURE_NOT_IMPLEMENTED;

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	user_data = config.data;
	if (config.len && config.data) {
		int *data;
		data = kzalloc(config.len, GFP_KERNEL);
		if (!data)
			return INV_ERROR_MEMORY_EXAUSTED;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = inv_mpu_get_slave_config(mldl_cfg, gyro_adapter,
					  slave_adapter, &config, slave, pdata);
	if (retval) {
		kfree(config.data);
		return retval;
	}
	retval = copy_to_user((unsigned char __user *)user_data,
			      config.data, config.len);
	kfree(config.data);
	return retval;
}

static int inv_slave_read(struct mldl_cfg *mldl_cfg,
			  void *gyro_adapter,
			  void *slave_adapter,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata,
			  void __user *usr_data)
{
	int retval;
	unsigned char *data;
	data = kzalloc(slave->read_len, GFP_KERNEL);
	if (!data)
		return -EFAULT;

	retval = inv_mpu_slave_read(mldl_cfg, gyro_adapter, slave_adapter,
				    slave, pdata, data);

	if ((INV_SUCCESS == retval) &&
	    (copy_to_user((unsigned char __user *)usr_data,
			  data, slave->read_len)))
		retval = -EFAULT;

	kfree(data);
	return retval;
}

static int mpu_handle_mlsl(void *sl_handle,
			   unsigned char addr,
			   unsigned int cmd,
			   struct mpu_read_write __user *usr_msg)
{
	int retval = INV_SUCCESS;
	struct mpu_read_write msg;
	unsigned char *user_data;
	retval = copy_from_user(&msg, usr_msg, sizeof(msg));
	if (retval)
		return -EFAULT;

	user_data = msg.data;
	if (msg.length && msg.data) {
		unsigned char *data;
		data = kzalloc(msg.length, GFP_KERNEL);
		if (!data)
			return INV_ERROR_MEMORY_EXAUSTED;

		retval = copy_from_user(data,
					(void __user *)msg.data, msg.length);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		msg.data = data;
	} else {
		return INV_ERROR_INVALID_PARAMETER;
	}

	switch (cmd) {
	case MPU_READ:
		retval = inv_serial_read(sl_handle, addr,
					 msg.address, msg.length, msg.data);
		break;
	case MPU_WRITE:
		retval = inv_serial_write(sl_handle, addr,
					  msg.length, msg.data);
		break;
	case MPU_READ_MEM:
		retval = inv_serial_read_mem(sl_handle, addr,
					     msg.address, msg.length, msg.data);
		break;
	case MPU_WRITE_MEM:
		retval = inv_serial_write_mem(sl_handle, addr,
					      msg.address, msg.length,
					      msg.data);
		break;
	case MPU_READ_FIFO:
		retval = inv_serial_read_fifo(sl_handle, addr,
					      msg.length, msg.data);
		break;
	case MPU_WRITE_FIFO:
		retval = inv_serial_write_fifo(sl_handle, addr,
					       msg.length, msg.data);
		break;

	};
	retval = copy_to_user((unsigned char __user *)user_data,
			      msg.data, msg.length);
	kfree(msg.data);
	return retval;
}

/* ioctl - I/O control */
static long mpu_dev_ioctl(struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	int retval = 0;
	struct i2c_adapter *gyro_adapter;
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;

	gyro_adapter = client->adapter;
	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	retval = mutex_lock_interruptible(&mpu->mutex);
	if (retval) {
		dev_err(&client->adapter->dev,
			"%s: mutex_lock_interruptible returned %d\n",
			__func__, retval);
		return retval;
	}

	switch (cmd) {
	case MPU_SET_MPU_CONFIG:
		retval = mpu_dev_ioctl_set_mpu_config(client, arg);
		break;
	case MPU_SET_PLATFORM_DATA:
		retval = mpu_dev_ioctl_set_mpu_pdata(client, arg);
		break;
	case MPU_GET_MPU_CONFIG:
		retval = mpu_dev_ioctl_get_mpu_config(client, arg);
		break;
	case MPU_READ:
	case MPU_WRITE:
	case MPU_READ_MEM:
	case MPU_WRITE_MEM:
	case MPU_READ_FIFO:
	case MPU_WRITE_FIFO:
		retval = mpu_handle_mlsl(gyro_adapter, mldl_cfg->addr, cmd,
					 (struct mpu_read_write __user *)arg);
		break;
	case MPU_CONFIG_ACCEL:
		retval = slave_config(mldl_cfg,
				      gyro_adapter,
				      accel_adapter,
				      mldl_cfg->accel,
				      &mldl_cfg->pdata->accel,
				      (struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_COMPASS:
		retval = slave_config(mldl_cfg,
				      gyro_adapter,
				      compass_adapter,
				      mldl_cfg->compass,
				      &mldl_cfg->pdata->compass,
				      (struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_PRESSURE:
		retval = slave_config(mldl_cfg,
				      gyro_adapter,
				      pressure_adapter,
				      mldl_cfg->pressure,
				      &mldl_cfg->pdata->pressure,
				      (struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_ACCEL:
		retval = slave_get_config(mldl_cfg,
					  gyro_adapter,
					  accel_adapter,
					  mldl_cfg->accel,
					  &mldl_cfg->pdata->accel,
					  (struct ext_slave_config __user *)
					  arg);
		break;
	case MPU_GET_CONFIG_COMPASS:
		retval = slave_get_config(mldl_cfg,
					  gyro_adapter,
					  compass_adapter,
					  mldl_cfg->compass,
					  &mldl_cfg->pdata->compass,
					  (struct ext_slave_config __user *)
					  arg);
		break;
	case MPU_GET_CONFIG_PRESSURE:
		retval = slave_get_config(mldl_cfg,
					  gyro_adapter,
					  pressure_adapter,
					  mldl_cfg->pressure,
					  &mldl_cfg->pdata->pressure,
					  (struct ext_slave_config __user *)
					  arg);
		break;
	case MPU_SUSPEND:
		retval = inv_mpu_suspend(mldl_cfg,
					 gyro_adapter,
					 accel_adapter,
					 compass_adapter,
					 pressure_adapter,
					 (~(mldl_cfg->requested_sensors))
					  & INV_ALL_SENSORS);
		break;
	case MPU_RESUME:
		retval = inv_mpu_resume(mldl_cfg,
					gyro_adapter,
					accel_adapter,
					compass_adapter,
					pressure_adapter,
					mldl_cfg->requested_sensors);
		break;
	case MPU_PM_EVENT_HANDLED:
		dev_dbg(&client->adapter->dev, "%s: %d\n", __func__, cmd);
		complete(&mpu->completion);
		break;
	case MPU_READ_ACCEL:
		{
			retval = inv_slave_read(mldl_cfg,
						gyro_adapter,
						accel_adapter,
						mldl_cfg->accel,
						&mldl_cfg->pdata->accel,
						(unsigned char __user *)arg);
		}
		break;
	case MPU_READ_COMPASS:
		{
			retval = inv_slave_read(mldl_cfg,
						gyro_adapter,
						compass_adapter,
						mldl_cfg->compass,
						&mldl_cfg->pdata->compass,
						(unsigned char __user *)arg);
		}
		break;
	case MPU_READ_PRESSURE:
		{
			retval = inv_slave_read(mldl_cfg,
						gyro_adapter,
						pressure_adapter,
						mldl_cfg->pressure,
						&mldl_cfg->pdata->pressure,
						(unsigned char __user *)arg);
		}
		break;
	default:
		dev_err(&client->adapter->dev,
			"%s: Unknown cmd %x, arg %lu: MIN %x MAX %x\n",
			__func__, cmd, arg,
			MPU_SET_MPU_CONFIG, MPU_SET_MPU_CONFIG);
		retval = -EINVAL;
	}

	mutex_unlock(&mpu->mutex);
	return retval;
}

void mpu_shutdown(struct i2c_client *client)
{
	printk("%s_shutdown+\n",__FUNCTION__);
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;

	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	mutex_lock(&mpu->mutex);
	(void)inv_mpu_suspend(mldl_cfg, client->adapter,
			      accel_adapter, compass_adapter, pressure_adapter,
			      INV_ALL_SENSORS);
	mutex_unlock(&mpu->mutex);
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	printk("%s_shutdown-\n",__FUNCTION__);
}

int mpu_dev_suspend(struct device *dev, pm_message_t mesg)
{
	printk("%s+\n", __FUNCTION__);
	struct i2c_client *client = i2c_verify_client(dev);
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;

	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	mutex_lock(&mpu->mutex);
	if (!mldl_cfg->ignore_system_suspend) {
		dev_dbg(&client->adapter->dev,
			"%s: suspending on event %d\n", __func__, mesg.event);
		(void)inv_mpu_suspend(mldl_cfg, client->adapter,
				      accel_adapter, compass_adapter,
				      pressure_adapter, INV_ALL_SENSORS);
	} else {
		dev_dbg(&client->adapter->dev,
			"%s: Already suspended %d\n", __func__, mesg.event);
	}
	mutex_unlock(&mpu->mutex);
	printk("%s-\n", __FUNCTION__);
	return 0;
}

int mpu_dev_resume(struct device *dev)
{
	printk("%s+\n", __FUNCTION__);
	struct i2c_client *client = i2c_verify_client(dev);
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;

	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	mutex_lock(&mpu->mutex);
	if (mpu->pid && !mldl_cfg->ignore_system_suspend) {
		(void)inv_mpu_resume(mldl_cfg, client->adapter,
				     accel_adapter,
				     compass_adapter,
				     pressure_adapter,
				     mldl_cfg->requested_sensors);
		dev_dbg(&client->adapter->dev,
			"%s for pid %d\n", __func__, mpu->pid);
	}
	mutex_unlock(&mpu->mutex);
	printk("%s-\n", __FUNCTION__);
	return 0;
}

/* define which file operations are supported */
static const struct file_operations mpu_fops = {
	.owner = THIS_MODULE,
	.read = mpu_read,
	.poll = mpu_poll,

#if HAVE_COMPAT_IOCTL
	.compat_ioctl = mpu_dev_ioctl,
#endif
#if HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = mpu_dev_ioctl,
#endif
	.open = mpu_dev_open,
	.release = mpu_release,
};

static unsigned short normal_i2c[] = { I2C_CLIENT_END };

#ifdef I2C_CLIENT_INSMOD
I2C_CLIENT_INSMOD;
#endif

int mpu_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	printk(KERN_INFO "%s+ #####\n", __func__);
	struct mpu_platform_data *pdata;
	struct mpu_private_data *mpu;
	struct mldl_cfg *mldl_cfg;
	int res = 0;
	struct i2c_adapter *accel_adapter = NULL;
	struct i2c_adapter *compass_adapter = NULL;
	struct i2c_adapter *pressure_adapter = NULL;
	unsigned char accel_wia = 0;
	unsigned char compass_wia = 0;


	dev_info(&client->adapter->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		res = -ENODEV;
		goto out_check_functionality_failed;
	}

	mpu = kzalloc(sizeof(struct mpu_private_data), GFP_KERNEL);
	if (!mpu) {
		res = -ENOMEM;
		goto out_alloc_data_failed;
	}

	i2c_set_clientdata(client, mpu);
	mpu->client = client;
	this_client = mpu->client;
	mpu->gyro_status = 0;
	mpu->accel_status = 0;
	mpu->compass_status = 0;
	mldl_cfg = &mpu->mldl_cfg;

	init_waitqueue_head(&mpu->mpu_event_wait);

	mutex_init(&mpu->mutex);
	init_completion(&mpu->completion);

	mpu->response_timeout = 60;	/* Seconds */
	mpu->timeout.function = mpu_pm_timeout;
	mpu->timeout.data = (u_long) mpu;
	init_timer(&mpu->timeout);

	pdata = (struct mpu_platform_data *)client->dev.platform_data;
	if (!pdata) {
		dev_warn(&client->adapter->dev,
			 "Missing platform data for mpu\n");
	} else {
		mldl_cfg->pdata = pdata;

//#if defined(CONFIG_INV_SENSORS_MODULE)
		pdata->accel.get_slave_descr = get_accel_slave_descr;
		pdata->compass.get_slave_descr = get_compass_slave_descr;
		pdata->pressure.get_slave_descr = get_pressure_slave_descr;
//#endif

		if (pdata->accel.get_slave_descr) {
			mldl_cfg->accel = pdata->accel.get_slave_descr();
			dev_info(&client->adapter->dev,
				 "%s: +%s\n", MPU_NAME, mldl_cfg->accel->name);
			accel_adapter = i2c_get_adapter(pdata->accel.adapt_num);
			if (pdata->accel.irq > 0) {
				dev_info(&client->adapter->dev,
					 "Installing Accel irq using %d\n",
					 pdata->accel.irq);
				res = slaveirq_init(accel_adapter,
						    &pdata->accel, "accelirq");
				if (res)
					goto out_accelirq_failed;
			} else {
				dev_warn(&client->adapter->dev,
					 "WARNING: Accel irq not assigned\n");
			}
		} else {
			dev_warn(&client->adapter->dev,
				 "%s: No Accel Present\n", MPU_NAME);
		}

		if (pdata->compass.get_slave_descr) {
			mldl_cfg->compass = pdata->compass.get_slave_descr();
			dev_info(&client->adapter->dev,
				 "%s: +%s\n", MPU_NAME,
				 mldl_cfg->compass->name);
			compass_adapter =
			    i2c_get_adapter(pdata->compass.adapt_num);
			if (pdata->compass.irq > 0) {
				dev_info(&client->adapter->dev,
					 "Installing Compass irq using %d\n",
					 pdata->compass.irq);
				res = slaveirq_init(compass_adapter,
						    &pdata->compass,
						    "compassirq");
				if (res)
					goto out_compassirq_failed;
			} else {
				dev_warn(&client->adapter->dev,
					 "WARNING: Compass irq not assigned\n");
			}
		} else {
			dev_warn(&client->adapter->dev,
				 "%s: No Compass Present\n", MPU_NAME);
		}

		if (pdata->pressure.get_slave_descr) {
			mldl_cfg->pressure = pdata->pressure.get_slave_descr();
			dev_info(&client->adapter->dev,
				 "%s: +%s\n", MPU_NAME,
				 mldl_cfg->pressure->name);
			pressure_adapter =
			    i2c_get_adapter(pdata->pressure.adapt_num);

			if (pdata->pressure.irq > 0) {
				dev_info(&client->adapter->dev,
					 "Installing Pressure irq using %d\n",
					 pdata->pressure.irq);
				res = slaveirq_init(pressure_adapter,
						    &pdata->pressure,
						    "pressureirq");
				if (res)
					goto out_pressureirq_failed;
			} else {
				dev_warn(&client->adapter->dev,
					 "Pressure irq not assigned\n");
			}
		} else {
			dev_warn(&client->adapter->dev,
				 "%s: No Pressure Present\n", MPU_NAME);
		}
	}

	mldl_cfg->addr = client->addr;
	res = inv_mpu_open(&mpu->mldl_cfg, client->adapter,
			   accel_adapter, compass_adapter, pressure_adapter);

	if (res) {
		dev_err(&client->adapter->dev,
			"Unable to open %s %d\n", MPU_NAME, res);
		res = -ENODEV;
		goto out_whoami_failed;
	}
/*	mpu->nb.notifier_call = mpu_pm_notifier_callback;
	mpu->nb.priority = 0;
	register_pm_notifier(&mpu->nb);*/

	mpu->dev.minor = MISC_DYNAMIC_MINOR;
	mpu->dev.name = "mpu";		/* Same for both 3050 and 6000 */
	mpu->dev.fops = &mpu_fops;
	res = misc_register(&mpu->dev);
	if (res < 0) {
		dev_err(&client->adapter->dev,
			"ERROR: misc_register returned %d\n", res);
		goto out_misc_register_failed;
	}

	if (client->irq > 0) {
		dev_info(&client->adapter->dev,
			 "Installing irq using %d\n", client->irq);
		res = mpuirq_init(client, mldl_cfg);
		if (res)
			goto out_mpuirq_failed;
	} else {
		dev_warn(&client->adapter->dev,
			 "Missing %s IRQ\n", MPU_NAME);
	}

	if(factory_mode){

		mpu->attrs.attrs = mpu_3050_attr;
		res = sysfs_create_group(&client->dev.kobj, &mpu->attrs);

		if (res) {
			dev_err(&client->dev, "Not able to create the sysfs\n");
			printk("%s:Not able to create the sysfs\n", __FUNCTION__);
			goto out_attr_register_failed;
		}
	}

	if(mldl_cfg->accel ){
		res = inv_serial_read(client->adapter, mldl_cfg->pdata->accel.address,
				0x0F, 1, &accel_wia);
		if(res)
			printk("%s: Read accel who am i register fail\n", __FUNCTION__);
		else if(accel_wia == 0x01)
			mpu->accel_status = 1;
		else
			printk("%s: Check accel who am i register fail, wia:%d\n", __FUNCTION__, accel_wia);
	}
	if(mldl_cfg->compass){
		res = inv_serial_read(client->adapter, mldl_cfg->pdata->compass.address,
				0x0F, 1, &compass_wia);
		if(res)
			printk("%s: Read compass who am i register fail\n", __FUNCTION__);
		else if((compass_wia == 0x47) || (compass_wia == 0x46))
			mpu->compass_status = 1;
		else
			printk("%s: Check compass who am i register fail, wia:%d\n", __FUNCTION__, compass_wia);
	}
	mpu->gyro_status = 1;
	printk(KERN_INFO "%s- #####\n", __func__);
	return res;

out_attr_register_failed:
	if (mpu->client->irq)
		mpuirq_exit();
 out_mpuirq_failed:
	misc_deregister(&mpu->dev);
 out_misc_register_failed:
	inv_mpu_close(&mpu->mldl_cfg, client->adapter,
		      accel_adapter, compass_adapter, pressure_adapter);
 out_whoami_failed:
	if (pdata && pdata->pressure.get_slave_descr && pdata->pressure.irq)
		slaveirq_exit(&pdata->pressure);
 out_pressureirq_failed:
	if (pdata && pdata->compass.get_slave_descr && pdata->compass.irq)
		slaveirq_exit(&pdata->compass);
 out_compassirq_failed:
	if (pdata && pdata->accel.get_slave_descr && pdata->accel.irq)
		slaveirq_exit(&pdata->accel);
 out_accelirq_failed:
	kfree(mpu);
 out_alloc_data_failed:
 out_check_functionality_failed:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, res);
	return res;

}

static int mpu_remove(struct i2c_client *client)
{
	struct mpu_private_data *mpu = i2c_get_clientdata(this_client);
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct mpu_platform_data *pdata = mldl_cfg->pdata;

	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	inv_mpu_close(mldl_cfg, client->adapter,
		      accel_adapter, compass_adapter, pressure_adapter);

	if (client->irq)
		mpuirq_exit();

	if (pdata && pdata->pressure.get_slave_descr && pdata->pressure.irq)
		slaveirq_exit(&pdata->pressure);

	if (pdata && pdata->compass.get_slave_descr && pdata->compass.irq)
		slaveirq_exit(&pdata->compass);

	if (pdata && pdata->accel.get_slave_descr && pdata->accel.irq)
		slaveirq_exit(&pdata->accel);

	misc_deregister(&mpu->dev);
	kfree(mpu);

	return 0;
}

static const struct i2c_device_id mpu_id[] = {
	{MPU_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mpu_id);
static const struct dev_pm_ops mpu_dev_pm_ops={
	.suspend = mpu_dev_suspend,
	.resume = mpu_dev_resume,
};

static struct i2c_driver mpu_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = mpu_probe,
	.remove = mpu_remove,
	.id_table = mpu_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = MPU_NAME,
		   .pm = &mpu_dev_pm_ops,
		   },
#ifdef I2C_CLIENT_INSMOD
	.address_data = &addr_data,
#else
	.address_list = normal_i2c,
#endif

	.shutdown = mpu_shutdown,	/* optional */

};

static int __init mpu_init(void)
{
	printk(KERN_INFO "%s+ #####\n", __func__);
	INIT_DELAYED_WORK(&mpu_init_work, mpu_delay_init);
	schedule_delayed_work(&mpu_init_work, 200);
	printk(KERN_INFO "%s- #####\n", __func__);
}

static int mpu_delay_init(void)
{
	printk(KERN_INFO "%s+ #####\n", __func__);
	tegra_gpio_enable(143);
	gpio_request(143, "gpio_pr7");
	gpio_direction_output(143, 1);
	pr_info("gpio 2.85V %d set to %d\n",143, gpio_get_value(143));
	gpio_free(143);

	int res = i2c_add_driver(&mpu_driver);
	pr_info("%s: Probe name %s\n", __func__, MPU_NAME);
	if (res)
		pr_err("%s failed\n", __func__);
	printk(KERN_INFO "%s- #####\n", __func__);
	return res;
}

static void __exit mpu_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&mpu_driver);
}

module_init(mpu_init);
module_exit(mpu_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("User space character device interface for MPU");
MODULE_LICENSE("GPL");
MODULE_ALIAS(MPU_NAME);
