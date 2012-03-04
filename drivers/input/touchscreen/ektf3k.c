/* drivers/input/touchscreen/ektf3k.c - ELAN EKTF3K FIFO verions of driver
 *
 * Copyright (C) 2011 Elan Microelectronics Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define ELAN_BUFFER_MODE

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>

#define PACKET_SIZE		40 	// james
#define FINGER_NUM		10	// james
		
#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK			BIT(3)

#define CMD_S_PKT			0x52
#define CMD_R_PKT			0x53
#define CMD_W_PKT			0x54

#define HELLO_PKT			0x55
#define NORMAL_PKT			0x63
#define TEN_FINGERS_PKT			0x62

#define RPT_LOCK_PKT		0x56
#define RPT_UNLOCK_PKT		0xA6

#define RESET_PKT			0x77
#define CALIB_PKT			0xA8

#define IDX_FINGER			3

#define ABS_MT_POSITION         0x2a    /* Group a set of X and Y */
#define ABS_MT_AMPLITUDE        0x2b    /* Group a set of Z and W */

#include <linux/i2c/ektf3k.h>

// For Firmware Update 
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_MAJOR_FW_VER  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)

uint16_t checksum_err=0;
uint8_t RECOVERY=0x00;
int FW_VERSION=0x00;
int X_RESOLUTION=0x00;
int Y_RESOLUTION=0x00;
int FW_ID=0x00;
int work_lock=0x00;

#define USB_NO_Cable 0
#define USB_DETECT_CABLE 1 
#define USB_SHIFT 0
#define AC_SHIFT 1 
#define USB_Cable ((1 << (USB_SHIFT)) | (USB_DETECT_CABLE))
#define USB_AC_Adapter ((1 << (AC_SHIFT)) | (USB_DETECT_CABLE))
#define USB_CALBE_DETECT_MASK (USB_Cable  | USB_DETECT_CABLE)
static unsigned now_usb_cable_status=0;

#define _ENABLE_DBG_LEVEL    
#ifdef _ENABLE_DBG_LEVEL
	#define PROC_FS_NAME	"ektf_dbg"
	#define PROC_FS_MAX_LEN	8
	static struct proc_dir_entry *dbgProcFile;
#endif

struct elan_ktf3k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	int (*power)(int on);
	struct early_suspend early_suspend;
	int intr_gpio;
// Firmware Information
	int fw_ver;
	int fw_id;
	int x_resolution;
	int y_resolution;
// For Firmare Update 
	struct miscdevice firmware;
      struct attribute_group attrs;
	int status;
	struct switch_dev touch_sdev;
};

static struct elan_ktf3k_ts_data *private_ts = NULL;
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf3k_ts_rough_calibrate(struct i2c_client *client);

// For Firmware Update 
/* Todo: (1) Need to Add the lock mechanism
                  (2) Use the function "container_of" instead of global variable*/
int elan_iap_open(struct inode *inode, struct file *filp){ 
	printk("[ELAN]into elan_iap_open\n");
		if (private_ts == NULL)  printk("private_ts is NULL~~~");
		
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp){    
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp){  
    int ret;
    char *tmp;
    printk("[ELAN]into elan_iap_write\n");

    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);
    
    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }
	
    ret = i2c_master_send(private_ts->client, tmp, count);
    if (ret != count) printk("ELAN i2c_master_send fail, ret=%d \n", ret);
    kfree(tmp);
    return ret;

}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp){    
    char *tmp;
    int ret;  
    long rc;
    printk("[ELAN]into elan_iap_read\n");
   
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;

    ret = i2c_master_recv(private_ts->client, tmp, count);

    if (ret >= 0)
        rc = copy_to_user(buff, tmp, count);
    
    kfree(tmp);

    return ret;
}

static long elan_iap_ioctl(/*struct inode *inode,*/ struct file *filp,    unsigned int cmd, unsigned long arg){

	int __user *ip = (int __user *)arg;
	printk("[ELAN]into elan_iap_ioctl cmd=%u\n", cmd);

	switch (cmd) {        
		case IOCTL_I2C_SLAVE: 
			private_ts->client->addr = (int __user)arg;
			break;   
		case IOCTL_MAJOR_FW_VER:            
			break;        
		case IOCTL_MINOR_FW_VER:            
			break;        
		case IOCTL_RESET:
			break;
		case IOCTL_IAP_MODE_LOCK:
			work_lock=1;
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			work_lock=0;
			if (gpio_get_value(private_ts->intr_gpio))
    			{
        			enable_irq(private_ts->client->irq);
			}
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return RECOVERY;
			break;
		case IOCTL_FW_VER:
			__fw_packet_handler(private_ts->client);
			return FW_VERSION;
			break;
		case IOCTL_X_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return X_RESOLUTION;
			break;
		case IOCTL_Y_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return Y_RESOLUTION;
			break;
		case IOCTL_FW_ID:
			__fw_packet_handler(private_ts->client);
			return FW_ID;
			break;
		case IOCTL_ROUGH_CALIBRATE:
			return elan_ktf3k_ts_rough_calibrate(private_ts->client);
		case IOCTL_I2C_INT:
			put_user(gpio_get_value(private_ts->intr_gpio), ip);
			break;	
		default:            
			break;   
	}       
	return 0;
}

struct file_operations elan_touch_fops = {    
        .open =         elan_iap_open,    
        .write =        elan_iap_write,    
        .read = 	elan_iap_read,    
        .release =	elan_iap_release,    
	.unlocked_ioctl=elan_iap_ioctl, 
 };

// End Firmware Update

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf3k_ts_early_suspend(struct early_suspend *h);
static void elan_ktf3k_ts_late_resume(struct early_suspend *h);
#endif

static ssize_t elan_ktf3k_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_ktf3k_ts_data *ts = private_ts;

	ret = gpio_get_value(ts->intr_gpio);
	printk(KERN_DEBUG "GPIO_TP_INT_N=%d\n", ts->intr_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf3k_gpio_show, NULL);

static ssize_t elan_ktf3k_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf3k_ts_data *ts = private_ts;

	sprintf(buf, "%s_x%4.4x\n", "ELAN_KTF3K", ts->fw_ver);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf3k_vendor_show, NULL);

static ssize_t elan_show_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf3k_ts_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->status);
}

DEVICE_ATTR(elan_touchpanel_status, S_IRUGO, elan_show_status, NULL);

static struct attribute *elan_attr[] = {
	&dev_attr_elan_touchpanel_status.attr,
	&dev_attr_vendor.attr,
	&dev_attr_gpio.attr,
	NULL
};

static struct kobject *android_touch_kobj;

static int elan_ktf3k_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "[elan]%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		printk(KERN_ERR "[elan]%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR "[elan]%s: sysfs_create_group failed\n", __func__);
		return ret;
	}
	return 0 ;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}

static int __elan_ktf3k_ts_poll(struct i2c_client *client)
{
	struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 10;

	do {
		status = gpio_get_value(ts->intr_gpio);
		dev_dbg(&client->dev, "%s: status = %d\n", __func__, status);
		retry--;
		mdelay(20);
	} while (status == 1 && retry > 0);

	dev_dbg(&client->dev, "[elan]%s: poll interrupt status %s\n",
			__func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf3k_ts_poll(struct i2c_client *client)
{
	return __elan_ktf3k_ts_poll(client);
}

static int elan_ktf3k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
			uint8_t *buf, size_t size)
{
	int rc;

	dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		dev_err(&client->dev,
			"[elan]%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf3k_ts_poll(client);
	if (rc < 0)
		return -EINVAL;
	else {
		if (i2c_master_recv(client, buf, size) != size ||
		    buf[0] != CMD_S_PKT)
		        return -EINVAL;
	}

	return 0;
}

static int elan_ktf3k_ts_read_command(struct i2c_client *client,
			   u8* cmd, u16 cmd_length, u8 *value, u16 value_length){
       struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16 le_addr;
	struct elan_ktf3k_ts_data *ts;

	ts = i2c_get_clientdata(client);

	msg[0].addr = client->addr;
	msg[0].flags = 0x00;
	msg[0].len = cmd_length;
	msg[0].buf = cmd;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = value_length;
	msg[1].buf = (u8 *) value;
	if (i2c_transfer(adapter, msg, 2) == 2)
		return value_length;
	else
		return -EIO;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[4] = { 0 };
	uint8_t buf_recv1[4] = { 0 };

	rc = elan_ktf3k_ts_poll(client);
	if (rc < 0) {
		dev_err(&client->dev, "[elan] %s: failed!\n", __func__);
		return -EINVAL;
	}

	rc = i2c_master_recv(client, buf_recv, 4);
	printk("[elan] %s: hello packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);

	if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80)
	{
		rc = elan_ktf3k_ts_poll(client);
		if (rc < 0) {
			dev_err(&client->dev, "[elan] %s: failed!\n", __func__);
			return -EINVAL;
		}
		rc = i2c_master_recv(client, buf_recv1, 4);
		printk("[elan] %s: recovery hello packet %2x:%2X:%2x:%2x\n", __func__, buf_recv1[0], buf_recv1[1], buf_recv1[2], buf_recv1[3]);
		RECOVERY=0x80;
		return RECOVERY;
	}
	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t buf_recv[4] = {0};
// Firmware version
	rc = elan_ktf3k_ts_read_command(client, cmd, 4, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;
// X Resolution
	rc = elan_ktf3k_ts_read_command(client, cmd_x, 4, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->x_resolution =minor;
	X_RESOLUTION = ts->x_resolution;
// Y Resolution	
	rc = elan_ktf3k_ts_read_command(client, cmd_y, 4, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->y_resolution =minor;
	Y_RESOLUTION = ts->y_resolution;
// Firmware ID
	rc = elan_ktf3k_ts_read_command(client, cmd_id, 4, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;

	printk(KERN_INFO "[elan] %s: firmware version: 0x%4.4x\n",
			__func__, ts->fw_ver);
	printk(KERN_INFO "[elan] %s: firmware ID: 0x%4.4x\n",
			__func__, ts->fw_id);
	printk(KERN_INFO "[elan] %s: x resolution: %d, y resolution: %d\n",
			__func__, ts->x_resolution, ts->y_resolution);
	
	return 0;
}

static inline int elan_ktf3k_ts_parse_xy(uint8_t *data,
			uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ktf3k_ts_setup(struct i2c_client *client)
{
	int rc;

	rc = __hello_packet_handler(client);
	if (rc < 0)
		goto hand_shake_failed;
	dev_dbg(&client->dev, "[elan] %s: hello packet got.\n", __func__);
// for firmware update
	if(rc==0x80)
	{
		return rc;
	}
// end firmware update
	rc = __fw_packet_handler(client);
	if (rc < 0)
		goto hand_shake_failed;
	dev_dbg(&client->dev, "[elan] %s: firmware checking done.\n", __func__);

hand_shake_failed:
	return rc;
}

static int elan_ktf3k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf3k_ts_rough_calibrate(struct i2c_client *client){
      uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};

	//dev_info(&client->dev, "[elan] %s: enter\n", __func__);
	printk("[elan] %s: enter\n", __func__);
	dev_info(&client->dev,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf3k_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t buf[4], power_state;

	rc = elan_ktf3k_ts_get_data(client, cmd, buf, 4);
	if (rc)
		return rc;

	power_state = buf[1];
	dev_dbg(&client->dev, "[elan] dump repsponse: %0x\n", power_state);
	power_state = (power_state & PWR_STATE_MASK) >> 3;
	dev_dbg(&client->dev, "[elan] power state = %s\n",
		power_state == PWR_STATE_DEEP_SLEEP ?
		"Deep Sleep" : "Normal/Idle");

	return power_state;
}

static int elan_ktf3k_ts_set_power_source(struct i2c_client *client, u8 state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x40, 0x00, 0x01};

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);
    /*0x52 0x40 0x00 0x01  =>    Battery Mode
       0x52 0x41 0x00 0x01  =>   AC Adapter Mode
       0x52 0x42 0x00 0x01 =>    USB Mode */
	cmd[1] |= state & 0x0F;

	dev_dbg(&client->dev,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf3k_ts_get_power_source(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x40, 0x00, 0x01};
	uint8_t buf[4] = {0}, power_source;

	//rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	rc = elan_ktf3k_ts_read_command(client, cmd, 4, buf, 4);
	if (rc < 0)
		return rc;

	power_source = buf[1];
	dev_dbg(&client->dev, "[elan] get power source : %0x\n", power_source);
	power_source = (power_source & 0x0F);
	dev_dbg(&client->dev, "[elan] power source = %d\n", power_source);

	return power_source;
}

static void update_power_source(){
      unsigned power_source = now_usb_cable_status;
	if(private_ts == NULL) return;
	
      dev_info(&private_ts->client->dev, "Update power source to %d\n", power_source);
      switch(power_source){
	case USB_NO_Cable:
	    elan_ktf3k_ts_set_power_source(private_ts->client, 0);
	    break;
	case USB_Cable:
          elan_ktf3k_ts_set_power_source(private_ts->client, 1);
	    break;
	case USB_AC_Adapter:
          elan_ktf3k_ts_set_power_source(private_ts->client, 2);
      }

      power_source = elan_ktf3k_ts_get_power_source(private_ts->client);
      dev_info(&private_ts->client->dev, "Get power source %d\n", power_source);	  
}

void touch_callback_elan(unsigned cable_status){ 
      now_usb_cable_status = cable_status;
      update_power_source();
}

static int elan_ktf3k_ts_recv_data(struct i2c_client *client, uint8_t *buf, int size)
{

	int rc, bytes_to_recv = size;

	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);
	rc = i2c_master_recv(client, buf, bytes_to_recv);

	if (rc != bytes_to_recv) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_recv error?! \n", __func__);
		rc = i2c_master_recv(client, buf, bytes_to_recv);
		return -EINVAL;
	}

	return rc;
}

static void elan_ktf3k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits=0, checksum=0;
	uint8_t i, num;
	uint8_t reported = 0;

        num = buf[2] & 0xf; 

	for (i=0; i<34;i++)
		checksum +=buf[i];

	if ( (num < 3) || ((checksum & 0x00ff) == buf[34])) {   
     	   switch (buf[0]) {
	   case NORMAL_PKT:
	   case TEN_FINGERS_PKT:
		fbits = buf[2] & 0x30;	
		fbits = (fbits << 4) | buf[1];  
		input_report_key(idev, BTN_TOUCH, 1);
		if (num == 0) {
			dev_dbg(&client->dev, "no press\n");
		} else {
			uint8_t idx;
			dev_dbg(&client->dev, "[elan] %d fingers\n", num);
                        idx=IDX_FINGER;
                   input_report_key(idev, BTN_TOUCH, 1);
			for (i = 0; i < FINGER_NUM; i++) {
			  if ((fbits & 0x1)) {
			     elan_ktf3k_ts_parse_xy(&buf[idx], &x, &y);  
                             y = ELAN_Y_MAX - y;

			     if (!((x<=0) || (y<=0) || (x>=ELAN_X_MAX) || (y>=ELAN_Y_MAX))) {   
    				input_report_abs(idev, ABS_MT_TRACKING_ID, i);
				input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 8);
				input_report_abs(idev, ABS_MT_POSITION_X, x);
				input_report_abs(idev, ABS_MT_POSITION_Y, y);
				input_mt_sync(idev);
				reported++;
			     } // end if border
 			  } // end if finger status

			  fbits = fbits >> 1;
			  idx += 3;
			} // end for
		}

	      if (reported)
		    input_sync(idev);
		else {
		    input_mt_sync(idev);
		    input_sync(idev);
		}

		break;
	   default:
		dev_err(&client->dev,
			"[elan] %s: unknown packet type: %0x\n", __func__, buf[0]);
		break;
	   } // end switch

	} // checksum
	else {
		checksum_err +=1;
		printk("[elan] Checksum Error %d\n", checksum_err);
	} 

	return;
}

static void elan_ktf3k_ts_work_func(struct work_struct *work)
{
	int rc;
	struct elan_ktf3k_ts_data *ts =
		container_of(work, struct elan_ktf3k_ts_data, work);
	uint8_t buf[44] = { 0 };
	uint8_t buf1[PACKET_SIZE] = { 0 };
	uint8_t buf2[PACKET_SIZE] = { 0 };

	if(work_lock==0)
	{ 

		if (gpio_get_value(ts->intr_gpio))
		{
			enable_irq(ts->client->irq);
			return;
		}
	// First package
#ifndef ELAN_BUFFER_MODE
		rc = elan_ktf3k_ts_recv_data(ts->client, buf, 40);
#else
		rc = elan_ktf3k_ts_recv_data(ts->client, buf, 44);
#endif 
		if (rc < 0)
		{
			enable_irq(ts->client->irq);
			return;
		}
#ifndef ELAN_BUFFER_MODE
		elan_ktf3k_ts_report_data(ts->client, buf);
#else
		elan_ktf3k_ts_report_data(ts->client, buf+4);
	// Second package
	 	if ((buf[1] == 2) || (buf[1] == 3)) {
			rc = elan_ktf3k_ts_recv_data(ts->client, buf1,PACKET_SIZE);
			if (rc < 0)
			{
				enable_irq(ts->client->irq);
				return;
			}
			elan_ktf3k_ts_report_data(ts->client, buf1);
		}
		
	// Final package
		if (buf[1] == 3) {
			rc = elan_ktf3k_ts_recv_data(ts->client, buf2, PACKET_SIZE);
			if (rc < 0)
			{
				enable_irq(ts->client->irq);
				return;
			}
			elan_ktf3k_ts_report_data(ts->client, buf2);
		}
#endif

		enable_irq(ts->client->irq);
	}
	return;
}

static irqreturn_t elan_ktf3k_ts_irq_handler(int irq, void *dev_id)
{
	struct elan_ktf3k_ts_data *ts = dev_id;
	struct i2c_client *client = ts->client;

	dev_dbg(&client->dev, "[elan] %s\n", __func__);
	disable_irq_nosync(ts->client->irq);
	queue_work(ts->elan_wq, &ts->work);

	return IRQ_HANDLED;
}

static int elan_ktf3k_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_irq(client->irq, elan_ktf3k_ts_irq_handler,
			IRQF_TRIGGER_LOW, client->name, ts);
	if (err)
		dev_err(&client->dev, "[elan] %s: request_irq %d failed\n",
				__func__, client->irq);

	return err;
}


static ssize_t elan_touch_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "ELAN-%4.4x-%4.4x\n", 
		   private_ts->fw_id, private_ts->fw_ver);
}

static ssize_t elan_touch_switch_state(struct switch_dev *sdev, char *buf)
{ 
      	return sprintf(buf, "%s\n", "0");
}

#ifdef _ENABLE_DBG_LEVEL
static int ektf_proc_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data )
{
	int ret;
	
	printk("call proc_read\n");
	
	if(offset > 0)  /* we have finished to read, return 0 */
		ret  = 0;
	else 
		ret = sprintf(buffer, "Debug Level: Release Date: %s\n","2011/10/05");

	return ret;
}

static int ektf_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char procfs_buffer_size = 0; 
	int i, ret;
	unsigned char procfs_buf[PROC_FS_MAX_LEN+1] = {0};
	unsigned int command;

	procfs_buffer_size = count;
	if(procfs_buffer_size > PROC_FS_MAX_LEN ) 
		procfs_buffer_size = PROC_FS_MAX_LEN+1;
	
	if( copy_from_user(procfs_buf, buffer, procfs_buffer_size) ) 
	{
		printk(" proc_write faied at copy_from_user\n");
		return -EFAULT;
	}

	command = 0;
	for(i=0; i<procfs_buffer_size-1; i++)
	{
		if( procfs_buf[i]>='0' && procfs_buf[i]<='9' )
			command |= (procfs_buf[i]-'0');
		else if( procfs_buf[i]>='A' && procfs_buf[i]<='F' )
			command |= (procfs_buf[i]-'A'+10);
		else if( procfs_buf[i]>='a' && procfs_buf[i]<='f' )
			command |= (procfs_buf[i]-'a'+10);
		
		if(i!=procfs_buffer_size-2)
			command <<= 4;
	}

	command = command&0xFFFFFFFF;
      switch(command){
      case 0xFF:
	      	ret = elan_ktf3k_ts_rough_calibrate(private_ts->client);
	}
	printk("Run command: 0x%08X  result:%d\n", command, ret);

	return count; // procfs_buffer_size;
}
#endif // #ifdef _ENABLE_DBG_LEV


/*Todo: (5)Add the IC checkout mechanism: ask twice before iniitalize */
static int elan_ktf3k_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	struct elan_ktf3k_i2c_platform_data *pdata;
	struct elan_ktf3k_ts_data *ts;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[elan] %s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf3k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "[elan] %s: allocate elan_ktf3k_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		printk(KERN_ERR "[elan] %s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&ts->work, elan_ktf3k_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	if (likely(pdata != NULL)) {
		ts->intr_gpio = pdata->intr_gpio;
	}

	err = elan_ktf3k_ts_setup(client);
	if (err < 0) {
		printk(KERN_INFO "No Elan chip inside\n");
		err = -ENODEV;
		goto err_detect_failed;
	}
// Firmware Update
	if(err==0x80)
	{
		// MISC
		private_ts = ts;
  	ts->firmware.minor = MISC_DYNAMIC_MINOR;
  	ts->firmware.name = "elan-iap";
  	ts->firmware.fops = &elan_touch_fops;
  	ts->firmware.mode = S_IRWXUGO; 
   	
  	if (misc_register(&ts->firmware) < 0)
  		printk("[ELAN]misc_register failed!!");
  	else
  	  printk("[ELAN]misc_register finished!!");
  	  return 0;
	}
// End Firmware Update

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "[elan] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
//	ts->input_dev->name = "elan-touchscreen";   // for andorid2.2 Froyo  
	ts->input_dev->name = "atmel-maxtouch";     // for andorid3.1 HoneyComb 

	set_bit(BTN_TOUCH, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_X, pdata->abs_x_min,  pdata->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, pdata->abs_y_min,  pdata->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, pdata->abs_x_min,  pdata->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, pdata->abs_y_min,  pdata->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, FINGER_NUM, 0, 0);	// james Max Finger number is 5

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(EV_TOUCH, ts->input_dev->evbit);

	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
			"[elan]%s: unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	elan_ktf3k_ts_register_interrupt(ts->client);

	if (gpio_get_value(ts->intr_gpio) == 0) {
		printk(KERN_INFO "[elan]%s: handle missed interrupt\n", __func__);
		elan_ktf3k_ts_irq_handler(client->irq, ts);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = elan_ktf3k_ts_early_suspend;
	ts->early_suspend.resume = elan_ktf3k_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	private_ts = ts;

	//elan_ktf2k_touch_sysfs_init();
      ts->attrs.attrs = elan_attr;
	err = sysfs_create_group(&client->dev.kobj, &ts->attrs);
	if (err) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
	}
	
#ifdef _ENABLE_DBG_LEVEL
	dbgProcFile = create_proc_entry(PROC_FS_NAME, 0666, NULL);
	if (dbgProcFile == NULL) 
	{
		remove_proc_entry(PROC_FS_NAME, NULL);
		printk(" Could not initialize /proc/%s\n", PROC_FS_NAME);
	}
	else
	{
		dbgProcFile->read_proc = ektf_proc_read;
		dbgProcFile->write_proc = ektf_proc_write;
		printk(" /proc/%s created\n", PROC_FS_NAME);
	}
#endif // #ifdef _ENABLE_DBG_LEVEL

	ts->status=1;
      /* Register Switch file */
      ts->touch_sdev.name = "touch";
      ts->touch_sdev.print_name = elan_touch_switch_name;
	ts->touch_sdev.print_state = elan_touch_switch_state;
	if(switch_dev_register(&ts->touch_sdev) < 0){
		dev_info(&client->dev, "switch_dev_register for dock failed!\n");
		//goto exit;
	}
	switch_set_state(&ts->touch_sdev, 0);
	   
	dev_info(&client->dev, "[elan] Start touchscreen %s in interrupt mode\n",
		ts->input_dev->name);

// Firmware Update
  ts->firmware.minor = MISC_DYNAMIC_MINOR;
  ts->firmware.name = "elan-iap";
  ts->firmware.fops = &elan_touch_fops;
  ts->firmware.mode = S_IFREG|S_IRWXUGO; 

  if (misc_register(&ts->firmware) < 0)
  	printk("[ELAN]misc_register failed!!");
  else
    printk("[ELAN]misc_register finished!!");
// End Firmware Update	
      update_power_source();
	return 0;

err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);

err_create_wq_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:

	return err;
}

static int elan_ktf3k_ts_remove(struct i2c_client *client)
{
	struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);
#ifdef _ENABLE_DBG_LEVEL
	remove_proc_entry(PROC_FS_NAME, NULL);
#endif
	return 0;
}

static int elan_ktf3k_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
	int rc = 0;

	printk(KERN_INFO "[elan] %s: enter\n", __func__);

	disable_irq(client->irq);

	rc = cancel_work_sync(&ts->work);
	if (rc)
		enable_irq(client->irq);

	rc = elan_ktf3k_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);

	return 0;
}

static int elan_ktf3k_ts_resume(struct i2c_client *client)
{

	int rc = 0, retry = 5;

	printk(KERN_INFO "[elan] %s: enter\n", __func__);

	do {
		rc = elan_ktf3k_ts_set_power_state(client, PWR_STATE_NORMAL);
		rc = elan_ktf3k_ts_get_power_state(client);
		if (rc != PWR_STATE_NORMAL)
			printk(KERN_ERR "[elan] %s: wake up tp failed! err = %d\n",
				__func__, rc);
		else
			break;
	} while (--retry);

	enable_irq(client->irq);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf3k_ts_early_suspend(struct early_suspend *h)
{
	struct elan_ktf3k_ts_data *ts;
	ts = container_of(h, struct elan_ktf3k_ts_data, early_suspend);
	elan_ktf3k_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void elan_ktf3k_ts_late_resume(struct early_suspend *h)
{
	struct elan_ktf3k_ts_data *ts;
	ts = container_of(h, struct elan_ktf3k_ts_data, early_suspend);
	elan_ktf3k_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id elan_ktf3k_ts_id[] = {
	{ ELAN_KTF3K_NAME, 0 },
	{ }
};

static struct i2c_driver ektf3k_ts_driver = {
	.probe		= elan_ktf3k_ts_probe,
	.remove		= elan_ktf3k_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= elan_ktf3k_ts_suspend,
	.resume		= elan_ktf3k_ts_resume,
#endif
	.id_table	= elan_ktf3k_ts_id,
	.driver		= {
		.name = ELAN_KTF3K_NAME,
	},
};

static int __devinit elan_ktf3k_ts_init(void)
{
	printk(KERN_INFO "[elan] %s\n", __func__);
	return i2c_add_driver(&ektf3k_ts_driver);
}

static void __exit elan_ktf3k_ts_exit(void)
{
	i2c_del_driver(&ektf3k_ts_driver);
	return;
}

module_init(elan_ktf3k_ts_init);
module_exit(elan_ktf3k_ts_exit);

MODULE_DESCRIPTION("ELAN KTF3K Touchscreen Driver");
MODULE_LICENSE("GPL");
