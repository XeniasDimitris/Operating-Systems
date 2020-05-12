/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Xenias Dimitris
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	int ret;
	WARN_ON ( !(sensor = state->sensor));
	/* ? */

	/* The following return is bogus, just for the stub to compile */
	printk("Sensor time is %d\n",sensor->msr_data[state->type]->last_update);
	printk("State time is %d\n",state->buf_timestamp);
	if (state->buf_timestamp != sensor->msr_data[state->type]->last_update) ret=1;
	else ret=0;
	debug("need refresh is %d",ret);
	return ret; /* ? */
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	sensor = state->sensor;
	debug("entering state_update\n");
	uint16_t value;
	uint32_t timestamp;
	long data;
	int ret = -EAGAIN;	
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	/* ? */
	/* Why use spinlocks? See LDD3, p. 119 */

	/*
	 * Any new data available?
	 */
	/* ? */

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */

	/* ? */
	if(lunix_chrdev_state_needs_refresh(state)){
		debug("Updating the state");	
		spin_lock(&sensor->lock);
		value = sensor->msr_data[state->type]->values[0];	
		timestamp = sensor->msr_data[state->type]->last_update;
		spin_unlock(&sensor->lock);
		
		state->buf_timestamp = timestamp;
		if (state->type == BATT ) data = lookup_voltage[value];
	      	else if (state->type == TEMP) data = lookup_temperature[value];
		else data = lookup_light[value];
		state->buf_lim = snprintf(state->buf_data,LUNIX_CHRDEV_BUFSZ,"%ld.%ld\n",data/1000,data % 1000);
		ret = 0;

	}	
	debug("leaving state update\n");
	return ret;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */
	unsigned int minor_number;
	struct lunix_chrdev_state_struct *private_state;
	int type;
	int ret;
	int sensor;
	debug("entering open\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	minor_number = iminor(inode);
	type = minor_number % 8;
	sensor = minor_number / 8;
	if ( type >= N_LUNIX_MSR){
		debug("Minor number represents a type of non-existed measurement\n");
		goto out;
	}
	/* Allocate a new Lunix character device private state structure */
	/* ? */ 
	private_state = kzalloc(sizeof(*private_state), GFP_KERNEL);
	if(!private_state){
		printk(KERN_ERR "Failed to allocate memory for Lunix sensors\n");
		goto out;
	}
	private_state->type = type;
	private_state->buf_timestamp=0;
	private_state->buf_lim = 0;
	private_state->sensor = &lunix_sensors[sensor];
	sema_init(&private_state->lock,1);
	filp->private_data = private_state;
out:
	debug("leaving from open with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	kfree(filp->private_data);
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;
	int bytes_to_send;
	int remaining_bytes;
	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;	
	
	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);
	debug("entering read\n");	
	/* Lock? */
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (down_interruptible(&state->lock)){
		return -ERESTARTSYS;
	}
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* ? */
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
			up(&state->lock);
			if ( wait_event_interruptible(sensor->wq,lunix_chrdev_state_needs_refresh(state))){
				debug("BLocking the process failed\n");
				return -ERESTARTSYS;
			}
			if (down_interruptible(&state->lock)){
				return -ERESTARTSYS;
			}
		}
	}	
	/* End of file */
	/* ? */
	
	/* Determine the number of cached bytes to copy to userspace */
	/* ? */
	//count = *f_pos;
	//debug("hello\n");
	remaining_bytes = state->buf_lim - *f_pos;
	if (cnt > remaining_bytes) bytes_to_send = remaining_bytes;
	else bytes_to_send = cnt;
	if (copy_to_user(usrbuf, state->buf_data, bytes_to_send)){
		debug("Failed to copy to user\n");
		ret = -EFAULT;
		goto out;
	}
	ret = bytes_to_send;
	/* Auto-rewind on EOF mode? */
	/* ? */
	*f_pos += bytes_to_send;
	if (*f_pos==state->buf_lim) *f_pos=0;

out:
	/* Unlock? */
	up(&state->lock);
	debug("Leaving from read with ret %ld\n",ret);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
        .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	
	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */
	/* register_chrdev_region? */
	ret = register_chrdev_region(dev_no,lunix_minor_cnt,"TNG:lunix");
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}	
	/* ? */
	/* cdev_add? */
	ret = cdev_add(&lunix_chrdev_cdev,dev_no,lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
		
	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
