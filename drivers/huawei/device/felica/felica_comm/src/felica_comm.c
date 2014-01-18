/**
 * @file felica.c
 * @brief FeliCa Driver
 *
 * @date 2012/01/19
 *
 */
/* =============================================================================
 * include
 * ========================================================================== */
#include <linux/err.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/param.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/io.h>

#include <asm/uaccess.h>
#include <asm/ioctls.h>

#include <linux/module.h>
#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/termios.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>

#include "../inc/felica_comm_local.h"
#include <hsad/config_interface.h> 

#include <linux/notifier.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <linux/semaphore.h>
#define FELICA_UART_ON		0x0001  
#define FELICA_UART_OFF		0x0002
static struct semaphore felica_sem;

/* -------------------------------------------------------------------------- */
/* Global                                                                     */
/* -------------------------------------------------------------------------- */
struct cdev g_cdev_fcomm;
static spinlock_t timer_lock;

/* -------------------------------------------------------------------------- */
/* prottype                                                                   */
/* -------------------------------------------------------------------------- */
static int __init FeliCaDD_init( void );
static void __exit FeliCaDD_exit( void );
static int FeliCaDD_open(struct inode *, struct file *);
static int FeliCaDD_close(struct inode *, struct file *);
static ssize_t FeliCaDD_write(struct file *, const char *, size_t, loff_t *);
static ssize_t FeliCaDD_read(struct file *, char *, size_t, loff_t *);
static long FeliCaDD_ioctl( struct file * , unsigned int, unsigned long);
static int FeliCaDD_fsync(struct file*, int);
static int FeliCaDD_priv_comm_open(struct inode *, struct file *);
static int FeliCaDD_priv_comm_close(struct inode *, struct file *);

static int  FeliCaDD_priv_comm_uart_open(void);
static void FeliCaDD_priv_comm_uart_close(void);
static int  FeliCaDD_priv_comm_uart_write(const char *buff, size_t count);
static int  FeliCaDD_priv_comm_uart_read(char *buff, size_t count);



static struct file_operations felica_fops = {
    .owner=   THIS_MODULE,
    .read=    FeliCaDD_read,
    .write=   FeliCaDD_write,
    .unlocked_ioctl=   FeliCaDD_ioctl,
    .fsync=   FeliCaDD_fsync,
    .open=    FeliCaDD_open,
    .release= FeliCaDD_close
};

/* -----------------------------------------------------------------------------
 * UART Access
 * -------------------------------------------------------------------------- */
#define UART_TTY_READBUFF_SIZE	(30*1024)

static unsigned char *uart_tty_readbuff = 0;
static int uart_tty_read_size = 0;
static int uart_tty_read_size_user = 0;
static mm_segment_t uart_tty_oldfs;
struct file *uart_tty_filp;
struct inode *uart_tty_inode = NULL;
static int uart_tty_open_count = 0;

#define TTY_FILE_PATH "/dev/ttyHSL2"

static int FeliCaDD_priv_comm_uart_open()
{
	mm_segment_t	oldfs;
	struct file		*filp;
	struct inode	*inode = NULL;
	struct termios options;


	oldfs = get_fs();
	set_fs(KERNEL_DS);	
	filp = filp_open(TTY_FILE_PATH, O_RDWR, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

	if (IS_ERR(filp)) {
		printk("filp_open error\n");
		set_fs(oldfs);
		return -1;
	}

	inode = filp->f_path.dentry->d_inode;
	if (!filp->f_op || !inode) {
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}	
	if (filp->f_op->unlocked_ioctl(filp, (unsigned int)TCFLSH, (unsigned long)TCIOFLUSH) < 0) {
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	if (filp->f_op->unlocked_ioctl(filp, (unsigned int)TCGETS, (unsigned long)&options) < 0) {
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= (CS8 | CLOCAL | CREAD | CRTSCTS);
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cflag = (options.c_cflag & ~CBAUD) | (B460800 & CBAUD);
	options.c_cflag = (options.c_cflag & ~CBAUD) | (B460800 & CBAUD);
	if (filp->f_op->unlocked_ioctl(filp, (unsigned int)TCSETS, (unsigned long)&options) < 0) {
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	uart_tty_oldfs = oldfs;
	uart_tty_filp = filp;
	uart_tty_inode = inode;	
	return 0;
}

static void FeliCaDD_priv_comm_uart_close()
{

	filp_close(uart_tty_filp, NULL);
	set_fs(uart_tty_oldfs);
	uart_tty_filp = NULL;

}


static int FeliCaDD_priv_comm_uart_write(const char *buff, size_t count)
{
	struct file		*filp;
	int			ret = 0;

	if (NULL == buff) {
		printk("parameter error\n");
		return -1;
	}
	
	if (count == 0) {
		printk("count 0\n");
		return 0;
	}	
	filp = uart_tty_filp;

	filp->f_pos = 0;
	ret = filp->f_op->write(filp, buff, count, &filp->f_pos);
	if (ret < count) {
		printk("write error %d\n", ret);
		return -1;
	}	
	msleep(10);
	printk("ret: %d\n", ret);
	return ret;
}

static int FeliCaDD_priv_comm_uart_read(char *buff, size_t count)
{
	struct file		*filp;
	int				ret = 0;


	if (NULL == buff) {
		printk("parameter error\n");
		return -1;
	}

	if (count == 0){
		printk("count 0\n");
		return 0;
	}	
	filp = uart_tty_filp;

	filp->f_pos = 0;
	ret = filp->f_op->read(filp, buff, count, &filp->f_pos);
	if (ret < 0) {
		printk("read error %d\n", ret);
		return ret;
	}	
	msleep(10);	
	printk("ret: %d\n", ret);
	return ret;

}


static int FeliCaDD_priv_comm_open(struct inode *pinode, struct file *pfile)
{

    if(( pfile == NULL ) || ( pinode == NULL )) {
        return -EINVAL;
    }


	if ( uart_tty_open_count == 0 ) {
		printk("First open\n");		
		down(&felica_sem);		
		if ( FeliCaDD_priv_comm_uart_open() < 0 ) {
			return -EACCES;
		}
		up(&felica_sem);
		uart_tty_readbuff = vmalloc(UART_TTY_READBUFF_SIZE);
		uart_tty_read_size = 0;
		uart_tty_read_size_user = 0;
	}

	uart_tty_open_count++;

    return 0;	
}


static int FeliCaDD_priv_comm_close(struct inode *pinode, struct file *pfile)
{

    if( ( pfile == NULL ) || ( pinode == NULL )) {
        return -EINVAL;
    }


	uart_tty_open_count--;

	if ( uart_tty_open_count == 0 ) {
		printk("Last close\n");		
		down(&felica_sem);		
		FeliCaDD_priv_comm_uart_close();
		up(&felica_sem);
		vfree(uart_tty_readbuff);
		uart_tty_read_size = 0;
		uart_tty_read_size_user = 0;
	}

    return 0;
}


static int FeliCaDD_open(struct inode *pinode, struct file  *pfile)
{
    int ret             = 0;
    unsigned char minor = 0;


    if(( pfile == NULL ) || ( pinode == NULL )) {
        return -EINVAL;
    }

    minor = MINOR(pinode->i_rdev);

    switch( minor )
    {
        case FELICA_COMM_MINOR_NO_COMM:
            ret = FeliCaDD_priv_comm_open(pinode, pfile);
            break;
        default:
            ret = -ENODEV;
            break;
    }

    return ret;
}


static int FeliCaDD_close(struct inode *pinode, struct file *pfile)
{
    int ret             = 0;
    unsigned char minor = 0;


    if(( pfile == NULL ) || ( pinode == NULL )) {
        return -EINVAL;
    }

    minor = MINOR(pinode->i_rdev);

    switch( minor )
    {
        case FELICA_COMM_MINOR_NO_COMM:
            ret = FeliCaDD_priv_comm_close(pinode, pfile);
            break;
        default:
            ret = -ENODEV;
            break;
    }

    return ret;
}


static ssize_t FeliCaDD_write(
                           struct file *pfile
                           ,const char  *buff
                           ,size_t      count
                           ,loff_t      *poff )
{
    ssize_t          ret = 0;
    unsigned char    minor = 0;
    unsigned char        local_buff[FELICA_COMM_WR_BUFF_SIZE];
    int                  remain_size = 0;
    int                  copy_size = 0;
    int                  written_size = 0;

	printk("FeliCaDD_write(%d)\n", count);

    if ( pfile == NULL ) {
        return -EINVAL;
    }
    if (( buff == NULL )
     || ( count > FELICA_COMM_BUFF_SIZE )) {
        return -EINVAL;
    }

    minor = MINOR(pfile->f_dentry->d_inode->i_rdev);

    if(FELICA_COMM_MINOR_NO_COMM == minor) {
    	if ( count == 0 ) {
    		printk("count 0\n");
    		return 0;
    	}

    	remain_size = count;
    	written_size = 0;
    	while ( remain_size > 0 )
    	{
    		if ( remain_size > FELICA_COMM_WR_BUFF_SIZE ) {
    			copy_size = FELICA_COMM_WR_BUFF_SIZE;
    		}
    		else {
	    		copy_size = remain_size;
    		}
    		
	    	if (copy_from_user(
    	        local_buff,
        	    ( void * )( buff + written_size ),
            	( unsigned int )copy_size ) == 0) {				
				down(&felica_sem);				
    		    FeliCaDD_priv_comm_uart_write((const char *)local_buff, (size_t)copy_size);
				up(&felica_sem);
    		    written_size += copy_size;
   		        remain_size -= copy_size;
    	    }
    		else {
    			printk("copy_from_user error\n");
    			break;
    		}
        }
    	ret = written_size;
    }
    else {
        ret = (ssize_t)(-ENODEV);
    }

    return ret;
}


static ssize_t FeliCaDD_read(
                           struct file *pfile,
                           char        *buff,
                           size_t      count,
                           loff_t      *poff )
{
    ssize_t              ret = 0;
    unsigned char        minor = 0;
	int read_size = 0;
	int returnable_size = 0;
	int return_size = 0;
        int actual_size = 0;
        int remain_size = 0;

	printk("FeliCaDD_read(%d)\n", count);

    if ( pfile == NULL ) {
        return -EINVAL;
    }

    if (( buff == NULL )
     || ( count > FELICA_COMM_BUFF_SIZE )) {
        return -EINVAL;
    }

    minor = MINOR(pfile->f_dentry->d_inode->i_rdev);

    if(FELICA_COMM_MINOR_NO_COMM == minor){		
		down(&felica_sem);		
    	read_size = FeliCaDD_priv_comm_uart_read(
    		(char *)(uart_tty_readbuff + uart_tty_read_size), (UART_TTY_READBUFF_SIZE - uart_tty_read_size) );
		up(&felica_sem);
    	uart_tty_read_size += read_size;

    	returnable_size = uart_tty_read_size - uart_tty_read_size_user;
    	
    	if ( returnable_size == 0 ) {
    		printk("returnable_size 0\n");
    		return 0;
    	}

    	if ( returnable_size >= count ) {
    		return_size = count;
    	}
    	else {
    		return_size = returnable_size;
    	}

        remain_size = copy_to_user(
        	(void *)buff,
        	(void *)(uart_tty_readbuff + uart_tty_read_size_user),
        	(unsigned int)return_size);

		actual_size = return_size - remain_size;
		uart_tty_read_size_user += actual_size;

    	if ( uart_tty_read_size == uart_tty_read_size_user ) {
    		printk("buffer clear\n");
    		uart_tty_read_size = 0;
    		uart_tty_read_size_user = 0;
    	}

    	ret = actual_size;
    	printk("uart_tty_read_size: %d\n", uart_tty_read_size);
    	printk("uart_tty_read_size_user: %d\n", uart_tty_read_size_user);
    	printk("ret: %d\n", ret);
    }

    else {
        ret = (ssize_t)(-ENODEV);
    }

    return ret;
}


static long FeliCaDD_ioctl(struct file   *pfile, unsigned int  cmd, unsigned long arg )
{
    int           ret   = 0;
    int           ret_sub,ret_cpy;
    unsigned char minor = 0;
    int           read_size = 0;



    if( NULL == pfile ) {
        return -EINVAL;
    }


    minor = FELICA_COMM_MINOR_NO_COMM;


    switch( minor )
    {

        case FELICA_COMM_MINOR_NO_COMM:
            if( cmd == FIONREAD ) {				
				down(&felica_sem);				
		    	read_size = FeliCaDD_priv_comm_uart_read(
		    		(char *)(uart_tty_readbuff + uart_tty_read_size), (UART_TTY_READBUFF_SIZE - uart_tty_read_size) );
				up(&felica_sem);
		    	uart_tty_read_size += read_size;
		    	ret_sub = uart_tty_read_size - uart_tty_read_size_user;
            	ret_cpy = copy_to_user( (void*)arg, &ret_sub, sizeof(int) );
                if( ret_cpy != 0 ) {
                    ret = -EFAULT;
                }
            }
            else{
                ret = -ENOSYS;
            }
            break;
        default:
            ret = -ENODEV;
            break;
    }

    return ret;
}


static int FeliCaDD_fsync( struct file *pfile, int datasync)
{
    int                     ret = 0;
    unsigned char           minor;


    if( NULL == pfile ){
        return -EINVAL;
    }

    minor = MINOR(pfile->f_dentry->d_inode->i_rdev);

    switch( minor )
    {
        case FELICA_COMM_MINOR_NO_COMM:
            break;
        default:
            ret = -ENODEV;
            break;
    }

    return ret;
}

extern int unregister_felica_notifier(struct notifier_block *nb);
extern int register_felica_notifier(struct notifier_block *nb);

static struct gpiomux_setting felica_uart_tx_gpio_active_cfg1 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting felica_uart_tx_gpio_suspend_cfg1 = {

	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting felica_uart_rx_gpio_active_cfg1 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting felica_uart_rx_gpio_suspend_cfg1 = {

	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,	
};

static struct msm_gpiomux_config  msm8960_felica_uart_gpio_configs1[]  = {
	{
		.gpio = 6,
		.settings = {
			[GPIOMUX_ACTIVE]    = &felica_uart_tx_gpio_active_cfg1,
			[GPIOMUX_SUSPENDED] = &felica_uart_tx_gpio_suspend_cfg1,
		},
	},
	{
		.gpio = 7,
		.settings = {
			[GPIOMUX_ACTIVE]    = &felica_uart_rx_gpio_active_cfg1,
			[GPIOMUX_SUSPENDED] = &felica_uart_rx_gpio_suspend_cfg1,
		},
	},
};

static struct gpiomux_setting felica_uart_tx_gpio_active_cfg2 = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting felica_uart_tx_gpio_suspend_cfg2 = {

	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting felica_uart_rx_gpio_active_cfg2 = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting felica_uart_rx_gpio_suspend_cfg2 = {

	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct msm_gpiomux_config  msm8960_felica_uart_gpio_configs2[]  = {
	{
		.gpio = 6,
		.settings = {
			[GPIOMUX_ACTIVE]    = &felica_uart_tx_gpio_active_cfg2,
			[GPIOMUX_SUSPENDED] = &felica_uart_tx_gpio_suspend_cfg2,
		},
	},
	{
		.gpio = 7,
		.settings = {
			[GPIOMUX_ACTIVE]    = &felica_uart_rx_gpio_active_cfg2,
			[GPIOMUX_SUSPENDED] = &felica_uart_rx_gpio_suspend_cfg2,
		},
	},
};

static int felica_notifier_call(struct notifier_block *nb,
				  unsigned long code, void *_param)
{
	struct file	*filp;
	int	ret = 0;
	char buff[10];
	filp = uart_tty_filp;
	switch (code) {
	case FELICA_UART_ON:
		msm_gpiomux_install(msm8960_felica_uart_gpio_configs1,
							ARRAY_SIZE(msm8960_felica_uart_gpio_configs1));
		mdelay(1);		
		down(&felica_sem);		
		if (filp != NULL) {
			filp->f_pos = 0;
			ret = filp->f_op->read(filp, buff, 1, &filp->f_pos);
			if (ret < 0) {
				printk("read error %d\n", ret);
				return ret;
			}		
		}
		up(&felica_sem);		
		break;
	case FELICA_UART_OFF:
		msm_gpiomux_install(msm8960_felica_uart_gpio_configs2,
							ARRAY_SIZE(msm8960_felica_uart_gpio_configs2));		
		mdelay(1);
		break;
	}
	return 0;
}

static struct notifier_block felica_notifier_block = {
	.notifier_call = felica_notifier_call,
};

static int __init FeliCaDD_init( void )
{
    int ret;
	int err;
	if(!is_felica_exist())
		return -ENODEV;
    spin_lock_init(&timer_lock);

    ret = register_chrdev_region(
                MKDEV(FELICA_COMM_MAJOR_NO, FELICA_COMM_MINOR_NO_COMM),
                FELICA_DEV_NUM,
                FELICA_DEV_NAME_COMM);
    if (ret < 0) {
    }
    else{
        cdev_init(&g_cdev_fcomm, &felica_fops);
        g_cdev_fcomm.owner = THIS_MODULE;
        ret = cdev_add(
                &g_cdev_fcomm,
                MKDEV(FELICA_COMM_MAJOR_NO, FELICA_COMM_MINOR_NO_COMM),
                FELICA_DEV_NUM);
        if (ret < 0) {
        }
    }
	err = register_felica_notifier(&felica_notifier_block);
	sema_init(&felica_sem, 1);
    return ret;
}


static void __exit FeliCaDD_exit( void )
{
	int err;
    cdev_del(&g_cdev_fcomm);
    unregister_chrdev_region(
        MKDEV(FELICA_COMM_MAJOR_NO, FELICA_COMM_MINOR_NO_COMM),
        FELICA_DEV_NUM);
	err = unregister_felica_notifier(&felica_notifier_block);
    return;
}


 module_init( FeliCaDD_init );
 module_exit( FeliCaDD_exit );
/******************************************************************************/
/* Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved             */
/******************************************************************************/
