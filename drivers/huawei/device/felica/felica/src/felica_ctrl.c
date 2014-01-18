/*
 * @file felica.c
 * @brief FeliCa code
 */

/*
 * include files
 */
#include <linux/err.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/param.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/vmalloc.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>

#include <linux/mfd/pm8xxx/gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include <asm/uaccess.h>
#include <asm/ioctls.h>

#include "../common/felica_ctrl.h"
#include "../inc/felica_ctrl_local.h"

#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/termios.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>

#include <linux/notifier.h>
#define FELICA_UART_ON		0x0001  
#define FELICA_UART_OFF		0x0002

#include <hsad/config_interface.h> 

/*
 * Function prottype defined
 */
/* External if */
 static int __init FeliCaCtrl_init(void);
 static void __exit FeliCaCtrl_exit(void);
 static int FeliCaCtrl_open(struct inode *pinode, struct file *pfile);
 static int FeliCaCtrl_close(struct inode *pinode, struct file *pfile);
 static unsigned int FeliCaCtrl_poll(struct file *pfile,  struct poll_table_struct *pwait);
 static ssize_t FeliCaCtrl_write(struct file *pfile,  const char *buff, size_t count, loff_t *poff);
 static ssize_t FeliCaCtrl_read(struct file *pfile, char *buff, size_t count, loff_t *poff);

/* Interrupt handler */
 static irqreturn_t FeliCaCtrl_interrupt_INT(int i_req, void *pv_devid);
 static void FeliCaCtrl_timer_hnd_general(unsigned long p);

/* Internal function */
 static int FeliCaCtrl_priv_ctrl_enable(struct FELICA_TIMER *pTimer);
 static int FeliCaCtrl_priv_ctrl_unenable(struct FELICA_TIMER* pPonTimer, struct FELICA_TIMER* pCenTimer);
 static int FeliCaCtrl_priv_ctrl_online(struct FELICA_TIMER *pTimer);
 static int FeliCaCtrl_priv_ctrl_offline(void);
 static int FeliCaCtrl_priv_terminal_cen_read(unsigned char *cen_data);
 static int FeliCaCtrl_priv_terminal_rfs_read(unsigned char *rfs_data);
 static int FeliCaCtrl_priv_terminal_int_read(unsigned char *int_data);
 static int FeliCaCtrl_priv_terminal_cen_write(unsigned char cen_data);
 static int FeliCaCtrl_priv_terminal_pon_write(unsigned char pon_data);
 static int FeliCaCtrl_priv_interrupt_ctrl_init(unsigned char itr_type);
 static void FeliCaCtrl_priv_interrupt_ctrl_exit(unsigned char itr_type);
 static int FeliCaCtrl_priv_interrupt_ctrl_regist(unsigned char itr_type);
 static void FeliCaCtrl_priv_interrupt_ctrl_unregist(unsigned char itr_type);
 static void FeliCaCtrl_priv_drv_timer_data_init(struct FELICA_TIMER *pTimer);
 static void FeliCaCtrl_priv_drv_init_cleanup(int cleanup);
 static int FeliCaCtrl_priv_dev_pon_open(struct inode *pinode, struct file *pfile);
 static int FeliCaCtrl_priv_dev_cen_open(struct inode *pinode, struct file *pfile);
 static int FeliCaCtrl_priv_dev_rfs_open(struct inode *pinode, struct file *pfile);
 static int FeliCaCtrl_priv_dev_itr_open(struct inode *pinode, struct file *pfile);
 static int FeliCaCtrl_priv_dev_pon_close(struct inode *pinode, struct file *pfile);
 static int FeliCaCtrl_priv_dev_cen_close(struct inode *pinode, struct file *pfile);
 static int FeliCaCtrl_priv_dev_rfs_close(struct inode *pinode, struct file *pfile);
 static int FeliCaCtrl_priv_dev_itr_close(struct inode *pinode, struct file *pfile);
 static void FeliCaCtrl_priv_alarm(unsigned long err_id);

/*
 * static paramater
 */
static struct class *felica_class;

static struct FELICA_DEV_INFO gPonCtrl;
static struct FELICA_DEV_INFO gCenCtrl;
static struct FELICA_DEV_INFO gRfsCtrl;
static struct FELICA_DEV_INFO gItrCtrl;

static spinlock_t itr_lock;
static unsigned long itr_lock_flag;

struct cdev g_cdev_fpon;
struct cdev g_cdev_fcen;
struct cdev g_cdev_frfs;
struct cdev g_cdev_fitr;

static BLOCKING_NOTIFIER_HEAD(felica_notifier_list);

int register_felica_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&felica_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(register_felica_notifier);

int unregister_felica_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&felica_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(unregister_felica_notifier);

static struct FELICA_ITR_INFO gFeliCaItrInfo[DFD_ITR_TYPE_NUM] = {
	{
		.irq=     0,
		.handler= NULL,
		.flags=   IRQF_DISABLED|IRQF_TRIGGER_FALLING,
		.name=    FELICA_CTRL_ITR_STR_RFS,
		.dev=     NULL
	},
	{
		.irq=     0,
		.handler= FeliCaCtrl_interrupt_INT,
		.flags=   IRQF_DISABLED|IRQF_TRIGGER_FALLING,
		.name=    FELICA_CTRL_ITR_STR_INT,
		.dev=     NULL
	}
};

/*
 * sturut of reginsting External if
 */
static struct file_operations felica_fops = {
	.owner=   THIS_MODULE,
	.poll=    FeliCaCtrl_poll,
	.read=    FeliCaCtrl_read,
	.write=   FeliCaCtrl_write,
	.open=    FeliCaCtrl_open,
	.release= FeliCaCtrl_close
};


static int FeliCaCtrl_priv_pm8xxx_gpio_write(int, int);


static int FeliCaCtrl_priv_gpio_clk_pm_init(void)
{
	int rc = 0;
	int gpio_no = PM8921_GPIO_PM_TO_SYS(FELICA_GPIO_PORT_CLK_PD);

	struct pm_gpio param = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull	        = PM_GPIO_PULL_NO,
		.vin_sel	    = PM_GPIO_VIN_S4,
		.out_strength   = PM_GPIO_STRENGTH_NO,
		.function       = PM_GPIO_FUNC_NORMAL,
	};

	rc = gpio_request(gpio_no, FELICA_GPIO_STR_CEN_CLK);
	if (rc) 
	{
		pr_err("%s: Failed to request gpio %d\n", __func__, gpio_no);
		return rc;
	}
	rc = pm8xxx_gpio_config(gpio_no, &param);
	if (rc)
	{
		pr_err("%s: Failed to configure gpio %d\n", __func__, gpio_no);
	}
	else
	{
		rc = gpio_direction_output(gpio_no, 0);
	}
	return rc;
}


static int FeliCaCtrl_priv_gpio_cen_pm_init(void)
{
	int rc = 0;
	int gpio_no = PM8921_GPIO_PM_TO_SYS(FELICA_GPIO_PORT_CEN_PM);

	struct pm_gpio param = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull	        = PM_GPIO_PULL_NO,
		.vin_sel	    = PM_GPIO_VIN_S4,
		.out_strength   = PM_GPIO_STRENGTH_NO,
		.function       = PM_GPIO_FUNC_NORMAL,
	};

	rc = gpio_request(gpio_no, FELICA_GPIO_STR_CEN);
	if (rc) 
	{
		pr_err("%s: Failed to request gpio %d\n", __func__, gpio_no);
		return rc;
	}

	param.output_value = gpio_get_value_cansleep(gpio_no);

	rc = pm8xxx_gpio_config(gpio_no, &param);
	if (rc)
	{
		pr_err("%s: Failed to configure gpio %d\n", __func__, gpio_no);
	}
	else
	{
		rc = gpio_direction_output(gpio_no, param.output_value);
	}

	return rc;
}


static int FeliCaCtrl_priv_gpio_request(int gpio_no, const unsigned char *name)
{
	int rc = 0;

	rc = gpio_request(gpio_no, name);
	if (rc) {
		pr_err("%s: Failed to request gpio %d\n", __func__, gpio_no);
		return rc;
	}

	rc = gpio_direction_input(gpio_no);
	if (rc) {
		pr_err("%s: Failed to direction_input gpio %d\n", __func__, gpio_no);
	}

	return rc;
}


static int FeliCaCtrl_priv_gpio_pon_request(int gpio_no, const unsigned char *name)
{
	int rc = 0;

	rc = gpio_request(gpio_no, name);
	if (rc) {
		pr_err("%s: Failed to request gpio %d\n", __func__, gpio_no);
		return rc;
	}

	rc = gpio_direction_output(gpio_no, 0);
	if (rc) {
		pr_err("%s: Failed to direction_input gpio %d\n", __func__, gpio_no);
	}

	return rc;
}


static int FeliCaCtrl_priv_gpio_init(void)
{
	int ret = 0;

	ret = FeliCaCtrl_priv_gpio_clk_pm_init();
	if (ret) {
		return ret;
	}

	ret = FeliCaCtrl_priv_gpio_cen_pm_init();
	if (ret) {
		return ret;
	}

	ret = FeliCaCtrl_priv_pm8xxx_gpio_write(
			PM8921_GPIO_PM_TO_SYS(FELICA_GPIO_PORT_CLK_PD), 1);
	if (ret) {
		return ret;
	}

	ret = FeliCaCtrl_priv_gpio_pon_request(
				FELICA_GPIO_PORT_PON
				, FELICA_GPIO_STR_PON);
	if (ret) {
		return ret;
	}

	ret = FeliCaCtrl_priv_gpio_request(
				FELICA_GPIO_PORT_RFS
				, FELICA_GPIO_STR_RFS);
	if (ret) {
		return ret;
	}

	ret = FeliCaCtrl_priv_gpio_request(
				FELICA_GPIO_PORT_INT
				, FELICA_GPIO_STR_INT);
	if (ret) {
		return ret;
	}

	return ret;
}


static void FeliCaCtrl_priv_gpio_exit(void)
{

	gpio_free(PM8921_GPIO_PM_TO_SYS(FELICA_GPIO_PORT_CLK_PD));
	gpio_free(PM8921_GPIO_PM_TO_SYS(FELICA_GPIO_PORT_CEN_PM));
	gpio_free(FELICA_GPIO_PORT_PON);
	gpio_free(FELICA_GPIO_PORT_RFS);
	gpio_free(FELICA_GPIO_PORT_INT);
	return;
}


static int FeliCaCtrl_priv_pm8xxx_gpio_write(int gpio_no, int data)
{
	int rc = 0;
	struct pm_gpio param = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.pull	        = PM_GPIO_PULL_NO,
		.vin_sel	    = PM_GPIO_VIN_VPH,
		.out_strength   = PM_GPIO_STRENGTH_NO,
		.function       = PM_GPIO_FUNC_NORMAL,
	};

	param.output_value = data;

	rc = pm8xxx_gpio_config(gpio_no, &param);
	if (rc) {
		pr_err("%s: Failed to configure gpio %d\n", __func__, gpio_no);
	}
	else {
		rc = gpio_direction_output(gpio_no, data);
	}
	return rc;
}


static int FeliCaCtrl_priv_pm8xxx_gpio_read(int gpio_no, int *data)
{
	int rc = 0;
	int rc_value;

	rc_value = gpio_get_value_cansleep(gpio_no);
	*data = rc_value;

	return rc;
}
struct device *pon_device = NULL;
struct regulator *lvs1;
struct regulator *l16;

static int felica_set_lvs1(void)
{

	int ret;

	lvs1 = regulator_get(pon_device, "felica_lvs1");
	if (IS_ERR(lvs1)) {
		ret = PTR_ERR(lvs1);
		return ret;
	}

	regulator_enable(lvs1);

	msleep(10);
	return 0;
}


static int felica_set_l16(void)
{
	l16 = regulator_get(pon_device, "felica_l16");
	printk("ENTER %s config power 8921_l16\n",__func__);
	regulator_set_voltage(l16, 3000000,3000000);
	regulator_enable(l16);
	return 0;
}


static int felica_reset_lvs1(void)
{

	regulator_disable(lvs1);
	regulator_put(lvs1);

	msleep(1);
	return 0;
}

static int felica_reset_l16(void)
{
	regulator_disable(l16);
	regulator_put(l16);
	
	return 0;
}


static int __init FeliCaCtrl_init(void)
{
	int ret = 0;
	int cleanup = DFD_CLUP_NONE;
	struct device *devt;
	if(!is_felica_exist())
		return -ENODEV;
	memset(&gPonCtrl, 0x00, sizeof(gPonCtrl));
	memset(&gCenCtrl, 0x00, sizeof(gCenCtrl));
	memset(&gRfsCtrl, 0x00, sizeof(gRfsCtrl));
	memset(&gItrCtrl, 0x00, sizeof(gItrCtrl));

	ret = FeliCaCtrl_priv_gpio_init();
	if (ret < 0) {
		return ret;
	}


	FeliCaCtrl_priv_terminal_cen_write(DFD_OUT_H);


	ret = FeliCaCtrl_priv_interrupt_ctrl_init(DFD_ITR_TYPE_INT);
	if (ret < 0) {
	}

	do {
		ret = register_chrdev_region(
					MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_PON),
					FELICA_DEV_NUM,
					FELICA_DEV_NAME_CTRL_PON);
		if (ret < 0) {
			break;
		}
		cdev_init(&g_cdev_fpon, &felica_fops);
		g_cdev_fpon.owner = THIS_MODULE;
		ret = cdev_add(
				&g_cdev_fpon,
				MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_PON),
				FELICA_DEV_NUM);
		if (ret < 0) {
			cleanup = DFD_CLUP_UNREG_CDEV_PON;
			break;
		}
		ret = register_chrdev_region(
					MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_CEN),
					FELICA_DEV_NUM,
					FELICA_DEV_NAME_CTRL_CEN);
		if (ret < 0) {
			cleanup = DFD_CLUP_CDEV_DEL_PON;
			break;
		}
		cdev_init(&g_cdev_fcen, &felica_fops);
		g_cdev_fcen.owner = THIS_MODULE;
		ret = cdev_add(
				&g_cdev_fcen,
				MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_CEN),
				FELICA_DEV_NUM);
		if (ret < 0) {
			cleanup = DFD_CLUP_UNREG_CDEV_CEN;
			break;
		}
		ret = register_chrdev_region(
					MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_RFS),
					FELICA_DEV_NUM,
					FELICA_DEV_NAME_CTRL_RFS);
		if (ret < 0) {
			cleanup = DFD_CLUP_CDEV_DEL_CEN;
			break;
		}
		cdev_init(&g_cdev_frfs, &felica_fops);
		g_cdev_frfs.owner = THIS_MODULE;
		ret = cdev_add(
				&g_cdev_frfs,
				MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_RFS),
				FELICA_DEV_NUM);
		if (ret < 0) {
			cleanup = DFD_CLUP_UNREG_CDEV_RFS;
			break;
		}
		ret = register_chrdev_region(
					MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_ITR),
					FELICA_DEV_NUM,
					FELICA_DEV_NAME_CTRL_ITR);
		if (ret < 0) {
			cleanup = DFD_CLUP_CDEV_DEL_RFS;
			break;
		}
		cdev_init(&g_cdev_fitr, &felica_fops);
		g_cdev_fitr.owner = THIS_MODULE;
		ret = cdev_add(
				&g_cdev_fitr,
				MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_ITR),
				FELICA_DEV_NUM);
		if (ret < 0) {
			cleanup = DFD_CLUP_UNREG_CDEV_ITR;
			break;
		}

		felica_class = class_create(THIS_MODULE, FELICA_DEV_NAME);
		if (IS_ERR(felica_class)) {
			ret = PTR_ERR(felica_class);
			cleanup = DFD_CLUP_CDEV_DEL_ITR;
			break;
		}

		devt = device_create(
			felica_class,
			NULL,
			MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_PON),
			NULL,
			FELICA_DEV_NAME_CTRL_PON);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_CLASS_DESTORY;
			break;
		}
		pon_device = devt;
		devt = device_create(
				felica_class,
				NULL,
				MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_CEN),
				NULL,
				FELICA_DEV_NAME_CTRL_CEN);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_DEV_DESTORY_PON;
			break;
		}
		devt = device_create(
				felica_class,
				NULL,
				MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_RFS),
				NULL,
				FELICA_DEV_NAME_CTRL_RFS);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_DEV_DESTORY_CEN;
			break;
		}
		devt = device_create(
				felica_class,
				NULL,
				MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_ITR),
				NULL,
				FELICA_DEV_NAME_CTRL_ITR);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_DEV_DESTORY_RFS;
			break;
		}

		devt = device_create(
				felica_class,
				NULL,
				MKDEV(FELICA_COMM_MAJOR_NO, FELICA_COMM_MINOR_NO_COMM),
				NULL,
				FELICA_DEV_NAME_COMM);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_DEV_DESTORY_ITR;
			break;
		}

		devt = device_create(
				felica_class,
				NULL,
				MKDEV(FELICA_RWS_MAJOR_NO, FELICA_RWS_MINOR_NO_RWS),
				NULL,
				FELICA_DEV_NAME_RWS);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_DEV_DESTORY_COMM;
			break;
		}

		devt = device_create(
				felica_class,
				NULL,
				MKDEV(FELICA_CFG_MAJOR_NO, FELICA_CFG_MINOR_NO_CFG),
				NULL,
				FELICA_DEV_NAME_CFG);
		if (IS_ERR(devt)) {
			ret = PTR_ERR(devt);
			cleanup = DFD_CLUP_DEV_DESTORY_COMM;
			break;
		}
	} while(0);

	FeliCaCtrl_priv_drv_init_cleanup(cleanup);

	return ret;
}
module_init(FeliCaCtrl_init);


static void __exit FeliCaCtrl_exit(void)
{


	FeliCaCtrl_priv_drv_init_cleanup(DFD_CLUP_ALL);

	FeliCaCtrl_priv_interrupt_ctrl_exit(DFD_ITR_TYPE_INT);

	FeliCaCtrl_priv_gpio_exit();

	return;
}
module_exit(FeliCaCtrl_exit);


static int FeliCaCtrl_open(struct inode *pinode, struct file *pfile)
{
	int ret = 0;
	unsigned char minor = 0;


	if ((pinode == NULL) || (pfile == NULL)) {
		return -EINVAL;
	}

	minor = MINOR(pinode->i_rdev);

	switch (minor) {
	case FELICA_CTRL_MINOR_NO_PON:
		ret = FeliCaCtrl_priv_dev_pon_open(pinode, pfile);
		break;
	case FELICA_CTRL_MINOR_NO_CEN:
		ret = FeliCaCtrl_priv_dev_cen_open(pinode, pfile);
		break;
	case FELICA_CTRL_MINOR_NO_RFS:
		ret = FeliCaCtrl_priv_dev_rfs_open(pinode, pfile);
		break;
	case FELICA_CTRL_MINOR_NO_ITR:
		ret = FeliCaCtrl_priv_dev_itr_open(pinode, pfile);
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}


static int FeliCaCtrl_close(struct inode *pinode, struct file *pfile)
{
	int ret = 0;
	unsigned char minor = 0;


	if ((pinode == NULL) || (pfile == NULL)) {
		return -EINVAL;
	}

	minor = MINOR(pinode->i_rdev);

	switch (minor) {
	case FELICA_CTRL_MINOR_NO_PON:
		ret = FeliCaCtrl_priv_dev_pon_close(pinode, pfile);
		break;
	case FELICA_CTRL_MINOR_NO_CEN:
		ret = FeliCaCtrl_priv_dev_cen_close(pinode, pfile);
		break;
	case FELICA_CTRL_MINOR_NO_RFS:
		ret = FeliCaCtrl_priv_dev_rfs_close(pinode, pfile);
		break;
	case FELICA_CTRL_MINOR_NO_ITR:
		ret = FeliCaCtrl_priv_dev_itr_close(pinode, pfile);
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}


static unsigned int FeliCaCtrl_poll(
							struct file *pfile,
							struct poll_table_struct *pwait)
{
	unsigned int ret = 0;
	unsigned char minor = 0;
    struct FELICA_CTRLDEV_ITR_INFO *pItr = NULL;


	if ((pfile == NULL) || (pwait == NULL)) {
		ret = (POLLHUP | POLLERR);
		return ret;
	}

	minor = MINOR(pfile->f_dentry->d_inode->i_rdev);

	switch (minor) {
	case FELICA_CTRL_MINOR_NO_ITR:
		pItr = (struct FELICA_CTRLDEV_ITR_INFO*)gItrCtrl.Info;
		if (NULL == pItr) {
			ret = (POLLHUP | POLLERR);
			break;
		}
		poll_wait(pfile, &pItr->RcvWaitQ, pwait);

		if (FELICA_EDGE_NON != pItr->INT_Flag) {
			ret |= (POLLIN  | POLLRDNORM);
		}
		if (FELICA_EDGE_NON != pItr->RFS_Flag) {
			ret |= (POLLIN  | POLLRDNORM);
		}

		ret |= (POLLOUT | POLLWRNORM);
		break;
	default:
		ret = (POLLHUP | POLLERR);
		break;
	}

	return ret;
}


static ssize_t FeliCaCtrl_write(
							struct file  *pfile,
							const char   *buff,
							size_t        count,
							loff_t       *poff)
{
	ssize_t ret = 0;
	unsigned char minor = 0;
	unsigned long ret_cpy = 0;
	int ret_sub = 0;
	struct FELICA_CTRLDEV_PON_INFO *pPonCtrl = NULL;
	struct FELICA_CTRLDEV_CEN_INFO *pCenCtrl = NULL;

	unsigned char local_buff[FELICA_OUT_SIZE];


	if ((pfile == NULL) || (buff == NULL)) {
		return -EINVAL;
	}

	minor = MINOR(pfile->f_dentry->d_inode->i_rdev);

	switch (minor) {
	case FELICA_CTRL_MINOR_NO_PON:
		pPonCtrl = (struct FELICA_CTRLDEV_PON_INFO*)gPonCtrl.Info;
		if (NULL == pPonCtrl) {
			return -EINVAL;
		}

		if (count != FELICA_OUT_SIZE) {
			return -EINVAL;
		}

		ret_cpy = copy_from_user(local_buff, buff, FELICA_OUT_SIZE);

		if (ret_cpy != 0) {
			return -EFAULT;
		}

		if (FELICA_OUT_L == local_buff[0]) {
			ret_sub = FeliCaCtrl_priv_ctrl_offline();
		}
		else if (FELICA_OUT_H == local_buff[0]) {
			ret_sub = FeliCaCtrl_priv_ctrl_online(&(pPonCtrl->PON_Timer));
		}
		else {
			ret_sub = -EINVAL;
		}

		if (ret_sub == 0) {
			ret = FELICA_OUT_SIZE;
		}
		else{
			ret = ret_sub;
		}
		
		break;

	case FELICA_CTRL_MINOR_NO_CEN:
		pCenCtrl = (struct FELICA_CTRLDEV_CEN_INFO*)gCenCtrl.Info;
		if (NULL == pCenCtrl) {
			return -EINVAL;
		}

		if (count != FELICA_OUT_SIZE) {
			return -EINVAL;
		}

		ret_cpy = copy_from_user(local_buff, buff, FELICA_OUT_SIZE);
		if (ret_cpy != 0) {
			return -EFAULT;
		}

		if (FELICA_OUT_L == local_buff[0]) {
			ret_sub = FeliCaCtrl_priv_ctrl_unenable(
						  &pCenCtrl->PON_Timer
						, &pCenCtrl->CEN_Timer);
		}
		else if (FELICA_OUT_H == local_buff[0]) {
			ret_sub = FeliCaCtrl_priv_ctrl_enable(
						  &pCenCtrl->CEN_Timer);
		}
		else {
			ret_sub = -EINVAL;
		}

		if (ret_sub == 0) {
			ret = FELICA_OUT_SIZE;
		}
		else {
			ret = ret_sub;
		}
		break;

	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}


static ssize_t FeliCaCtrl_read(
							struct file  *pfile,
							char         *buff,
							size_t        count,
							loff_t       *poff)
{
	ssize_t 		ret = 0;
	unsigned char	local_buff[max(FELICA_OUT_SIZE, FELICA_EDGE_OUT_SIZE)];
	unsigned char	minor = 0;
	int 			ret_sub = 0;
	unsigned char	w_rdata;
	unsigned long	ret_cpy = 0;
	struct FELICA_CTRLDEV_ITR_INFO *pItr;


	if ((pfile == NULL) || (buff == NULL)) {
		return -EINVAL;
	}

	minor = MINOR(pfile->f_dentry->d_inode->i_rdev);

	switch (minor) {
	case FELICA_CTRL_MINOR_NO_CEN:
		if (count != FELICA_OUT_SIZE) {
			return -EINVAL;
		}

		ret_sub = FeliCaCtrl_priv_terminal_cen_read(&w_rdata);
		if (ret_sub != 0) {
			return ret_sub;
		}

		switch (w_rdata) {
		case DFD_OUT_H:
			local_buff[0] = FELICA_OUT_H;
			break;
		case DFD_OUT_L:
		default:
			local_buff[0] = FELICA_OUT_L;
			break;
		}

		ret_cpy = copy_to_user(buff, local_buff, FELICA_OUT_SIZE);
		if (ret_cpy != 0) {
			return -EFAULT;
		}
		else {
			ret = FELICA_OUT_SIZE;
		}
		break;

	case FELICA_CTRL_MINOR_NO_RFS:
		if (count != FELICA_OUT_SIZE) {
			return -EINVAL;
		}

		ret_sub = FeliCaCtrl_priv_terminal_rfs_read(&w_rdata);
		if (ret_sub != 0) {
			return ret_sub;
		}

		switch (w_rdata) {
		case DFD_OUT_L:
			local_buff[0] = FELICA_OUT_RFS_L;
			break;
		case DFD_OUT_H:
		default:
			local_buff[0] = FELICA_OUT_RFS_H;
			break;
		}

		ret_cpy = copy_to_user(buff, local_buff, FELICA_OUT_SIZE);
		if (ret_cpy != 0) {
			return -EFAULT;
		}
		else {
			ret = FELICA_OUT_SIZE;
		}
		break;

	case FELICA_CTRL_MINOR_NO_ITR:
		pItr = (struct FELICA_CTRLDEV_ITR_INFO*)gItrCtrl.Info;
		if (NULL == pItr) {
			return -EINVAL;
		}

		if (count != FELICA_EDGE_OUT_SIZE) {
			return -EINVAL;
		}

		spin_lock_irqsave(&itr_lock, itr_lock_flag);

		local_buff[0] = pItr->RFS_Flag;
		local_buff[1] = pItr->INT_Flag;
		pItr->RFS_Flag = FELICA_EDGE_NON;
		pItr->INT_Flag = FELICA_EDGE_NON;

		spin_unlock_irqrestore(&itr_lock, itr_lock_flag);


		ret_cpy = copy_to_user(buff, local_buff, FELICA_EDGE_OUT_SIZE);
		if (ret_cpy != 0) {
			return -EFAULT;
		}
		else {
			ret = FELICA_EDGE_OUT_SIZE;
		}
		break;

	default:
		ret = -ENODEV;
		break;
	}
	
	return ret;
}


static irqreturn_t FeliCaCtrl_interrupt_INT(int i_req, void *pv_devid)
{
	struct FELICA_CTRLDEV_ITR_INFO *pItr = (struct FELICA_CTRLDEV_ITR_INFO*)gItrCtrl.Info;
	int ret_sub = 0;
	unsigned char rdata = DFD_OUT_H;

	
	if (NULL == pItr) {
		return IRQ_HANDLED;
	}
	
	ret_sub = FeliCaCtrl_priv_terminal_int_read(&rdata);

	if ((ret_sub == 0) && (rdata == DFD_OUT_L)) {
		pItr->INT_Flag = FELICA_EDGE_L;
		wake_up_interruptible(&(pItr->RcvWaitQ));
	}

	return IRQ_HANDLED;
}


static void FeliCaCtrl_timer_hnd_general(unsigned long p)
{
	struct FELICA_TIMER *pTimer;


	pTimer = (struct FELICA_TIMER *)p;
	if (pTimer != NULL) {
		wake_up_interruptible(&pTimer->wait);
	}

	return;
}


static int FeliCaCtrl_priv_ctrl_enable(struct FELICA_TIMER *pTimer)
{
	int ret = 0;
	int ret_sub = 0;


	if (NULL == pTimer) {
		return -EINVAL;
	}

	ret_sub = timer_pending(&pTimer->Timer);

	if (ret_sub != 0) {
		return -EBUSY;
	}

	ret = FeliCaCtrl_priv_terminal_cen_write(DFD_OUT_H);

	if (ret >= 0) {
		pTimer->Timer.function = FeliCaCtrl_timer_hnd_general;
		pTimer->Timer.data	   = (unsigned long)pTimer;
		mod_timer(&pTimer->Timer, jiffies + FELICA_TIMER_CEN_TERMINAL_WAIT);

		wait_event_interruptible(
			pTimer->wait,
			(timer_pending(&pTimer->Timer) == 0));

		ret_sub = timer_pending(&pTimer->Timer);
		if (ret_sub == 1) {
			del_timer(&pTimer->Timer);
		}
	}

	return ret;
}


static int FeliCaCtrl_priv_ctrl_unenable(
							struct FELICA_TIMER* pPonTimer,
							struct FELICA_TIMER* pCenTimer)
{
	int ret = 0;
	int ret_sub = 0;


	if ((NULL == pPonTimer) || (NULL == pCenTimer)) {
		return -EINVAL;
	}

	(void)FeliCaCtrl_priv_ctrl_offline();

	pPonTimer->Timer.function = FeliCaCtrl_timer_hnd_general;
	pPonTimer->Timer.data	   = (unsigned long)pPonTimer;
	mod_timer(&pPonTimer->Timer, jiffies + FELICA_TIMER_PON_TERMINAL_WAIT);

	wait_event_interruptible(
		pPonTimer->wait,
		(timer_pending(&pPonTimer->Timer) == 0));

	ret_sub = timer_pending(&pPonTimer->Timer);
	if (ret_sub == 1) {
		del_timer(&pPonTimer->Timer);
	}

	ret = FeliCaCtrl_priv_terminal_cen_write(DFD_OUT_L);

	if (ret == 0) {
		pCenTimer->Timer.function = FeliCaCtrl_timer_hnd_general;
		pCenTimer->Timer.data	   = (unsigned long)pCenTimer;
		mod_timer(&pCenTimer->Timer, jiffies + FELICA_TIMER_CEN_TERMINAL_WAIT);

		wait_event_interruptible(
			pCenTimer->wait,
			(timer_pending(&pCenTimer->Timer) == 0));

		ret_sub = timer_pending(&pCenTimer->Timer);
		if (ret_sub == 1) {
			del_timer(&pCenTimer->Timer);
		}
	}

	return ret;
}


#define TTY_FILE_PATH "/dev/ttyHSL2"

static int FeliCaCtrl_priv_ctrl_online(struct FELICA_TIMER *pTimer)
{
	int ret = 0;
	int ret_sub = 0;
	unsigned char rdata;


	if (NULL == pTimer) {
		return -EINVAL;
	}

	ret_sub = FeliCaCtrl_priv_terminal_cen_read(&rdata);
	if ((ret_sub != 0) || (rdata == DFD_OUT_L)) {
		ret = -EIO;
	}

	if (ret == 0) {
		ret_sub = FeliCaCtrl_priv_terminal_pon_write(DFD_OUT_H);
		if (ret_sub != 0) {
			ret = ret_sub;
		}
	
	}

	if (ret == 0) {
		pTimer->Timer.function = FeliCaCtrl_timer_hnd_general;
		pTimer->Timer.data     = (unsigned long)pTimer;
		mod_timer(&pTimer->Timer, jiffies + FELICA_TIMER_PON_TERMINAL_WAIT);

		wait_event_interruptible(
			pTimer->wait,
			(timer_pending(&pTimer->Timer) == 0));

		ret_sub = timer_pending(&pTimer->Timer);
		if (ret_sub == 1) {
			del_timer(&pTimer->Timer);
		}
	}


	return ret;
}


static int FeliCaCtrl_priv_ctrl_offline(void)
{
	int ret = 0;
	int ret_sub = 0;
	unsigned char rdata;


	ret_sub = FeliCaCtrl_priv_terminal_cen_read(&rdata);
	if ((ret_sub != 0) || (rdata == DFD_OUT_L)) {
		ret = -EIO;
	}

	if (ret == 0) {
		ret_sub = FeliCaCtrl_priv_terminal_pon_write(DFD_OUT_L);

		if (ret_sub != 0) {
			ret = ret_sub;
		}
	}

	return ret;
}


static int FeliCaCtrl_priv_terminal_cen_read(unsigned char *cen_data)
{
	int ret = 0;
	int data = 0;

	ret = FeliCaCtrl_priv_pm8xxx_gpio_read(
			PM8921_GPIO_PM_TO_SYS(FELICA_GPIO_PORT_CEN_PM)
			,&data);
	if (ret == 0){
		if (data == 1){
			*cen_data = DFD_OUT_H;
		}
		else {
			*cen_data = DFD_OUT_L;
		}
	}

	return ret;

}


static int FeliCaCtrl_priv_terminal_rfs_read(unsigned char *rfs_data)
{

	int ret = 0;
	int w_rdata;


	if (NULL == rfs_data) {
		return -EINVAL;
	}

	w_rdata = gpio_get_value(FELICA_GPIO_PORT_RFS);

	if (w_rdata == 0) {
		*rfs_data = DFD_OUT_L;
	}
	else {
		*rfs_data = DFD_OUT_H;
	}

	return ret;
}


static int FeliCaCtrl_priv_terminal_int_read(unsigned char *int_data)
{
	int ret = 0;
	int w_rdata;


	if (NULL == int_data) {
		return -EINVAL;
	}

	w_rdata = gpio_get_value(FELICA_GPIO_PORT_INT);

	if (w_rdata == 0) {
		*int_data = DFD_OUT_L;
	}
	else {
		*int_data = DFD_OUT_H;
	}

	return ret;
}


static int FeliCaCtrl_priv_terminal_cen_write(unsigned char cen_data)
{
	int ret = 0;
	int ret_sub = 0;
	int w_wdata = 0;

	switch (cen_data) {
	case DFD_OUT_H:
		w_wdata = 1;
		break;
	case DFD_OUT_L:
		w_wdata = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret == 0) {
		//clk down
		ret = FeliCaCtrl_priv_pm8xxx_gpio_write(
						PM8921_GPIO_PM_TO_SYS(FELICA_GPIO_PORT_CLK_PD), 0);
		if (ret == 0) {
			//set value
			ret_sub = FeliCaCtrl_priv_pm8xxx_gpio_write(
						PM8921_GPIO_PM_TO_SYS(FELICA_GPIO_PORT_CEN_PM), w_wdata);
			if (ret_sub == 0) {
				
			}
			else {
				ret = ret_sub;
			}
			//clk up
			(void)FeliCaCtrl_priv_pm8xxx_gpio_write(
						PM8921_GPIO_PM_TO_SYS(FELICA_GPIO_PORT_CLK_PD), 1);
		}
	}

	return ret;
}


static int FeliCaCtrl_priv_terminal_pon_write(unsigned char pon_data)
{
	int ret = 0;
	int ret_sub = 0;
	u8 w_wdata = 0;
	static int status = 0;
	int rc;
	
	switch (pon_data) {
	case DFD_OUT_L:
		w_wdata = 0;
		break;
	case DFD_OUT_H:
		w_wdata = 1;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret == 0) {
		if ( (w_wdata == 1) && (status == 0) ){
			felica_set_lvs1();
			felica_set_l16();
			rc = blocking_notifier_call_chain(&felica_notifier_list, FELICA_UART_ON, NULL);
			status = 1;
		}
			ret_sub = gpio_direction_output(FELICA_GPIO_PORT_PON, w_wdata);
			if (ret_sub != 0) {
				ret = -EIO;
			}
		if ( (w_wdata == 0) && (status == 1) ){
			felica_reset_lvs1();
			felica_reset_l16();
			rc = blocking_notifier_call_chain(&felica_notifier_list, FELICA_UART_OFF, NULL); 
			status = 0;
		}
	}


	return ret;
}


static int FeliCaCtrl_priv_interrupt_ctrl_init(unsigned char itr_type)
{
	int ret = 0;
	unsigned gpio_type = 0;
	const char *gpio_label = NULL;


	switch (itr_type) {
	case DFD_ITR_TYPE_RFS:
		gpio_type = FELICA_GPIO_PORT_RFS;
		gpio_label = FELICA_GPIO_STR_RFS;
		break;
	case DFD_ITR_TYPE_INT:
		gpio_type = FELICA_GPIO_PORT_INT;
		gpio_label = FELICA_GPIO_STR_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret == 0) {
	gFeliCaItrInfo[itr_type].irq = MSM_GPIO_TO_INT(gpio_type);

	}

	return ret;
}


static void FeliCaCtrl_priv_interrupt_ctrl_exit(unsigned char itr_type)
{
	int ret = 0;
	unsigned gpio_type = 0;


	switch (itr_type) {
	case DFD_ITR_TYPE_RFS:
		gpio_type = FELICA_GPIO_PORT_RFS;
		break;
	case DFD_ITR_TYPE_INT:
		gpio_type = FELICA_GPIO_PORT_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret == 0) {
	}

	return;
}


static int FeliCaCtrl_priv_interrupt_ctrl_regist(unsigned char itr_type)
{
	int ret = 0;
	int ret_sub = 0;


	switch (itr_type) {
	case DFD_ITR_TYPE_RFS:
	case DFD_ITR_TYPE_INT:
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret == 0) {
		ret_sub = request_irq(
						gFeliCaItrInfo[itr_type].irq,
						gFeliCaItrInfo[itr_type].handler,
						gFeliCaItrInfo[itr_type].flags,
						gFeliCaItrInfo[itr_type].name,
						gFeliCaItrInfo[itr_type].dev);
		if (ret_sub != 0)
		{
			FeliCaCtrl_priv_alarm(DFD_ALARM_INTERRUPT_REQUEST);
			ret = -EIO;
		}
	}

	return ret;
}


static void FeliCaCtrl_priv_interrupt_ctrl_unregist(unsigned char itr_type)
{
	int ret = 0;


	switch (itr_type) {
	case DFD_ITR_TYPE_RFS:
	case DFD_ITR_TYPE_INT:
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret == 0) {
		free_irq(
				gFeliCaItrInfo[itr_type].irq,
				gFeliCaItrInfo[itr_type].dev);
	}

	return;
}


static void FeliCaCtrl_priv_drv_timer_data_init(struct FELICA_TIMER *pTimer)
{

	if (pTimer != NULL) {
		init_timer(&pTimer->Timer);
		init_waitqueue_head(&pTimer->wait);
	}

	return;
}

static void FeliCaCtrl_priv_drv_init_cleanup(int cleanup)
{

	switch (cleanup) {
	case DFD_CLUP_ALL:
	case DFD_CLUP_DEV_DESTORY_CFG:
		device_destroy(
			felica_class,
			MKDEV(FELICA_CFG_MAJOR_NO, FELICA_CFG_MINOR_NO_CFG));
	case DFD_CLUP_DEV_DESTORY_RWS:
		device_destroy(
			felica_class,
			MKDEV(FELICA_RWS_MAJOR_NO, FELICA_RWS_MINOR_NO_RWS));
	case DFD_CLUP_DEV_DESTORY_COMM:
		device_destroy(
			felica_class,
			MKDEV(FELICA_COMM_MAJOR_NO, FELICA_COMM_MINOR_NO_COMM));
	case DFD_CLUP_DEV_DESTORY_ITR:
		device_destroy(
			felica_class,
			MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_ITR));
	case DFD_CLUP_DEV_DESTORY_RFS:
		device_destroy(
			felica_class,
			MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_RFS));
	case DFD_CLUP_DEV_DESTORY_CEN:
		device_destroy(
			felica_class,
			MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_CEN));
	case DFD_CLUP_DEV_DESTORY_PON:
		device_destroy(
			felica_class,
			MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_PON));
	case DFD_CLUP_CLASS_DESTORY:
		class_destroy(felica_class);
	case DFD_CLUP_CDEV_DEL_ITR:
		cdev_del(&g_cdev_fitr);
	case DFD_CLUP_UNREG_CDEV_ITR:
		unregister_chrdev_region(
			MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_ITR),
			FELICA_DEV_NUM);
	case DFD_CLUP_CDEV_DEL_RFS:
		cdev_del(&g_cdev_frfs);
	case DFD_CLUP_UNREG_CDEV_RFS:
		unregister_chrdev_region(
			MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_RFS),
			FELICA_DEV_NUM);
	case DFD_CLUP_CDEV_DEL_CEN:
		cdev_del(&g_cdev_fcen);
	case DFD_CLUP_UNREG_CDEV_CEN:
		unregister_chrdev_region(
			MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_CEN),
			FELICA_DEV_NUM);
	case DFD_CLUP_CDEV_DEL_PON:
		cdev_del(&g_cdev_fpon);
	case DFD_CLUP_UNREG_CDEV_PON:
		unregister_chrdev_region(
			MKDEV(FELICA_CTRL_MAJOR_NO, FELICA_CTRL_MINOR_NO_PON),
			FELICA_DEV_NUM);
	case DFD_CLUP_NONE:
	default:
		break;
	}

	return;
}


static int FeliCaCtrl_priv_dev_pon_open(struct inode *pinode,
														struct file *pfile)
{
	int ret = 0;
	struct FELICA_CTRLDEV_PON_INFO *pCtrl = NULL;

	if ((NULL == pfile) || (NULL == pinode)) {
		return -EINVAL;
	}

	if ((pfile->f_flags & O_ACCMODE) == O_RDONLY) {
		return -EACCES;
	}
	else if ((pfile->f_flags & O_ACCMODE) != O_WRONLY) {
		return -EINVAL;
	}
	else{}

	if ((gPonCtrl.w_cnt == 0) && (gPonCtrl.Info != NULL)) {
		kfree(gPonCtrl.Info);
		gPonCtrl.Info = NULL;
	}

	pCtrl = gPonCtrl.Info;

	if (NULL == pCtrl) {
		pCtrl = kmalloc(sizeof(*pCtrl), GFP_KERNEL);
		if (NULL == pCtrl) {
			return -ENOMEM;
		}
		gPonCtrl.Info = pCtrl;
		FeliCaCtrl_priv_drv_timer_data_init(&pCtrl->PON_Timer);

		gPonCtrl.w_cnt = 0;
	}

	gPonCtrl.w_cnt++;

	return ret;
}


static int FeliCaCtrl_priv_dev_cen_open(struct inode *pinode,
														struct file *pfile)
{
	int ret = 0;
	struct FELICA_CTRLDEV_CEN_INFO *pCtrl = NULL;


	if ((NULL == pfile) || (NULL == pinode)) {
		return -EINVAL;
	}

	if (((pfile->f_flags & O_ACCMODE) != O_RDONLY)
		&& ((pfile->f_flags & O_ACCMODE) != O_WRONLY)) {
		return -EINVAL;
    }

	if ((gCenCtrl.w_cnt == 0)
				&& (gCenCtrl.r_cnt == 0)
				&& (NULL != gCenCtrl.Info)) {
		kfree(gCenCtrl.Info);
		gCenCtrl.Info = NULL;
	}

	pCtrl = gCenCtrl.Info;

	if (NULL == pCtrl) {
		pCtrl = kmalloc(sizeof(*pCtrl), GFP_KERNEL);
		if (NULL == pCtrl) {
			return -ENOMEM;
		}
		gCenCtrl.Info = pCtrl;
		FeliCaCtrl_priv_drv_timer_data_init(&pCtrl->PON_Timer);
		FeliCaCtrl_priv_drv_timer_data_init(&pCtrl->CEN_Timer);

		gCenCtrl.r_cnt = 0;
		gCenCtrl.w_cnt = 0;
	}

	if ((pfile->f_flags & O_ACCMODE) == O_RDONLY) {
		gCenCtrl.r_cnt++;
	}
	else {
		gCenCtrl.w_cnt++;
	}

	return ret;
}


static int FeliCaCtrl_priv_dev_rfs_open(struct inode *pinode,
														struct file *pfile)
{
	int ret = 0;


	if ((NULL == pfile) || (NULL == pinode)) {
		return -EINVAL;
	}

	if ((pfile->f_flags & O_ACCMODE) == O_WRONLY) {
		return -EACCES;
	}
	else if ((pfile->f_flags & O_ACCMODE) != O_RDONLY) {
		return -EINVAL;
	}
	else{}

	gRfsCtrl.r_cnt++;

	return ret;
}


static int FeliCaCtrl_priv_dev_itr_open(struct inode *pinode,
														struct file *pfile)
{
	int ret = 0;
	struct FELICA_CTRLDEV_ITR_INFO *pCtrl = NULL;


	if ((NULL == pfile) || (NULL == pinode)) {
		return -EINVAL;
	}

	if ((pfile->f_flags & O_ACCMODE) == O_WRONLY) {
		return -EACCES;
	}
	else if ((pfile->f_flags & O_ACCMODE) != O_RDONLY) {
		return -EINVAL;
	}
	else if (gItrCtrl.r_cnt >= FELICA_DEV_ITR_MAX_OPEN_NUM) {
		return -EINVAL;
	}
	else{}

	if ((gItrCtrl.r_cnt == 0) && (gItrCtrl.Info != NULL)) {
		kfree(gItrCtrl.Info);
		gItrCtrl.Info = NULL;
	}

	pCtrl = gItrCtrl.Info;

	if (NULL == pCtrl) {
		pCtrl = kmalloc(sizeof(*pCtrl), GFP_KERNEL);
		if (NULL == pCtrl) {
			return -ENOMEM;
		}
		gItrCtrl.Info = pCtrl;
		init_waitqueue_head(&pCtrl->RcvWaitQ);
		spin_lock_init(&itr_lock);
		gItrCtrl.r_cnt = 0;
		gItrCtrl.w_cnt = 0;
		pCtrl->INT_Flag = FELICA_EDGE_NON;
		pCtrl->RFS_Flag = FELICA_EDGE_NON;
		gFeliCaItrInfo[DFD_ITR_TYPE_INT].dev = pCtrl;
	}

	if (gItrCtrl.r_cnt == 0) {
		do {
			ret = FeliCaCtrl_priv_interrupt_ctrl_regist(DFD_ITR_TYPE_INT);
			if (ret != 0) {
				break;
			}
		} while (0);
	}

	if (ret == 0) {
		gItrCtrl.r_cnt++;
	}
	else if (gItrCtrl.r_cnt == 0) {
		kfree(gItrCtrl.Info);
		gItrCtrl.Info = NULL;
	}
	else{}

	return ret;
}


static int FeliCaCtrl_priv_dev_pon_close(struct inode *pinode, struct file *pfile)
{
	int    ret = 0;
	struct FELICA_CTRLDEV_PON_INFO *pCtrl =
				 (struct FELICA_CTRLDEV_PON_INFO*)gPonCtrl.Info;


	if ((NULL == pfile) || (NULL == pinode)) {
		return -EINVAL;
	}

	if (NULL == pCtrl) {
		return -EINVAL;
	}

	if (((pfile->f_flags & O_ACCMODE) == O_WRONLY)
		 					&& (gPonCtrl.w_cnt > 0)) {
		gPonCtrl.w_cnt--;
	}
	else {
		return -EINVAL;
	}

	if (gPonCtrl.w_cnt == 0) {
		if (timer_pending(&pCtrl->PON_Timer.Timer) == 1) {
			del_timer(&pCtrl->PON_Timer.Timer);
			wake_up_interruptible(&pCtrl->PON_Timer.wait);
		}

		(void)FeliCaCtrl_priv_terminal_pon_write(DFD_OUT_L);

		kfree(gPonCtrl.Info);
		gPonCtrl.Info = NULL;
	}

	return ret;
}


static int FeliCaCtrl_priv_dev_cen_close(struct inode *pinode, struct file *pfile)
{
	int    ret = 0;
	struct FELICA_CTRLDEV_CEN_INFO *pCtrl =
				(struct FELICA_CTRLDEV_CEN_INFO*)gCenCtrl.Info;


	if ((NULL == pfile) || (NULL == pinode)) {
		return -EINVAL;
	}

	if (NULL == pCtrl) {
		return -EINVAL;
	}

	if (((pfile->f_flags & O_ACCMODE) == O_RDONLY)
							&& (gCenCtrl.r_cnt > 0)) {
		gCenCtrl.r_cnt--;
	}
	else if (((pfile->f_flags & O_ACCMODE) == O_WRONLY)
								&& (gCenCtrl.w_cnt > 0)) {
		gCenCtrl.w_cnt--;
	}
	else
	{
		return -EINVAL;
	}

	if ((gCenCtrl.r_cnt == 0) && (gCenCtrl.w_cnt == 0)) {
		if (timer_pending(&pCtrl->PON_Timer.Timer) == 1) {
			del_timer(&pCtrl->PON_Timer.Timer);
			wake_up_interruptible(&pCtrl->PON_Timer.wait);
		}

		if (timer_pending(&pCtrl->CEN_Timer.Timer) == 1) {
			del_timer(&pCtrl->CEN_Timer.Timer);
			wake_up_interruptible(&pCtrl->CEN_Timer.wait);
		}

		kfree(gCenCtrl.Info);
		gCenCtrl.Info = NULL;
	}

	return ret;
}


static int FeliCaCtrl_priv_dev_rfs_close(
								struct inode *pinode,
								struct file *pfile)
{
	int ret = 0;


	if ((NULL == pfile) || (NULL == pinode)) {
		return -EINVAL;
	}

	if (((pfile->f_flags & O_ACCMODE) == O_RDONLY)
							&& (gRfsCtrl.r_cnt > 0)) {
		gRfsCtrl.r_cnt--;
	}
	else {
		return -EINVAL;
	}

	return ret;
}


static int FeliCaCtrl_priv_dev_itr_close(
									struct inode *pinode,
									struct file *pfile)
{
	int ret = 0;
	struct FELICA_CTRLDEV_ITR_INFO *pCtrl =
				(struct FELICA_CTRLDEV_ITR_INFO*)gItrCtrl.Info;


	if ((NULL == pfile) || (NULL == pinode)) {
		return -EINVAL;
	}

	if (NULL == pCtrl) {
		return -EINVAL;
	}

	if (((pfile->f_flags & O_ACCMODE) == O_RDONLY)
								&& (gItrCtrl.r_cnt > 0)
								&& (gItrCtrl.r_cnt <= FELICA_DEV_ITR_MAX_OPEN_NUM)) {
		gItrCtrl.r_cnt--;
	}
	else {
		return -EINVAL;
	}

	wake_up_interruptible(&pCtrl->RcvWaitQ);

	if (gItrCtrl.r_cnt == 0) {

		FeliCaCtrl_priv_interrupt_ctrl_unregist(DFD_ITR_TYPE_INT);

		kfree(gItrCtrl.Info);
		gItrCtrl.Info = NULL;
	}

	return ret;
}


static void FeliCaCtrl_priv_alarm(unsigned long err_id)
{
	return;
}

MODULE_AUTHOR("Panasonic Mobile Communications Co., Ltd.");
MODULE_DESCRIPTION("FeliCa Control Driver");
MODULE_LICENSE("GPL");
/******************************************************************************/
/* Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved             */
/******************************************************************************/
