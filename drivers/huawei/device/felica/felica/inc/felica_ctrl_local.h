/*
 * @file felica_ctrl_local.h
 * @brief Local header file of FeliCa control driver
 *
 * @date 2012/01/29
 */

#ifndef __DV_FELICA_CTRL_LOCAL_H__
#define __DV_FELICA_CTRL_LOCAL_H__
/*
 * include extra header
 */
#include <linux/sched.h>


/*
 * defined
 */
#define FELICA_POS_BIT_0          (0x00000001U)
#define FELICA_POS_BIT_1          (0x00000002U)
#define FELICA_POS_BIT_2          (0x00000004U)
#define FELICA_POS_BIT_3          (0x00000008U)
#define FELICA_POS_BIT_4          (0x00000010U)
#define FELICA_POS_BIT_5          (0x00000020U)
#define FELICA_POS_BIT_6          (0x00000040U)

#define FELICA_DEV_NAME             "felica"
#define FELICA_DEV_NAME_CTRL_PON    "felica_pon"
#define FELICA_DEV_NAME_CTRL_CEN    "felica_cen"
#define FELICA_DEV_NAME_CTRL_RFS    "felica_rfs"
#define FELICA_DEV_NAME_CTRL_ITR    "felica_interrupt"
#define FELICA_DEV_NAME_COMM        "felica"
#define FELICA_DEV_NAME_RWS         "felica_rws"
#define FELICA_DEV_NAME_CFG         "felica_cfg"

#define FELICA_DEV_NUM (1)
#define FELICA_DEV_ITR_MAX_OPEN_NUM (1)

#define FELICA_CTRL_MAJOR_NO  (123)
#define FELICA_COMM_MAJOR_NO  (124)
#define FELICA_RWS_MAJOR_NO   (125)
#define FELICA_CFG_MAJOR_NO   (126)

#define FELICA_CTRL_MINOR_NO_PON   (0)
#define FELICA_CTRL_MINOR_NO_CEN   (1)
#define FELICA_CTRL_MINOR_NO_RFS   (2)
#define FELICA_CTRL_MINOR_NO_ITR   (3)

#define FELICA_COMM_MINOR_NO_COMM  (0)

#define FELICA_RWS_MINOR_NO_RWS    (0)

#define FELICA_CFG_MINOR_NO_CFG    (0)

#define FELICA_CTRL_ITR_STR_RFS  "felica_ctrl_rfs"
#define FELICA_CTRL_ITR_STR_INT  "felica_ctrl_int"

#define PM8921_GPIO_BASE                 NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)   (pm_gpio - 1 + PM8921_GPIO_BASE)
#define FELICA_GPIO_PORT_CEN_PM          (7)
#define FELICA_GPIO_PORT_CLK_PD          (10)
#define FELICA_GPIO_PORT_PON             (37)
#define FELICA_GPIO_PORT_RFS             (35)
#define FELICA_GPIO_PORT_INT             (106)

#define FELICA_GPIO_STR_RFS    "felica_rfs"
#define FELICA_GPIO_STR_INT    "felica_int"
#define FELICA_GPIO_STR_PON    "felica_pon"
#define FELICA_GPIO_STR_CEN       "felica_cen"
#define FELICA_GPIO_STR_CEN_CLK   "felica_cen_clk"

#define FELICA_TIMER_CHATA             (1)
#define FELICA_TIMER_CHATA_DELAY       ((50 * HZ) / 1000 + 1)
#define FELICA_TIMER_CEN_TERMINAL_WAIT ((20 * HZ) / 1000 + 1)
#define FELICA_TIMER_PON_TERMINAL_WAIT ((30 * HZ) / 1000 + 1)

#define DFD_OUT_L    (0x00)
#define DFD_OUT_H    (0x01)

enum{
	DFD_ITR_TYPE_RFS = 0,
	DFD_ITR_TYPE_INT,
	DFD_ITR_TYPE_NUM
};

enum{
	DFD_CLUP_NONE = 0,
	DFD_CLUP_UNREG_CDEV_PON,
	DFD_CLUP_CDEV_DEL_PON,
	DFD_CLUP_UNREG_CDEV_CEN,
	DFD_CLUP_CDEV_DEL_CEN,
	DFD_CLUP_UNREG_CDEV_RFS,
	DFD_CLUP_CDEV_DEL_RFS,
	DFD_CLUP_UNREG_CDEV_ITR,
	DFD_CLUP_CDEV_DEL_ITR,
	DFD_CLUP_CLASS_DESTORY,
	DFD_CLUP_DEV_DESTORY_PON,
	DFD_CLUP_DEV_DESTORY_CEN,
	DFD_CLUP_DEV_DESTORY_RFS,
	DFD_CLUP_DEV_DESTORY_ITR,
	DFD_CLUP_DEV_DESTORY_COMM,
	DFD_CLUP_DEV_DESTORY_RWS,
	DFD_CLUP_DEV_DESTORY_CFG,
	DFD_CLUP_ALL
};

#define DFD_LOW_RFS_CHATA_CNT_MAX  (3)
#define DFD_LOW_RFS_CHATA_CNT_WIN  (2)
#define DFD_HIGH_RFS_CHATA_CNT_MAX (5)
#define DFD_HIGH_RFS_CHATA_CNT_WIN (3)

#define DFD_ALARM_BASE              ( 0x00000000 )
#define DFD_ALARM_SUBPMIC_READ      ( DFD_ALARM_BASE | 0x0200 )
#define DFD_ALARM_SUBPMIC_WRITE     ( DFD_ALARM_BASE | 0x0201 )
#define DFD_ALARM_INTERRUPT_REQUEST ( DFD_ALARM_BASE | 0x0300 )

struct FELICA_TIMER {
	struct timer_list Timer;
	wait_queue_head_t wait;
};

struct FELICA_CTRLDEV_PON_INFO {
	struct FELICA_TIMER PON_Timer;
};

struct FELICA_CTRLDEV_CEN_INFO {
	struct FELICA_TIMER PON_Timer;
	struct FELICA_TIMER CEN_Timer;
};


struct FELICA_CTRLDEV_ITR_INFO {
	unsigned char     INT_Flag;
	unsigned char     RFS_Flag;
	unsigned char     reserve1[2];

	wait_queue_head_t RcvWaitQ;
};


struct FELICA_DEV_INFO {
	unsigned int    w_cnt;
	unsigned int    r_cnt;
	void * Info;
};

struct FELICA_ITR_INFO {
	unsigned int irq;
	irq_handler_t handler;
	unsigned long flags;
	const char *name;
	void *dev;
};
#endif /* __DV_FELICA_CTRL_LOCAL_H__ */
/******************************************************************************/
/* Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved             */
/******************************************************************************/
