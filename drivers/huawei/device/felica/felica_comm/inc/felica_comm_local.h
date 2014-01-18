/**
 * @file felica_comm_local.h
 * @brief FeliCa driver header file
 *
 * @date 2012/01/29
 *
 */
#ifndef __DV_FELICA_COMM_LOCAL_H__
#define __DV_FELICA_COMM_LOCAL_H__
/* ========================================================================== */
/* define                                                                     */
/* ========================================================================== */
#define FELICA_POS_BIT_0          (0x00000001U)
#define FELICA_POS_BIT_1          (0x00000002U)
#define FELICA_POS_BIT_2          (0x00000004U)
#define FELICA_POS_BIT_3          (0x00000008U)
#define FELICA_POS_BIT_4          (0x00000010U)
#define FELICA_POS_BIT_5          (0x00000020U)
#define FELICA_POS_BIT_6          (0x00000040U)
#define FELICA_POS_BIT_7          (0x00000080U)
#define FELICA_POS_BIT_8          (0x00000100U)
#define FELICA_POS_BIT_9          (0x00000200U)
#define FELICA_POS_BIT_10         (0x00000400U)
#define FELICA_POS_BIT_11         (0x00000800U)
#define FELICA_POS_BIT_12         (0x00001000U)
#define FELICA_POS_BIT_13         (0x00002000U)
#define FELICA_POS_BIT_14         (0x00004000U)
#define FELICA_POS_BIT_15         (0x00008000U)

#define FELICA_DEV_NAME_COMM        "felica"

#define FELICA_COMM_MAJOR_NO  (124)
#define FELICA_COMM_MINOR_NO_COMM  (0)

#define FELICA_DEV_NUM (1)

#define FELICA_UART_IRQ    (104)
#define FELICA_ITR_STR_COMM    "felica_comm"

#define FELICA_CLK_UART_FCK      "uart1_fck"
#define FELICA_CLK_UART_ICK      "uart1_ick"

#define FELICA_COMM_FRAME_NUM       (500)
#define FELICA_COMM_BUFF_SIZE       (30720)
#define FELICA_COMM_WR_BUFF_SIZE    (256)

#define FELICA_TIMER_ACK_SND  ((600 * HZ) / 1000 + 1)
#define FELICA_TIMER_NEXT_SND ((20 * HZ) / 1000 + 1)

#define FELICA_UART_BASE_ADDR     (0x4806A000UL)

#define FELICA_UARTREG_DLL        (FELICA_UART_BASE_ADDR + 0x00000000)
#define FELICA_UARTREG_RHR        (FELICA_UART_BASE_ADDR + 0x00000000)
#define FELICA_UARTREG_THR        (FELICA_UART_BASE_ADDR + 0x00000000)
#define FELICA_UARTREG_DLH        (FELICA_UART_BASE_ADDR + 0x00000004)
#define FELICA_UARTREG_IER        (FELICA_UART_BASE_ADDR + 0x00000004)
#define FELICA_UARTREG_IIR        (FELICA_UART_BASE_ADDR + 0x00000008)
#define FELICA_UARTREG_FCR        (FELICA_UART_BASE_ADDR + 0x00000008)
#define FELICA_UARTREG_EFR        (FELICA_UART_BASE_ADDR + 0x00000008)
#define FELICA_UARTREG_LCR        (FELICA_UART_BASE_ADDR + 0x0000000C)
#define FELICA_UARTREG_MCR        (FELICA_UART_BASE_ADDR + 0x00000010)
#define FELICA_UARTREG_XON1_ADDR1 (FELICA_UART_BASE_ADDR + 0x00000010)
#define FELICA_UARTREG_LSR        (FELICA_UART_BASE_ADDR + 0x00000014)
#define FELICA_UARTREG_XON2_ADDR2 (FELICA_UART_BASE_ADDR + 0x00000014)
#define FELICA_UARTREG_MSR        (FELICA_UART_BASE_ADDR + 0x00000018)
#define FELICA_UARTREG_TCR        (FELICA_UART_BASE_ADDR + 0x00000018)
#define FELICA_UARTREG_XOFF1      (FELICA_UART_BASE_ADDR + 0x00000018)
#define FELICA_UARTREG_SPR        (FELICA_UART_BASE_ADDR + 0x0000001C)
#define FELICA_UARTREG_TLR        (FELICA_UART_BASE_ADDR + 0x0000001C)
#define FELICA_UARTREG_XOFF2      (FELICA_UART_BASE_ADDR + 0x0000001C)
#define FELICA_UARTREG_MDR1       (FELICA_UART_BASE_ADDR + 0x00000020)
#define FELICA_UARTREG_MDR2       (FELICA_UART_BASE_ADDR + 0x00000024)
#define FELICA_UARTREG_SFLSR      (FELICA_UART_BASE_ADDR + 0x00000028)
#define FELICA_UARTREG_TXFLL      (FELICA_UART_BASE_ADDR + 0x00000028)
#define FELICA_UARTREG_RESUME     (FELICA_UART_BASE_ADDR + 0x0000002C)
#define FELICA_UARTREG_TXFLH      (FELICA_UART_BASE_ADDR + 0x0000002C)
#define FELICA_UARTREG_SFREGL     (FELICA_UART_BASE_ADDR + 0x00000030)
#define FELICA_UARTREG_RXFLL      (FELICA_UART_BASE_ADDR + 0x00000030)
#define FELICA_UARTREG_SFREGH     (FELICA_UART_BASE_ADDR + 0x00000034)
#define FELICA_UARTREG_RXFLH      (FELICA_UART_BASE_ADDR + 0x00000034)
#define FELICA_UARTREG_UASR       (FELICA_UART_BASE_ADDR + 0x00000038)
#define FELICA_UARTREG_BLR        (FELICA_UART_BASE_ADDR + 0x00000038)
#define FELICA_UARTREG_ACREG      (FELICA_UART_BASE_ADDR + 0x0000003C)
#define FELICA_UARTREG_SCR        (FELICA_UART_BASE_ADDR + 0x00000040)
#define FELICA_UARTREG_SSR        (FELICA_UART_BASE_ADDR + 0x00000044)
#define FELICA_UARTREG_EBLR       (FELICA_UART_BASE_ADDR + 0x00000048)
#define FELICA_UARTREG_MVR        (FELICA_UART_BASE_ADDR + 0x00000050)
#define FELICA_UARTREG_SYSC       (FELICA_UART_BASE_ADDR + 0x00000054)
#define FELICA_UARTREG_SYSS       (FELICA_UART_BASE_ADDR + 0x00000058)
#define FELICA_UARTREG_WER        (FELICA_UART_BASE_ADDR + 0x0000005C)
#define FELICA_UARTREG_CFPS       (FELICA_UART_BASE_ADDR + 0x00000060)
#define FELICA_UARTREG_RXFIFO_LVL (FELICA_UART_BASE_ADDR + 0x00000064)
#define FELICA_UARTREG_TXFIFO_LVL (FELICA_UART_BASE_ADDR + 0x00000068)
#define FELICA_UARTREG_IER2       (FELICA_UART_BASE_ADDR + 0x0000006C)
#define FELICA_UARTREG_ISR2       (FELICA_UART_BASE_ADDR + 0x00000070)
#define FELICA_UARTREG_MDR3       (FELICA_UART_BASE_ADDR + 0x00000080)

#define FELICA_INTT_TYPE              (0x0000003E)
#define FELICA_INTT_RCV_LINE_ERR      (0x00000006)
#define FELICA_INTT_RCV_TIMEOUT       (0x0000000C)
#define FELICA_INTT_RCV_FIFO_EXIST    (0x00000004)
#define FELICA_INTT_SND_FIFO_EMPTY    (0x00000002)
#define FELICA_INTT_MODEM_STAT        (0x00000000)
#define FELICA_INTT_CHAR              (0x00000010)
#define FELICA_INTT_CTS_RTS           (0x00000020)

#define DFD_SYSC_SOFTRESET     FELICA_POS_BIT_1
#define DFD_SYSC_IDLEMODE      (FELICA_POS_BIT_4 | FELICA_POS_BIT_3)
#define DFD_SYSS_RESETDONE     FELICA_POS_BIT_0
#define DFD_EFR_ENHANCED_EN    FELICA_POS_BIT_4
#define DFD_MCR_TCR_TLR        FELICA_POS_BIT_6
#define DFD_IER_RHR_IT         FELICA_POS_BIT_0
#define DFD_IER_THR_IT         FELICA_POS_BIT_1
#define DFD_IER_LINE_STS_IT    FELICA_POS_BIT_2
#define DFD_FCR_RX_FIFO_CLEAR  FELICA_POS_BIT_1

#define DFD_SYSS_MAX_LOOP            (100)
#define DFD_SYSC_NO_IDLE             (0x08)
#define DFD_LCR_AMODE                (0x0080)
#define DFD_LCR_BMODE                (0x00BF)
#define DFD_LCR_INIT                 (0x0000)
#define DFD_LCR_PROTOCOL_FOMAT       (0x03)
#define DFD_FCR_INIT                 (0x31)
#define DFD_TLR_INIT                 (0x8F)
#define DFD_SCR_INIT                 (0xC0)
#define DFD_MDR1_MODE_SELECT_DIABLE  (0x7)
#define DFD_MDR1_MODE_SELECT_UART13X (0x3)
#define DFD_IER_RESET                (0x0000)
#define DFD_IER_INIT                 (0x0000)
#define DFD_DLL_INIT                 (0x08)
#define DFD_DLH_INIT                 (0x00)

#define DFD_ITR_TYPE_UART    (0x02)

#define DFD_ITR_UART_TYPE_REV (0x00)
#define DFD_ITR_UART_TYPE_SND (0x01)

#define DFD_OK    (0x00)
#define DFD_NG    (0x01)

#define FELICA_COMM_FRAME_OVERFLOW  (-2000)
#define FELICA_COMM_BUFF_OVERFLOW   (-2001)
#define FELICA_COMM_FRAME_NOTHING   (-2002)
#define FELICA_COMM_DATA_NOTHING    (-2003)
#define FELICA_COMM_BAD_FRAME_DATA  (-2004)

#define FELICA_PARAM_NULL           (-3000)

#define FELICA_COMM_PREAMBLE_POS        (0)
#define FELICA_COMM_START_POS           (1)
#define FELICA_COMM_PACKET_SIZE_POS     (2)
#define FELICA_COMM_ACK_SIZE            (4)
#define FELICA_COMM_FRAME_CONST_SIZE    (5)
#define FELICA_COMM_PACKET_MAX          (256)
#define FELICA_COMM_SND_TRIGG_SIZE      (32)
#define FELICA_COMM_SND_MAX_SIZE        (64)

#define FELICA_PREAMBLE     (0x00)
#define FELICA_POSTAMBLE    (0x00)
#define FELICA_COMMSTART    (0xFA)
#define FELICA_STATSTART    (0xF5)
#define FELICA_ACKCODE      (0x05)
#define FELICA_READYCODE    (0x55)
#define FELICA_BUSYCODE     (0x5A)

#define DFD_INT_GUARD    (3)
#define DFD_READ_GUARD   (66)

#define DFD_SUBPMIC_FCCEN    FELICA_POS_BIT_0

#define DFD_ALARM_SIZE     (24)
#define DFD_ALARM_DUMMY    (2)
#define DFD_ALARM_INI      (0x00)

#define DFD_SYS_SIZE       (0)

#define DFD_FUNCID      ( 0 )
#define DFD_ALARM_OE    ( DFD_FUNCID | 0x0100 )
#define DFD_ALARM_INT   ( DFD_FUNCID | 0x0101 )
#define DFD_ALARM_READ  ( DFD_FUNCID | 0x0102 )
#define DFD_ALARM_PORT  ( DFD_FUNCID | 0x0103 )
#define DFD_ALARM_INTERUPT_REQUEST (DFD_FUNCID | 0x0200)

typedef enum {
    FLAG_RESET,
    FLAG_SET,
    FLAG_PROC
} FELICA_FLAG;

typedef enum {
    FRAME_EMPTY,
    FRAME_FULL,
    FRAME_INPUT,
    FRAME_OUTPUT
} FELICA_FRAME_STAT;

typedef enum {
    FRAME_INIT,
    FRAME_OK,
    FRAME_OK_ACK,
    FRAME_TIMEOUT,
    FRAME_COMMERR,
    FRAME_BUFFOVER,
    FRAME_ERROR
} FELICA_FRAME_RESULT;


/* -------------------------------------------------------------------------- */
/* struct                                                                     */
/* -------------------------------------------------------------------------- */
typedef struct {
    unsigned long  pDLL;
    unsigned long  pRDR;
    unsigned long  pTDR;
    unsigned long  pDLH;
    unsigned long  pIER;
    unsigned long  pIIR;
    unsigned long  pFCR;
    unsigned long  pEFR;
    unsigned long  pLCR;
    unsigned long  pMCR;
    unsigned long  pXON1_ADDR1;
    unsigned long  pLSR;
    unsigned long  pXON2_ADDR2;
    unsigned long  pMSR;
    unsigned long  pTCR;
    unsigned long  pXOFF1;
    unsigned long  pSPR;
    unsigned long  pTLR;
    unsigned long  pXOFF2;
    unsigned long  pMDR1;
    unsigned long  pMDR2;
    unsigned long  pSFLSR;
    unsigned long  pTXFLL;
    unsigned long  pRESUME;
    unsigned long  pTXFLH;
    unsigned long  pSFREGL;
    unsigned long  pRXFLL;
    unsigned long  pSFREGH;
    unsigned long  pRXFLH;
    unsigned long  pUASR;
    unsigned long  pBLR;
    unsigned long  pACREG;
    unsigned long  pSCR;
    unsigned long  pSSR;
    unsigned long  pEBLR;
    unsigned long  pMVR;
    unsigned long  pSYSC;
    unsigned long  pSYSS;
    unsigned long  pWER;
    unsigned long  pCFPS;
    unsigned long  pRXFIFO_LVL;
    unsigned long  pTXFIFO_LVL;
    unsigned long  pIER2;
    unsigned long  pISR2;
    unsigned long  pMDR3;
} FELICA_UART_ADDR;

typedef struct {
    FELICA_FLAG       Timeout;
    FELICA_FLAG       Start;
    struct timer_list Timer;
    wait_queue_head_t wait;
} FELICA_TIMER;

typedef struct {
    unsigned char      *FramePtr;
    int                 FrameSize;
    int                 PacketSize;
    FELICA_FLAG         AckCheck;
    FELICA_FRAME_STAT   Status;
    FELICA_FRAME_RESULT Result;
} FELICA_COMM_FRAME;

typedef struct {
    unsigned char  *HeadPtr;
    unsigned char  *TailPtr;
    FELICA_FLAG     PassingFlag;
    unsigned char   RingBuff[ FELICA_COMM_BUFF_SIZE ];
} FELICA_COMM_DATA;

typedef struct {
    unsigned char  *StartPtr;
    int             FrameSize;
    int             FrameCount;
    int             PacketSize;
    int             PacketCount;
} FELICA_COMM_INPUTMNG;

typedef struct {
    unsigned char  *EndPtr;
    int             FrameSize;
    int             FrameCount;
} FELICA_COMM_OUTPUTMNG;

typedef struct {
    struct {
        int                   HeadNo;
        int                   TailNo;
        FELICA_FLAG           PassingFlag;
        FELICA_COMM_FRAME     Frame[ FELICA_COMM_FRAME_NUM ];
    }
    FrameMng;

    struct {
        FELICA_COMM_INPUTMNG  Input;
        FELICA_COMM_OUTPUTMNG Output;
    }
    IOMng;

    FELICA_COMM_DATA          DataBuff;
} FELICA_COMMBUFF;

typedef struct {
    FELICA_TIMER      NextSnd;
    FELICA_TIMER      AckSnd;
    FELICA_FLAG       WriteBlock;
    FELICA_FLAG       NowClose;
    wait_queue_head_t SndWaitQ;
    FELICA_COMMBUFF   Snd;
    FELICA_COMMBUFF   Rcv;
    struct clk        *UartClkIck;
    struct clk        *UartClkFck;

} FELICA_COMMDEV_INFO;

typedef struct {
    int    w_cnt;
    int    r_cnt;
    void * Info;
} FELICA_DEV_INFO;
#endif /* __DV_FELICA_COMM_LOCAL_H__ */
/******************************************************************************/
/* Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved             */
/******************************************************************************/
