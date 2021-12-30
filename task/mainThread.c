/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/********************************************************************************
 *  INCLUDES
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* POSIX Header files */
#include <pthread.h>

/* Driver Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/pin/PINCC26XX.h>

#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>

/* Example/Board Header files */
#include "Board.h"

/* POSIX Header files */
#include <semaphore.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Application Header files */
#include <rf/RFQueue.h>
#include "smartrf_settings/smartrf_settings.h"
#include <task/DataType.h>

/********************************************************************************
 *  Macros
 */

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             30 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */



/********************************************************************************
 *  GLOBAL VARIABLES
 */
pthread_t   eventthread;
Display_Handle display = NULL;
sem_t EvtDataRecv;

/* 运行时配置 */
PIN_Config pinTable[] =
{
    CC1310_LAUNCHXL_SYNC_PWM | PIN_INPUT_EN, /* cc3235s 1s sync input */
    PIN_TERMINATE
};

/********************************************************************************
 *  LOCAL VARIABLES
 */
static RF_Object rfObject;
static RF_Handle rfHandle;

static RF_RatHandle ratHandle;
static PIN_Handle RATPinHandle;
static PIN_State RATPinState;

#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;

static uint8_t packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1]; /* The length byte is stored in a separate variable */

/********************************************************************************
 *  EXTERNAL VARIABLES
 */
extern I2CBuff_t I2C_BUFF;

/********************************************************************************
 *  EXTERNAL FUNCTIONS
 */
extern void *eventThread(void *arg0);

/********************************************************************************
 *  Callback
 */
void onSignalTriggered(RF_Handle h, RF_RatHandle rh, RF_EventMask e, uint32_t compareCaptureTime)
{
    if (e & RF_EventError)
    {
        // An internal error has occurred
    }
    uint32_t lastCaptureTime = I2C_BUFF.Tsor;
    I2C_BUFF.Tsor = compareCaptureTime;
    uint32_t delay = compareCaptureTime - lastCaptureTime;

    Display_printf(display, 0, 0,"\r\n SyncTime %u. LastTime %u. delay %u.\r\n",
                   compareCaptureTime,lastCaptureTime,delay);
}

void RxRecvcallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {
        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(packet, packetDataPointer, (packetLength + 1));

        RFQueue_nextEntry();
    }
}

// 测试版本 信号源输入模拟接收
static int idx = 0;
void testcb(uint_least8_t index){

    I2C_BUFF.Tror = RF_getCurrentTime();
    I2C_BUFF.Index = idx++;
    I2C_BUFF.Type = 0xaa;

    // 翻转IO 通知cc3235s发起I2C传输
    GPIO_toggle(Board_GPIO_WAKEUP);

    GPIO_toggle(Board_GPIO_LED_BLUE);

    sem_post(&EvtDataRecv);
}

/********************************************************************************
 *  LOCAL FUNCTIONS
 */
static void RF_Config(){

    RF_Params rfParams;
    RF_Params_init(&rfParams);

    if( RFQueue_defineQueue(&dataQueue,
                               rxDataEntryBuffer,
                               sizeof(rxDataEntryBuffer),
                               NUM_DATA_ENTRIES,
                               MAX_LENGTH + NUM_APPENDED_BYTES))
       {
           /* Failed to allocate space for all data entries */
           while(1);
       }

    /* Modify CMD_PROP_RX command for application needs */
    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

}

static void RFRAT_Config(){

    /* Initialize RF RAT */
    // Map cc33235s Sync Input to RFC_GPI0

    RATPinHandle = PIN_open(&RATPinState, pinTable);
    if (RATPinHandle == NULL)
        while(1);

    PINCC26XX_setMux(RATPinHandle, CC1310_LAUNCHXL_SYNC_PWM, PINCC26XX_MUX_RFC_GPI0);


    RF_RatConfigCapture config;
    RF_RatConfigCapture_init(&config);
    config.callback = &onSignalTriggered;
    config.channel = RF_RatChannelAny;
    config.source = RF_RatCaptureSourceRfcGpi0;
    config.captureMode = RF_RatCaptureModeBoth; // 上下边沿触发
    config.repeat = RF_RatCaptureRepeat;

    ratHandle = RF_ratCapture(rfHandle, &config, 0);
}

static void RF_rxRUN(){

    /* Enter RX mode and stay forever in RX */
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                               RF_PriorityNormal, &RxRecvcallback,
                                               RF_EventRxEntryDone);

    switch(terminationReason)
    {
        case RF_EventLastCmdDone:
            // A stand-alone radio operation command or the last radio
            // operation command in a chain finished.
            break;
        case RF_EventCmdCancelled:
            // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
            break;
        case RF_EventCmdAborted:
            // Abrupt command termination caused by RF_cancelCmd() or
            // RF_flushCmd().
            break;
        case RF_EventCmdStopped:
            // Graceful command termination caused by RF_cancelCmd() or
            // RF_flushCmd().
            break;
        default:
            // Uncaught error event
            while(1);
    }

    uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropRx)->status;
    switch(cmdStatus)
    {
        case PROP_DONE_OK:
            // Packet received with CRC OK
            break;
        case PROP_DONE_RXERR:
            // Packet received with CRC error
            break;
        case PROP_DONE_RXTIMEOUT:
            // Observed end trigger while in sync search
            break;
        case PROP_DONE_BREAK:
            // Observed end trigger while receiving packet when the command is
            // configured with endType set to 1
            break;
        case PROP_DONE_ENDED:
            // Received packet after having observed the end trigger; if the
            // command is configured with endType set to 0, the end trigger
            // will not terminate an ongoing reception
            break;
        case PROP_DONE_STOPPED:
            // received CMD_STOP after command started and, if sync found,
            // packet is received
            break;
        case PROP_DONE_ABORT:
            // Received CMD_ABORT after command started
            break;
        case PROP_ERROR_RXBUF:
            // No RX buffer large enough for the received data available at
            // the start of a packet
            break;
        case PROP_ERROR_RXFULL:
            // Out of RX buffer space during reception in a partial read
            break;
        case PROP_ERROR_PAR:
            // Observed illegal parameter
            break;
        case PROP_ERROR_NO_SETUP:
            // Command sent without setting up the radio in a supported
            // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
            break;
        case PROP_ERROR_NO_FS:
            // Command sent without the synthesizer being programmed
            break;
        case PROP_ERROR_RXOVF:
            // RX overflow observed during operation
            break;
        default:
            // Uncaught error event - these could come from the
            // pool of states defined in rf_mailbox.h
            while(1);
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;

    /* Call driver init functions */
    GPIO_init();
    Display_init();

    /* Initialize display */
    display = Display_open(Display_Type_UART,NULL); //TODO display输出有bug
    if (display == NULL) {
        /* UART_open() failed */
        while (1);
    }

    /* Initialize semaphore */
    sem_init(&EvtDataRecv, 0, 0);

    /* Initialize RF Core */
    RF_Config();
    RFRAT_Config();

    Display_printf(display, 0, 0, "\r\nNanoEEG cc1310 ready!\r\n");

    /* led on to indicate the system is ready! */
    GPIO_write(Board_GPIO_LED_BLUE,CC1310_LAUNCHXL_PIN_LED_ON);

    //测试版本
    GPIO_setConfig(Board_GPIO_TEST_IN, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
    GPIO_setCallback(Board_GPIO_TEST_IN,testcb);
    GPIO_enableInt(Board_GPIO_TEST_IN);

    /* cc3235事件标签处理线程*/
    /* Initialize the attributes structure with default values */
    pthread_attr_init(&attrs);

    /* Set priority, detach state, and stack size attributes */
    priParam.sched_priority = 3;
    retc = pthread_attr_setschedparam(&attrs, &priParam);
    retc |= pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    retc |= pthread_attr_setstacksize(&attrs, 1024);
    if (retc != 0) {
        /* failed to set attributes */
        while (1) {}
    }

    retc = pthread_create(&eventthread, &attrs, eventThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1) {}
    }

    /* Enter RX mode and stay forever in RX */
    RF_rxRUN();

    while (1) {

    }
}
