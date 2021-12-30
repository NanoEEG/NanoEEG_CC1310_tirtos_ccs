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

#include <task/DataType.h>

/********************************************************************************
 *  GLOBAL VARIABLES
 */
pthread_t   eventthread;
Display_Handle display = NULL;
sem_t EvtDataRecv;

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

    Display_printf(display, 0, 0,"SyncTime %u. LastTime %u. delay %u.",
                   compareCaptureTime,lastCaptureTime,delay);
}


// 测试版本 信号源输入模拟接收
void testcb(uint_least8_t index){


    GPIO_toggle(Board_GPIO_LED_BLUE);
    sem_post(&EvtDataRecv);
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
    Display_init();
    GPIO_init();

    /* Initialize display */
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;
    display = Display_open(Display_Type_UART, &params);

    if (display == NULL) {
        /* UART_open() failed */
        while (1);
    }

    /* Initialize semaphore */
    sem_init(&EvtDataRecv, 0, 0);

    /* Initialize RF RAT */
    // Map cc33235s Sync Input to RFC_GPI0
    static PIN_Handle RATPinHandle;
    static PIN_State RATPinState;
    // Get handle to this collection of pins
    if (!PIN_open(&RATPinState, BoardGpioInitTable)) {
        // Handle allocation error
    }
    PINCC26XX_setMux(RATPinHandle, CC1310_LAUNCHXL_SYNC_PWM, PINCC26XX_MUX_RFC_GPI0);

    /* Initialize RF Core */
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_Handle RFDriver;
    RF_RatConfigCapture config;
    RF_RatConfigCapture_init(&config);
    config.callback = &onSignalTriggered;
    config.channel = RF_RatChannelAny;
    config.source = RF_RatCaptureSourceRfcGpi0;
    config.captureMode = RF_RatCaptureModeBoth; // 上下边沿触发
    config.repeat = RF_RatCaptureRepeat;
    RF_RatHandle ratHandle = RF_ratCapture(RFDriver, &config, 0);

    Display_printf(display, 0, 0, "NanoEEG cc1310 ready!");

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

    while (1) {

    }
}
