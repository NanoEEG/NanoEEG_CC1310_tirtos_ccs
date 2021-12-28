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

/* Driver Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>

/* Example/Board Header files */
#include "Board.h"

/* POSIX Header files */
#include <semaphore.h>

/********************************************************************************
 *  GLOBAL VARIABLES
 */
Display_Handle display = NULL;
sem_t EvtDataRecv;
PIN_State   LEDPIN;

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    /* Call driver init functions */
    Display_init();

    // Get handle to this collection of pins
    if (!PIN_open(&LEDPIN, BoardGpioInitTable)) {
        // Handle allocation error
    }

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

    Display_printf(display, 0, 0, "NanoEEG cc1310 ready!");

    /* led on to indicate the system is ready! */
    PIN_setOutputValue(&LEDPIN, CC1310_LAUNCHXL_PIN_BLED, CC1310_LAUNCHXL_PIN_LED_ON);

    //测试版本
    //定时器2s释放信号量

    while (1) {

        //PIN_setOutputValue(&LEDPIN, CC1310_LAUNCHXL_PIN_BLED, ~PIN_getOutputValue(CC1310_LAUNCHXL_PIN_BLED));
    }
}
