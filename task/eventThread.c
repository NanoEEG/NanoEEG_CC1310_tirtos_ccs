/**
 * @file    eventThread.c
 * @author  gjmsilly
 * @brief   事件线程：处理事件标签向cc3235s传输
 * @version 0.0.1
 * @date    2021-12-28
 *
 * @copyright (c) 2021 gjmsilly
 *
 */

/********************************************************************
 *  MACROS
 */
#define CC1310_SLAVEADDR    0x01

/********************************************************************
 *  INCLUDES
 */

/* POSIX Header files */
#include <semaphore.h>

#include <NanoEEG_CC1310.h>

#include <ti/drivers/PIN.h>
#include <ti/devices/cc13x0/driverlib/i2c.h>
#include <ti/devices/cc13x0/driverlib/prcm.h>
#include <ti/devices/cc13x0/driverlib/ioc.h>
#include <ti/devices/cc13x0/inc/hw_memmap.h>

/*********************************************************************
 *  EXTERNAL VARIABLES
 */
extern sem_t EvtDataRecv;


/********************************************************************
 *  LOCAL FUNCTIONS
 */
static void cc1310_I2C_init(){

    /* enable the power domain */
    PRCMPeripheralRunEnable(PRCM_PERIPH_I2C0);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_I2C0);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_I2C0);

    PRCMLoadSet();

    /* configure the IOC to route SDA SCL signal from IO to I2C module */
    IOCPortConfigureSet(CC1310_LAUNCHXL_I2C0_SCL0,\
                        IOC_PORT_MCU_I2C_MSSCL,\
                        IOC_IOMODE_OPEN_DRAIN_NORMAL|\
                        IOC_NO_EDGE|\
                        IOC_INT_DISABLE|\
                        IOC_IOPULL_UP|\
                        IOC_INPUT_ENABLE);

    IOCPortConfigureSet(CC1310_LAUNCHXL_I2C0_SDA0,\
                        IOC_PORT_MCU_I2C_MSSDA,\
                        IOC_IOMODE_OPEN_DRAIN_NORMAL|\
                        IOC_NO_EDGE|\
                        IOC_INT_DISABLE|\
                        IOC_IOPULL_UP|\
                        IOC_INPUT_ENABLE);

    /* set the cc1310 as I2C slave */
    I2CSlaveInit(I2C0_BASE,CC1310_SLAVEADDR);
}



/*
 *  ======== eventThread ========
 */
void *eventThread(void *arg0){

    PIN_State   WAKEUP_PIN;

    // Get handle to this collection of pins
    if (!PIN_open(&WAKEUP_PIN, BoardGpioInitTable)) {
        // Handle allocation error
    }
    /* initial cc1310 I2C as slave */
    cc1310_I2C_init();

    // 测试版本：信号量由主线程定时器释放

    while(1){

        // 等待事件标签接收 信号量
        sem_wait(&EvtDataRecv);

        // 翻转IO 通知cc3235s发起I2C传输
        PIN_setOutputValue(&WAKEUP_PIN,CC1310_LAUNCHXL_WAKEUP,~PIN_getOutputValue(CC1310_LAUNCHXL_WAKEUP));

        // 轮询等待cc3235s发起事件标签传输
        while( I2CSlaveStatus(I2C0_BASE)!= I2C_SLAVE_ACT_TREQ);
        I2CSlaveDataPut(I2C0_BASE,0xaa);

    }

}
