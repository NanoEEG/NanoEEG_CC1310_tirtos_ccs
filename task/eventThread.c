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
#define CC1310_SLAVEADDR    0x33

#define GET_LOW_BYTE0(x) ((x >>  0) & 0x000000ff) /* 获取第0个字节 */
#define GET_LOW_BYTE1(x) ((x >>  8) & 0x000000ff) /* 获取第1个字节 */
#define GET_LOW_BYTE2(x) ((x >> 16) & 0x000000ff) /* 获取第2个字节 */
#define GET_LOW_BYTE3(x) ((x >> 24) & 0x000000ff) /* 获取第3个字节 */

/********************************************************************
 *  INCLUDES
 */
/* Example/Board Header files */
#include "Board.h"

/* POSIX Header files */
#include <semaphore.h>

#include <NanoEEG_CC1310.h>

/* RTOS driver header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* Driverlib header files */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(driverlib/i2c.h)
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include DeviceFamily_constructPath(driverlib/prcm.h)

#include <task/DataType.h>

/*********************************************************************
 *  GLOBAL VARIABLES
 */
I2CBuff_t I2C_BUFF;

/*********************************************************************
 *  EXTERNAL VARIABLES
 */
extern sem_t EvtDataRecv;

/********************************************************************
 *  LOCAL FUNCTIONS
 */
static void cc1310_I2C_init(){

    /* Power on the I2C module */
    Power_setDependency(PowerCC26XX_PERIPH_I2C0);

    /* Set constraints for Standby, powerdown and idle mode */
    Power_setConstraint(PowerCC26XX_SB_DISALLOW);
    Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);

    PRCMLoadSet();
    PRCMPeripheralRunEnable(PRCM_PERIPH_I2C0); // Enable I2C module
    PRCMLoadSet();
    while(!PRCMLoadGet());

    I2CSlaveDisable(I2C0_BASE);

    /* configure the IOC to route SDA SCL signal from IO to I2C module */
    IOCPinTypeI2c(I2C0_BASE,CC1310_LAUNCHXL_I2C0_SDA0,CC1310_LAUNCHXL_I2C0_SCL0);

    I2CSlaveIntDisable(I2C0_BASE, I2C_SLAVE_INT_STOP|\
                       I2C_SLAVE_INT_START|\
                       I2C_SLAVE_INT_DATA);

    /* set the cc1310 as I2C slave */
    I2CSlaveInit(I2C0_BASE,CC1310_SLAVEADDR);

}


/*
 *  ======== eventThread ========
 */
void *eventThread(void *arg0){

    int i = 0;
    /* initial cc1310 I2C as slave */
    cc1310_I2C_init();
    uint32_t test = 0xaabbccdd;

    while(1){

        // 等待事件标签接收 信号量
        sem_wait(&EvtDataRecv);

        // 轮询等待cc3235s发起事件标签传输
        while(I2CSlaveStatus(I2C0_BASE)!=I2C_SLAVE_ACT_TREQ);
        //I2CSlaveDataPut(I2C0_BASE,I2C_BUFF.Index);
        I2CSlaveDataPut(I2C0_BASE,0x11);
        // LSB
        for (i=0; i<4; i++){
            while(I2CSlaveStatus(I2C0_BASE)!=I2C_SLAVE_ACT_TREQ);
            //I2CSlaveDataPut(I2C0_BASE,(uint8_t)(I2C_BUFF.Tror>>(i*8)));
            I2CSlaveDataPut(I2C0_BASE,(uint8_t)(test>>(i*8)));
        }

        for (i=0; i<4; i++){
            while(I2CSlaveStatus(I2C0_BASE)!=I2C_SLAVE_ACT_TREQ);
            //I2CSlaveDataPut(I2C0_BASE,(uint8_t)(I2C_BUFF.Tsor>>(i*8)));
            I2CSlaveDataPut(I2C0_BASE,0xbb);
        }
        while(I2CSlaveStatus(I2C0_BASE)!=I2C_SLAVE_ACT_TREQ);
        //I2CSlaveDataPut(I2C0_BASE,I2C_BUFF.Type);
        I2CSlaveDataPut(I2C0_BASE,0x01);
    }

}
