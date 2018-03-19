/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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

/*
 *  ======== uartecho.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

#include <ti/drivers/ADC.h>
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Watchdog.h>

/* Example/Board Header files */
#include "EPS_Board.h"

#include "satellite.h"
#include "devices.h"

//#include "INA226.h"
//#include "TMP100.h"

int flag = 0;

void temp(UART_Handle handle, int *readBuf, int cnt) {
    flag = 1;
}

bool retVal = false;
I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;

/* Buffers used in this code example */
uint8_t             txBuffer[10];
uint8_t             rxBuffer[10];

SPI_Handle spi_fram2;

uint8_t rx[15] = {0};
uint8_t tx[15];

/* Services Header files */
//#include "packet_handling.h"

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    char        input;
    const char  echoPrompt[] = "Echoing characters:\r\n";

    UART_Params uartParams;

    /* Call driver init functions */
    GPIO_init();
    UART_init();
    I2C_init();
    SPI_init();

    /* Turn on user LED */
    GPIO_write(PQ9_EN, 1);

    /* Turn on subsystem en switches*/
    GPIO_write(SBSYS_EN_SW0, 1);
    GPIO_write(SBSYS_EN_SW1, 1);
    GPIO_write(SBSYS_EN_SW2, 1);
    GPIO_write(SBSYS_EN_SW3, 1);

    UART_Handle uart_dbg_bus;

    UART_Params_init(&uartParams);
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_ON;
    uartParams.baudRate = 9600;
    uart_dbg_bus = UART_open(DBG, &uartParams);

    UART_write(uart_dbg_bus, "Hello\n", 10);

    /* Create a UART with data processing off. */
   //UART_Params_init(&uartParams);
//    uartParams.writeMode = UART_MODE_BLOCKING;
//    uartParams.writeDataMode = UART_DATA_BINARY;
//    uartParams.readMode = UART_MODE_CALLBACK;
//    uartParams.readDataMode = UART_DATA_BINARY;
//    uartParams.readReturnMode = UART_RETURN_FULL;
//    uartParams.readEcho = UART_ECHO_OFF;
//    uartParams.baudRate = 9600;
//    uartParams.readCallback = &temp;
//
//    uart_pq9_bus = UART_open(PQ9, &uartParams);
//
//    if (uart_pq9_bus == NULL) {
//        /* UART_open() failed */
//        while (1);
//    }
//
//    /* I2C ina226 read manufacture id test*/
//    /* Initialize all buffers */
//    for (uint8_t i = 0; i < 10; i++) {
//        rxBuffer[i] = 0x00;
//        txBuffer[i] = 0x00;
//    }
//
//    I2C_Params_init(&i2cParams);
//    i2cParams.transferMode = I2C_MODE_BLOCKING;
//    i2cParams.bitRate = I2C_100kHz;
//
//    //i2c = I2C_open(Board_I2C0, &i2cParams);
//    if (i2c == NULL) {
//       // while (1);
//    }
//
//    /*
//     * FEh Manufacturer ID Register
//     */
//    txBuffer[0] = 0xFE;
//
//    /*
//     * Response from slave for GETSTATUS Cmd
//     * rxBuffer[0] = status
//     */
//    i2cTransaction.slaveAddress = 0x48;
//    i2cTransaction.writeBuf = txBuffer;
//    i2cTransaction.writeCount = 1;
//    i2cTransaction.readBuf = rxBuffer;
//    i2cTransaction.readCount = 2;
//
//    /* Re-try writing to slave till I2C_transfer returns true */
//    //do {
//       //retVal = I2C_transfer(i2c, &i2cTransaction);
//    //} while(!retVal);
//
//
//                /* I2C LTC2942 test*/
//                /* Initialize all buffers */
//                for (uint8_t i = 0; i < 10; i++) {
//                    rxBuffer[i] = 0x00;
//                    txBuffer[i] = 0x00;
//                }
//
//                I2C_Params_init(&i2cParams);
//                i2cParams.transferMode = I2C_MODE_BLOCKING;
//                i2cParams.bitRate = I2C_100kHz;
//
//                i2c = I2C_open(EPS_BATT, &i2cParams);
//                if (i2c == NULL) {
//                    while (1);
//                }
//
//                /*
//                 * READ LTC2942 configuration Register
//                 */
//                txBuffer[0] = 0x01;
//                txBuffer[1] = 0xF8;
//                /*
//                 * Response from slave for GETSTATUS Cmd
//                 * rxBuffer[0] = status
//                 */
//                i2cTransaction.slaveAddress = 0x64;
//                i2cTransaction.writeBuf = txBuffer;
//                i2cTransaction.writeCount = 1;
//                i2cTransaction.readBuf = rxBuffer;
//                i2cTransaction.readCount = 1;
//
//                /* Re-try writing to slave till I2C_transfer returns true */
//                //do {
//                    retVal = I2C_transfer(i2c, &i2cTransaction);
//                //} while(!retVal);
//
//                /*
//                 * WRITE LTC2942 configuration Register
//                 */
//                txBuffer[0] = 0x01;
//                txBuffer[1] = 0xF8;
//                /*
//                 * Response from slave for GETSTATUS Cmd
//                 * rxBuffer[0] = status
//                 */
//                i2cTransaction.slaveAddress = 0x64;
//                i2cTransaction.writeBuf = txBuffer;
//                i2cTransaction.writeCount = 2;
//                i2cTransaction.readBuf = rxBuffer;
//                i2cTransaction.readCount = 0;
//
//                /* Re-try writing to slave till I2C_transfer returns true */
//                //do {
//                retVal = I2C_transfer(i2c, &i2cTransaction);
//
//                /*
//                 * Read temp
//                 */
//                txBuffer[0] = 0x0c;
//
//                /*
//                 * Response from slave for GETSTATUS Cmd
//                 * rxBuffer[0] = status
//                 */
//                i2cTransaction.slaveAddress = 0x64;
//                i2cTransaction.writeBuf = txBuffer;
//                i2cTransaction.writeCount = 1;
//                i2cTransaction.readBuf = rxBuffer;
//                i2cTransaction.readCount = 2;
//
//                uint16_t res_raw = 0;
//                float res_temp = 0.0;
//                /* Re-try writing to slave till I2C_transfer returns true */
//                for(uint8_t i = 0; i < 10; i++) {
//                    sleep(1);
//                    retVal = I2C_transfer(i2c, &i2cTransaction);
//                    res_raw = (uint16_t)((uint16_t)(rxBuffer[0] << 8) | (uint16_t)(rxBuffer[1]));
//                    res_temp = 600000 * ((float)res_raw / 65535);
//                }
//
//       /* I2C TP100 read manufacture id test*/
//       /* Initialize all buffers */
//       for (uint8_t i = 0; i < 10; i++) {
//            rxBuffer[i] = 0x00;
//            txBuffer[i] = 0x00;
//        }
//
//        I2C_Params_init(&i2cParams);
//        i2cParams.transferMode = I2C_MODE_BLOCKING;
//        i2cParams.bitRate = I2C_100kHz;
//
//        i2c = I2C_open(EPS_SOL, &i2cParams);
//        if (i2c == NULL) {
//             while (1);
//        }
//
//        /*
//         * TMP100 configuration Register
//         */
//        txBuffer[0] = 0x01;
//        txBuffer[1] = 0x18;
//        /*
//         * Response from slave for GETSTATUS Cmd
//         * rxBuffer[0] = status
//         */
//        i2cTransaction.slaveAddress = 0x48;
//        i2cTransaction.writeBuf = txBuffer;
//        i2cTransaction.writeCount = 2;
//        i2cTransaction.readBuf = rxBuffer;
//        i2cTransaction.readCount = 0;
//
//        /* Re-try writing to slave till I2C_transfer returns true */
//        //do {
//        retVal = I2C_transfer(i2c, &i2cTransaction);
//
//        /*
//         * Read temp
//         */
//        txBuffer[0] = 0x00;
//
//        /*
//         * Response from slave for GETSTATUS Cmd
//         * rxBuffer[0] = status
//         */
//        i2cTransaction.slaveAddress = 0x48;
//        i2cTransaction.writeBuf = txBuffer;
//        i2cTransaction.writeCount = 1;
//        i2cTransaction.readBuf = rxBuffer;
//        i2cTransaction.readCount = 2;
//
//        uint16_t res_raw2 = 0;
//        float res_temp2 = 0.0;
//        /* Re-try writing to slave till I2C_transfer returns true */
//        for(uint8_t i = 0; i < 10; i++) {
//            sleep(1);
//            retVal = I2C_transfer(i2c, &i2cTransaction);
//            res_raw2 = (uint16_t)((uint16_t)(rxBuffer[0] << 8) | (uint16_t)(rxBuffer[1])) >> 4;
//            res_temp2 = (float)res_raw * 0.0625;
//        }

   // SPI_Params spiParams;

   // SPI_Params_init(&spiParams);
   // spiParams.frameFormat = SPI_POL0_PHA0;
   // spiParams.bitRate = 10000;
   // spi_fram2 = SPI_open(EPS_FRAM, &spiParams);

   // SPI_Transaction spiTransaction;

   // uint8_t t1, t2;
   // spiTransaction.count = 1; //count;
   // spiTransaction.txBuf = (void *)&t1; //writeBuf;
   // spiTransaction.rxBuf = (void *)&t2; //readBuf;

   // SPI_transfer(spi_fram2, &spiTransaction);

   // device_init();

    /*ECSS services start*/
   // pkt_pool_INIT();

    //UART_write(uart_pq9_bus, echoPrompt, sizeof(echoPrompt));

    uint8_t res = 0;
    //FRAM_read_Status(EPS_FRAM_DEV_ID, &res);



    for(uint16_t i=0; i <= 15; i++) {
        tx[i] = i+10;
    }
    for(uint16_t i=0; i < 200; i++) {
   //   FRAM_write(EPS_FRAM_DEV_ID, 0x16, tx, 15);
   //   FRAM_read(EPS_FRAM_DEV_ID, 0x16, rx, 15);
      for(uint32_t d = 0; d < 100000; d++) {}
    }

    //ina_readDeviceID(BATT_CHARGE_DEV_ID);
  //  update_device(1);
    /* Loop forever echoing */
    device_init();

    struct ina_device ina_dev_ur, ina_dev_dc;

    while (1) {
        //UART_read(uart_pq9_bus, &input, 1);
        //if(flag) {
        //    UART_write(uart_pq9_bus, &input, 1);
        //    flag = 0;
        //}
        update_device(EPS_UR_MON_DEV_ID);
        read_device_parameters(EPS_UR_MON_DEV_ID, &ina_dev_ur);
        for(uint32_t d = 0; d < 100000; d++) {}

        update_device(EPS_DC_MON_DEV_ID);
        read_device_parameters(EPS_DC_MON_DEV_ID, &ina_dev_dc);
        for(uint32_t d = 0; d < 100000; d++) {}
        import_pkt();
        export_pkt();
    }
}
