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
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/Timer.h>

/* Example/Board Header files */
#include "EPS_Board.h"

#include "satellite.h"
#include "devices.h"

#include "INA226.h"
#include "TMP100.h"

extern UART_Handle uart_dbg_bus;
extern UART_Handle uart_pq9_bus;

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    /* Call driver init functions */
    GPIO_init();
    UART_init();
    I2C_init();
    SPI_init();
    Timer_init();

    /* Turn on user LED */
    GPIO_write(PQ9_EN, 1);
    GPIO_write(PQ9_EN, 0);

    /* Turn on subsystem en switches*/
    GPIO_write(SBSYS_EN_SW0, 1);
    GPIO_write(SBSYS_EN_SW1, 1);
    GPIO_write(SBSYS_EN_SW2, 1);
    GPIO_write(SBSYS_EN_SW3, 1);

    /*ECSS services start*/
    pkt_pool_INIT();
    device_init();

    /* Loop forever echoing */


    while (1) {

        update_device(EPS_UR_MON_DEV_ID);
        usleep(1);

        update_device(EPS_DC_MON_DEV_ID);
        usleep(1);

        update_device(EPS_SU_MON_DEV_ID);
        usleep(1);

        update_device(EPS_OBC_MON_DEV_ID);
        usleep(1);

        update_device(EPS_COMMS_MON_DEV_ID);
        usleep(1);

        update_device(EPS_ADCS_MON_DEV_ID);
        usleep(1);

        //sol inas
        //sol temps

        //ltc

        usleep(100);

    }
}

/*  ======== ecssThread ========
 *  This thread runs on a higher priority, since wdg pin
 *  has to be ready for master.
 */
void *ecssThread(void *arg0)
{

    sleep(1);

    /* Loop forever */
    while (1) {
         import_pkt();
         export_pkt();
         usleep(1000);
    }

    return (NULL);
}

char msg[100];

/*  ======== senThread ========
 *  This a dbg thread for outputing sensor readings
 */
void *senThread(void *arg0)
{

    struct ina_device ina_dev;

    sprintf(msg, "Reset\n");
    UART_write(uart_dbg_bus, msg, strlen(msg));

    sleep(1);

    /* Loop forever */
    while (1) {

        //sol temp and inas
        //ltc

        //SOL_XM_MON_DEV_ID
        for(uint8_t i=EPS_OBC_MON_DEV_ID; i <= EPS_UR_MON_DEV_ID; i++) {
            read_device_parameters(i, &ina_dev);
            sprintf(msg, "INA: %d, C %d, V %d, W %d\n", i, (int)(ina_dev.current*1000), (int)ina_dev.voltage, (int)ina_dev.power);
            UART_write(uart_dbg_bus, msg, strlen(msg));

            sleep(1);
        }

    }

    return (NULL);
}
