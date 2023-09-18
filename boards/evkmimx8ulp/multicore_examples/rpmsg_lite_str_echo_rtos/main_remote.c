/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "version.h"
#include "fsl_fusion.h"

#include "fsl_reset.h"
#include "app_srtm.h"
#include "fsl_upower.h"
extern void app_create_task(void);
extern void app_destroy_task(void);
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RPMSG_LITE_LINK_ID            (RL_PLATFORM_IMX8ULP_M33_A35_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE         (VDEV1_VRING_BASE)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-virtual-tty-channel"
#define RPMSG_LITE_MASTER_IS_LINUX

#define APP_DEBUG_UART_BAUDRATE (115200U) /* Debug console baud rate. */
#define APP_TASK_STACK_SIZE (256)
#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30)
#endif

/* Globals */
static char app_buf[512]; /* Each RPMSG buffer can carry less than 512 payload */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern int psram_test(void);
extern int flash_test(void);

/*******************************************************************************
 * Code
 ******************************************************************************/
void app_rpmsg_monitor(struct rpmsg_lite_instance *rpmsgHandle, bool ready, void *rpmsgMonitorParam)
{
    if (ready)
    {
        app_create_task();
    }
    else
    {
        app_destroy_task();
    }
}

static TaskHandle_t app_task_handle = NULL;

static struct rpmsg_lite_instance *volatile my_rpmsg = NULL;

static struct rpmsg_lite_endpoint *volatile my_ept = NULL;
static volatile rpmsg_queue_handle my_queue        = NULL;
void app_destroy_task(void)
{
    if (app_task_handle)
    {
        vTaskDelete(app_task_handle);
        app_task_handle = NULL;
    }

    if (my_ept)
    {
        rpmsg_lite_destroy_ept(my_rpmsg, my_ept);
        my_ept = NULL;
    }

    if (my_queue)
    {
        rpmsg_queue_destroy(my_rpmsg, my_queue);
        my_queue = NULL;
    }

    if (my_rpmsg)
    {
        rpmsg_lite_deinit(my_rpmsg);
        my_rpmsg = NULL;
    }
}

void app_task(void *param)
{
    volatile uint32_t remote_addr;
    void *rx_buf;
    uint32_t len;
    int32_t result;
    void *tx_buf;
    uint32_t size;

    /* Print the initial banner */
    PRINTF("\r\nRPMSG String Echo FreeRTOS RTOS API Demo (%s-%s) ...\r\n", SDK_VERSION, GIT_VERSION);

#ifdef MCMGR_USED
    uint32_t startupData;

    /* Get the startup data */
    (void)MCMGR_GetStartupData(kMCMGR_Core1, &startupData);

    my_rpmsg = rpmsg_lite_remote_init((void *)startupData, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);

    /* Signal the other core we are ready */
    (void)MCMGR_SignalReady(kMCMGR_Core1);
#else
    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
#endif /* MCMGR_USED */

    rpmsg_lite_wait_for_link_up(my_rpmsg, RL_BLOCK);

    my_queue = rpmsg_queue_create(my_rpmsg);
    my_ept   = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    (void)rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);

    PRINTF("\r\nNameservice sent, ready for incoming messages...\r\n");

    for (;;)
    {
        /* Get RPMsg rx buffer with message */
        result =
            rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (uint32_t *)&remote_addr, (char **)&rx_buf, &len, RL_BLOCK);
        if (result != 0)
        {
            assert(false);
        }

        /* Copy string from RPMsg rx buffer */
        assert(len < sizeof(app_buf));
        memcpy(app_buf, rx_buf, len);
        app_buf[len] = 0; /* End string by '\0' */

        if ((len == 2) && (app_buf[0] == 0xd) && (app_buf[1] == 0xa))
            PRINTF("Get New Line From Master Side\r\n");
        else
            PRINTF("Get Message From Master Side : \"%s\" [len : %d]\r\n", app_buf, len);

        /* Get tx buffer from RPMsg */
        tx_buf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &size, RL_BLOCK);
        assert(tx_buf);
        /* Copy string to RPMsg tx buffer */
        memcpy(tx_buf, app_buf, len);
        /* Echo back received message with nocopy send */
        result = rpmsg_lite_send_nocopy(my_rpmsg, my_ept, remote_addr, tx_buf, len);
        if (result != 0)
        {
            assert(false);
        }
        /* Release held RPMsg rx buffer */
        result = rpmsg_queue_nocopy_free(my_rpmsg, rx_buf);
        if (result != 0)
        {
            assert(false);
        }
    }
}

void test_task(void *param)
{
    uint8_t  status = 0;

    vTaskDelay( pdMS_TO_TICKS(6000) );

    if( kStatus_Success != psram_test() )
    {
        status |= (1<<0);
    }

    if( kStatus_Success != flash_test() )
    {
        status |= (1<<1);
    }

    PRINTF("\r\n");
    PRINTF("All the test case                                    [  DONE  ]\r\n");
    PRINTF("\r\n");

    vTaskDelete(NULL);
}

void app_create_task(void)
{
    if (app_task_handle == NULL &&
        xTaskCreate(app_task, "APP_TASK", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &app_task_handle) != pdPASS)
    {
        PRINTF("\r\nFailed to create application task\r\n");
        for (;;)
            ;
    }
#ifdef CONFIG_QCTEST
    xTaskCreate(test_task, "TEST_TASK", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
#endif
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Initialize standard SDK demo application pins */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_DMA0);

    Fusion_Init();
    /* Must handshake with uboot, unless will get issues(such as: SoC reset all the time) */
    BOARD_HandshakeWithUboot_InAdvance();
    CLOCK_SetIpSrc(kCLOCK_Tpm3, kCLOCK_Pcc2BusIpSrcFusionDspBus);
    RESET_PeripheralReset(kRESET_Tpm3);

    CLOCK_SetIpSrcDiv(kCLOCK_Tpm0, kCLOCK_Pcc1BusIpSrcCm33Bus, 1U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c0, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c1, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    /* Use Pll1Pfd2Div clock source 12.288MHz. */
    CLOCK_SetIpSrc(kCLOCK_Sai0, kCLOCK_Cm33SaiClkSrcPll1Pfd2Div);

    CLOCK_EnableClock(kCLOCK_Dma0Ch16);
    CLOCK_EnableClock(kCLOCK_Dma0Ch17);
    CLOCK_EnableClock(kCLOCK_RgpioA);
    CLOCK_EnableClock(kCLOCK_RgpioB);
    CLOCK_EnableClock(kCLOCK_RgpioC);
    CLOCK_EnableClock(kCLOCK_Wuu0);
    CLOCK_EnableClock(kCLOCK_Bbnsm);

    RESET_PeripheralReset(kRESET_Sai0);
    RESET_PeripheralReset(kRESET_Lpi2c0);
    RESET_PeripheralReset(kRESET_Lpi2c1);
    RESET_PeripheralReset(kRESET_Tpm0);

    APP_SRTM_Init();

    /* register callback for restart the app task when A35 reset */
    APP_SRTM_SetRpmsgMonitor(app_rpmsg_monitor, NULL);

    APP_SRTM_StartCommunication();

#ifdef MCMGR_USED
    /* Initialize MCMGR before calling its API */
    (void)MCMGR_Init();
#endif /* MCMGR_USED */

    app_create_task();
    vTaskStartScheduler();

    PRINTF("Failed to start FreeRTOS on core0.\n");
    for (;;)
        ;
}
