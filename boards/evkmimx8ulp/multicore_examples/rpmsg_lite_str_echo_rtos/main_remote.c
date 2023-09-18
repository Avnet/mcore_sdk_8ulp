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
#include "fsl_pdm.h"
#include "fsl_sai.h"

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

/* PDM */
#define DEMO_PDM                      PDM
#define DEMO_PDM_CLK_FREQ             CLOCK_GetMicfilFreq()
#define DEMO_PDM_FIFO_WATERMARK       (FSL_FEATURE_PDM_FIFO_DEPTH / 2U - 1U)
#define DEMO_PDM_QUALITY_MODE         kPDM_QualityModeHigh
#define DEMO_PDM_CIC_OVERSAMPLE_RATE  (0U)
#define DEMO_PDM_ENABLE_CHANNEL_LEFT  (0U)
#define DEMO_PDM_ENABLE_CHANNEL_RIGHT (1U)
#define DEMO_PDM_HWVAD_SIGNAL_GAIN    0
#define DEMO_PDM_CHANNEL_GAIN         kPDM_DfOutputGain5
#define DEMO_AUDIO_SAMPLE_RATE        (kSAI_SampleRate16KHz)
#define DEMO_PDM_FRAMESIZE            (DEMO_PDM_FIFO_WATERMARK*FSL_FEATURE_PDM_FIFO_WIDTH*2U) /* 24 Bytes */

static uint8_t      pdm_workflag = 0;
static uint8_t      pdm_buf[DEMO_PDM_FRAMESIZE*100];

static const pdm_config_t pdmConfig         = {
    .enableDoze        = false,
    .fifoWatermark     = DEMO_PDM_FIFO_WATERMARK,
    .qualityMode       = DEMO_PDM_QUALITY_MODE,
    .cicOverSampleRate = DEMO_PDM_CIC_OVERSAMPLE_RATE,
};

static pdm_channel_config_t channelConfig = {
#if (defined(FSL_FEATURE_PDM_HAS_DC_OUT_CTRL) && (FSL_FEATURE_PDM_HAS_DC_OUT_CTRL))
    .outputCutOffFreq = kPDM_DcRemoverCutOff40Hz,
#else
    .cutOffFreq = kPDM_DcRemoverCutOff152Hz,
#endif
#ifdef DEMO_PDM_CHANNEL_GAIN
    .gain       = DEMO_PDM_CHANNEL_GAIN,
#else
    .gain       = kPDM_DfOutputGain7,
#endif
};

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

typedef struct gpio_pin_s
{
    char        *desc;
    RGPIO_Type  *base;
    uint32_t     pin;
} gpio_pin_t;

int gpio_pinloop_test(gpio_pin_t *gpio1, gpio_pin_t *gpio2)
{
    rgpio_pin_config_t out_config = { kRGPIO_DigitalOutput, 0 };
    rgpio_pin_config_t in_config =  { kRGPIO_DigitalInput,  0 };

    /*+-------------------+
     *|  gpio1 <-- gpio2  |
     *+-------------------+*/
    RGPIO_PinInit(gpio1->base, gpio1->pin, &in_config);
    RGPIO_PinInit(gpio2->base, gpio2->pin, &out_config);

    RGPIO_WritePinOutput(gpio2->base, gpio2->pin, 1);
    vTaskDelay( pdMS_TO_TICKS(2) );
    if( RGPIO_PinRead(gpio1->base, gpio1->pin) != 1 )
    {
        PRINTF("%4s <-- %4s test[1] fail\r\n", gpio1->desc, gpio2->desc);
        return 1;
    }

    RGPIO_WritePinOutput(gpio2->base, gpio2->pin, 0);
    vTaskDelay( pdMS_TO_TICKS(2) );
    if( RGPIO_PinRead(gpio1->base, gpio1->pin) != 0 )
    {
        PRINTF("%4s <-- %4s test[0] fail\r\n", gpio1->desc, gpio2->desc);
        return 2;
    }

    return 0;
}

int mikrobus_test(void)
{
    int                i, rv = 0;

    /* MISO and INT pins can only be set as input mode, so we use loop test */
    gpio_pin_t   mikrobus_loopins[] =
    {
        {.desc="MISO", .base=GPIOB, .pin=4U  }, /* 05#, input  */
        {.desc="MOSI", .base=GPIOB, .pin=3U  }, /* 06#, output */
        {.desc="INT",  .base=GPIOB, .pin=12U }, /* 15#, input  */
        {.desc="PWM",  .base=GPIOA, .pin=8U  }, /* 16#, output */
        {.desc="RXD",  .base=GPIOA, .pin=15U }, /* 14#, input */
        {.desc="TXD",  .base=GPIOA, .pin=18U }, /* 13#, output */
    };

    PRINTF("\r\n");
    PRINTF("+----------------------------------+\r\n");
    PRINTF("|       MikroBUS Test Case         |\r\n");
    PRINTF("+----------------------------------+\r\n");
    PRINTF("\r\n");

    /*+-----------------+
     *|  Loop pins test |
     *+-----------------+*/
    for(i=0; i<ARRAY_SIZE(mikrobus_loopins); i+=2 )
    {
        rv |= gpio_pinloop_test(&mikrobus_loopins[i], &mikrobus_loopins[i+1]);
    }
    PRINTF("MikroBUS loop pins test                              %s\r\n", rv ? "\e[31m[**FAIL**]\e[0m" : "\e[32m[  OKAY  ]\e[0m");

    return rv;
}

int show_mikrobus_led(int status)
{
    int                i, idx;
    rgpio_pin_config_t out_config = { kRGPIO_DigitalOutput, 0 };

    /* Below pins can only be set as output mode, so we use LED test */
    gpio_pin_t   mikrobus_ledpins[] = /* GPIO led test pins */
    {
        {.desc="AN",   .base=GPIOB, .pin=2U  }, /* 01#, test status indicate led */
        {.desc="RST",  .base=GPIOB, .pin=14U }, /* 02# */
        {.desc="CS",   .base=GPIOB, .pin=6U  }, /* 03# */
        {.desc="SCK",  .base=GPIOB, .pin=5U  }, /* 04# */
        /*   #11(SDA), #12(SCL) -> DA7212    */
    };

    PRINTF("\r\n");
    PRINTF("+----------------------------------+\r\n");
    PRINTF("|       Show MikroBUS Leds         |\r\n");
    PRINTF("+----------------------------------+\r\n");
    PRINTF("\r\n");

    PRINTF("MikroBUS led test                                    \e[32m[  WATCH ]\e[0m\r\n");

    /* set all pins as GPIO output mode */
    for(i=0; i<ARRAY_SIZE(mikrobus_ledpins); i++ )
        RGPIO_PinInit(mikrobus_ledpins[i].base, mikrobus_ledpins[i].pin, &out_config);

    if( kStatus_Success != status )
    {
        /* Get test case failed will turn the indicate led off constantly */
        RGPIO_WritePinOutput(mikrobus_ledpins[0].base, mikrobus_ledpins[0].pin, 0);
        idx = 1;
    }
    else
    {
        /* All test case pass will turn the indicate led blink */
        idx = 0;
    }

    /*+-----------------+
     *|  Led pins test  |
     *+-----------------+*/

    /* infinite loop here */
    while(1)
    {
        for( i=idx; i<ARRAY_SIZE(mikrobus_ledpins); i++ )
            RGPIO_WritePinOutput(mikrobus_ledpins[i].base, mikrobus_ledpins[i].pin, 1);

        vTaskDelay( pdMS_TO_TICKS(1000) );

        for( i=idx; i<ARRAY_SIZE(mikrobus_ledpins); i++ )
            RGPIO_WritePinOutput(mikrobus_ledpins[i].base, mikrobus_ledpins[i].pin, 0);

        vTaskDelay( pdMS_TO_TICKS(1000) );
    }
}

int pdm_test(void)
{
    uint32_t i = 0;
    status_t status = kStatus_Fail;

    PRINTF("\r\n");
    PRINTF("+----------------------------------+\r\n");
    PRINTF("|           PDM Test Case          |\r\n");
    PRINTF("+----------------------------------+\r\n");
    PRINTF("\r\n");

    PRINTF("PDM record test                                      ");

    /* PDM chipset not present will get interrupt too, but all the data is 0x00 */
    if( pdm_workflag )
    {
        for(i=0; i<sizeof(pdm_buf); i++)
        {
            if( pdm_buf[i] != 0x00 )
            {
                status = kStatus_Success;
                break;
            }
        }
    }

    PRINTF("%s", kStatus_Success==status ? "\e[32m[  OKAY  ]\e[0m\r\n" : "\e[31m[**FAIL**]\e[0m\r\n");
    return status;
}

int PDM_Setup(void);

void test_task(void *param)
{
    uint8_t  status = 0;

    vTaskDelay( pdMS_TO_TICKS(6000) );

    PDM_Setup();

    if( kStatus_Success != psram_test() )
    {
        status |= (1<<0);
    }

    if( kStatus_Success != flash_test() )
    {
        status |= (1<<1);
    }

    if( kStatus_Success != mikrobus_test() )
    {
        status |= (1<<2);
    }

    if( kStatus_Success != pdm_test() )
    {
        status |= (1<<3);
    }

    show_mikrobus_led(status);

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

static void pdm_error_irqHandler(void)
{
    uint32_t status = 0U;

#if (defined(FSL_FEATURE_PDM_HAS_STATUS_LOW_FREQ) && (FSL_FEATURE_PDM_HAS_STATUS_LOW_FREQ == 1U))
    if (PDM_GetStatus(DEMO_PDM) & PDM_STAT_LOWFREQF_MASK)
    {
        PDM_ClearStatus(DEMO_PDM, PDM_STAT_LOWFREQF_MASK);
    }
#endif

    status = PDM_GetFifoStatus(DEMO_PDM);
    if (status != 0U)
    {
        PDM_ClearFIFOStatus(DEMO_PDM, status);
    }

#if defined(FSL_FEATURE_PDM_HAS_RANGE_CTRL) && FSL_FEATURE_PDM_HAS_RANGE_CTRL
    status = PDM_GetRangeStatus(DEMO_PDM);
    if (status != 0U)
    {
        PDM_ClearRangeStatus(DEMO_PDM, status);
    }
#else
    status = PDM_GetOutputStatus(DEMO_PDM);
    if (status != 0U)
    {
        PDM_ClearOutputStatus(DEMO_PDM, status);
    }
#endif

    return ;
}

#if !(defined FSL_FEATURE_PDM_HAS_NO_INDEPENDENT_ERROR_IRQ && FSL_FEATURE_PDM_HAS_NO_INDEPENDENT_ERROR_IRQ)
void PDM_ERROR_IRQHandler(void)
{
    pdm_error_irqHandler();
    __DSB();
}
#endif

void PDM_EVENT_IRQHandler(void)
{
    uint32_t status = PDM_GetStatus(DEMO_PDM);
    static uint32_t  frame = 0;

#if (defined FSL_FEATURE_PDM_HAS_NO_INDEPENDENT_ERROR_IRQ && FSL_FEATURE_PDM_HAS_NO_INDEPENDENT_ERROR_IRQ)
    pdm_error_irqHandler();
#endif

    if ((1U << DEMO_PDM_ENABLE_CHANNEL_LEFT) & status)
    {
        PDM_ReadFifo(DEMO_PDM, DEMO_PDM_ENABLE_CHANNEL_LEFT, 2, &pdm_buf[frame*DEMO_PDM_FRAMESIZE], DEMO_PDM_FIFO_WATERMARK, FSL_FEATURE_PDM_FIFO_WIDTH);
        frame=(frame+1)%100;
        pdm_workflag = 1;
    }

    PDM_ClearStatus(DEMO_PDM, status);
    __DSB();
}

int PDM_Setup(void)
{
    CLOCK_SetIpSrc(kCLOCK_Micfil, kCLOCK_FusionMicfilClkSrcPll1Pfd2Div);
    RESET_PeripheralReset(kRESET_Micfil);

    PDM_Init(DEMO_PDM, &pdmConfig);
    PDM_SetChannelConfig(DEMO_PDM, DEMO_PDM_ENABLE_CHANNEL_LEFT, &channelConfig);
    PDM_SetChannelConfig(DEMO_PDM, DEMO_PDM_ENABLE_CHANNEL_RIGHT, &channelConfig);
    if (PDM_SetSampleRateConfig(DEMO_PDM, DEMO_PDM_CLK_FREQ, DEMO_AUDIO_SAMPLE_RATE) != kStatus_Success)
    {
        PRINTF("PDM configure sample rate failed.\r\n");
        return -1;
    }

    PDM_Reset(DEMO_PDM);
    PDM_EnableInterrupts(DEMO_PDM, kPDM_ErrorInterruptEnable | kPDM_FIFOInterruptEnable);
    EnableIRQ(PDM_EVENT_IRQn);
#if !(defined FSL_FEATURE_PDM_HAS_NO_INDEPENDENT_ERROR_IRQ && FSL_FEATURE_PDM_HAS_NO_INDEPENDENT_ERROR_IRQ)
    EnableIRQ(PDM_ERROR_IRQn);
#endif
    PDM_Enable(DEMO_PDM, true);

    return 0;
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
