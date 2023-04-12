/*
 * Copyright 2021,2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "epdc_support.h"
#include "panel_data.h"
#include "fsl_epdc.h"
#include "fsl_pxp.h"

#include "fsl_reset.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PANEL_HEIGHT         758U
#define PANEL_WIDTH          1024U
#define DEMO_WB_ADDR         0x80000000 /* size: height x width x 2bytes/pixel */
#define DEMO_Y4C_ADDR        0x80600000 /* size: height x width x 1bytes/pixel */
#define DEMO_UPD_ADDR        0x80400000 /* size: height x width x 1bytes/pixel */
#define DEMO_WV_ADDR         0x80200000
#define DEMO_EPDC_IRQHandler EPDC_IRQHandler
#define APP_INIT_LUT           0 /* LUT 0 is reserved for panel initialization */
#define APP_PIC_SIZE           154U
#define APP_FRAME_BUFFER_ALIGN 8U
#define APP_SCALER             2U /* Enlarge the image. */
#define APP_BPP                4U /* PS uses RGB888 data as input, 4 byte/pixel */
#define APP_PXP_PS_FORMAT      kPXP_PsPixelFormatRGB888
#define APP_PXP_OUT_FORMAT     kPXP_OutputPixelFormatY8

/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_lutData[256U], APP_FRAME_BUFFER_ALIGN);
AT_NONCACHEABLE_SECTION_ALIGN(static uint32_t s_psBufferPxp[APP_PIC_SIZE][APP_PIC_SIZE],
                              APP_FRAME_BUFFER_ALIGN); /* Original RGB888 image. */
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_outputBuffer[APP_PIC_SIZE * APP_SCALER][APP_PIC_SIZE * APP_SCALER],
                              APP_FRAME_BUFFER_ALIGN); /* Resized Y8 image data generated by legacy flow. */
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_ditherOutputBuffer[APP_PIC_SIZE * APP_SCALER][APP_PIC_SIZE * APP_SCALER],
                              APP_FRAME_BUFFER_ALIGN); /* Y4 image data generated by dither engine */
uint8_t blackDot = 0U;
static epdc_update_config_t updateConfig;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint8_t DEMO_SearchTempIndex(const int8_t tempBounds[], uint32_t arraySize, int8_t temp)
{
    if (temp <= tempBounds[0])
    {
        return 0;
    }
    else if (temp >= tempBounds[arraySize - 1])
    {
        return arraySize - 1;
    }
    else
    {
        for (uint8_t i = 0; i < arraySize - 1; i++)
        {
            if ((temp > tempBounds[i]) && (temp <= tempBounds[i + 1]))
            {
                return i;
            }
        }
    }

    return 0;
}

static void APP_InitEpdc(void)
{
    uint8_t tempIndex;
    uint8_t temp;

    /* Init EPDC. */
    /* Get the panel temparature. */
    if (DEMO_GetEpdcTemp(&temp) != kStatus_Success)
    {
        PRINTF("\r\nGet panel temperature failed!\r\n");
    }
    tempIndex = DEMO_SearchTempIndex(tempRangeBounds, tempRangeBoundsLen, temp);

    /* Initialie the panel display configuration. */
    epdc_display_config_t displayConfig;
    (void)memset(&displayConfig, 0, sizeof(displayConfig));
    displayConfig.updSwizzle   = kEPDC_SwapAllBytes;
    displayConfig.wbType       = kEPDC_Wb16bit;
    displayConfig.resX         = PANEL_WIDTH;
    displayConfig.resY         = PANEL_HEIGHT;
    displayConfig.tempIdx      = tempIndex;
    displayConfig.waveformAddr = DEMO_WV_ADDR;
    displayConfig.wbAddr       = DEMO_WB_ADDR;
    EPDC_InitDisplay(EPDC, &displayConfig);

    /* Initialize the timing controller engine, including the source/gate driver, line/frame scan configuration. */
    epdc_tce_config_t tceConfig;
    (void)memset(&tceConfig, 0, sizeof(tceConfig));
    tceConfig.tceWbAddr                = DEMO_WB_ADDR;
    tceConfig.vscanHoldoff             = 4U;
    tceConfig.gdConfig.shiftDir        = kEPDC_ShiftLeft;
    tceConfig.gdConfig.gdoePol         = kEPDC_ActiveHigh;
    tceConfig.gdConfig.gdoeOffset      = 0U;
    tceConfig.gdConfig.gdspOffset      = 327U;
    tceConfig.gdConfig.gdClkOffset     = 19U;
    tceConfig.gdConfig.gdClkHigh       = 524U;
    tceConfig.sdConfig.sdlePol         = kEPDC_ActiveHigh;
    tceConfig.sdConfig.sdoePol         = kEPDC_ActiveHigh;
    tceConfig.sdConfig.pixelReverse    = kEPDC_DataReversed;
    tceConfig.sdConfig.sdceCount       = 1U;
    tceConfig.sdConfig.shiftDir        = kEPDC_ShiftLeft;
    tceConfig.sdConfig.clockHoldEnable = true;
    tceConfig.sdConfig.sdoezDelay      = 20U;
    tceConfig.sdConfig.sdoezWidth      = 10U;
    tceConfig.sdConfig.sdoedDelay      = 20U;
    tceConfig.sdConfig.sdoedWidth      = 10U;
    tceConfig.scanConfig.lineSync      = 12U;
    tceConfig.scanConfig.lineBegin     = 12U;
    tceConfig.scanConfig.lineEnd       = 76U;
    tceConfig.scanConfig.frameSync     = 2U;
    tceConfig.scanConfig.frameBegin    = 4U;
    tceConfig.scanConfig.frameEnd      = 5U;
    EPDC_ConfigTCE(EPDC, &tceConfig);
}

static void APP_PanelInitialization(void)
{
    (void)memset(&updateConfig, 0, sizeof(updateConfig));
    updateConfig.width            = PANEL_WIDTH;
    updateConfig.height           = PANEL_HEIGHT;
    updateConfig.fixedEnable      = true;
    updateConfig.fixedNpEnable    = true;
    updateConfig.npValue          = 0xFFU;
    updateConfig.fixedCpEnable    = true;
    updateConfig.cpValue          = 0xFFU;
    updateConfig.fullUpdateEnable = true;
    EPDC_UpdateDisplay(EPDC, &updateConfig);

    while ((EPDC_GetLutCompleteStatusFlags(EPDC) & (1U << APP_INIT_LUT)) == 0U)
    {
    }
}

static void APP_SetupBuffer(void)
{
    uint32_t i;

    /* Initialize working buffer. Y4 data use bit 4-7 in each 16 bits. F is for full white. */
    for (i = 0; i < (PANEL_WIDTH * PANEL_HEIGHT); i++)
    {
        ((uint16_t *)DEMO_WB_ADDR)[i] = 0x00F0;
    }

    /* Read in the PS buffer */
    memcpy((void *)s_psBufferPxp, (void *)avatar, APP_PIC_SIZE * APP_PIC_SIZE * APP_BPP);

    /* Panel waveform data. */
    memcpy((void *)DEMO_WV_ADDR, waveform, waveformSize);
}

static void APP_PxpConfigurePath(void)
{
    /* Set the path for legacy process flow, and the 2 histogram engines */
    PXP_SetPath(PXP, kPXP_Mux0SelectProcessSurfaceEngine);
    PXP_SetPath(PXP, kPXP_Mux3SelectRotation1Engine);
    PXP_SetPath(PXP, kPXP_Mux8SelectAlphaBlending0);
    PXP_SetPath(PXP, kPXP_Mux9SelectMux8);
    PXP_SetPath(PXP, kPXP_Mux11SelectLut);
    PXP_SetPath(PXP, kPXP_Mux14SelectMux11);
    PXP_SetPath(PXP, kPXP_Mux16SelectAluA);
    PXP_SetPath(PXP, kPXP_Mux17SelectAluA);

    /* Disable the other path muxs, in case 2 path flows exist causing signal competition. */
    PXP_SetPath(PXP, kPXP_Mux1SelectNone);
    PXP_SetPath(PXP, kPXP_Mux2SelectNone);
    PXP_SetPath(PXP, kPXP_Mux5SelectNone);
    PXP_SetPath(PXP, kPXP_Mux6SelectNone);
    PXP_SetPath(PXP, kPXP_Mux7SelectNone);
    PXP_SetPath(PXP, kPXP_Mux10SelectNone);
    PXP_SetPath(PXP, kPXP_Mux12SelectNone);
    PXP_SetPath(PXP, kPXP_Mux13SelectNone);
    PXP_SetPath(PXP, kPXP_Mux15SelectNone);
}

static void APP_PxpConfigureHistogram(void)
{
    pxp_histogram_config_t histConfig;

    histConfig.enable         = true;
    histConfig.lutValueOffset = 8U;
    histConfig.lutValueWidth  = 3U;
    histConfig.pParamValue    = NULL;
    histConfig.enableMask     = true;
    histConfig.maskValue0     = 1U;
    histConfig.maskValue1     = 0U;
    histConfig.maskOffset     = 64U;
    histConfig.maskWidth      = 0U;
    histConfig.condition      = kPXP_HistogramMaskEqual;
    histConfig.totalHeight    = APP_PIC_SIZE * APP_SCALER;
    histConfig.totalWidth     = APP_PIC_SIZE * APP_SCALER;

    /* Use histogram engine B for histogram process. */
    PXP_SetHistogramConfig(PXP, 1U, &histConfig);
}

static void APP_PxpConfigureCollision(void)
{
    pxp_histogram_config_t collisionConfig;

    collisionConfig.enable         = true;
    collisionConfig.lutValueOffset = 24U;
    collisionConfig.lutValueWidth  = 6U;
    collisionConfig.pParamValue    = NULL;
    collisionConfig.enableMask     = true;
    collisionConfig.maskValue0     = 1U;
    collisionConfig.maskValue1     = 0U;
    collisionConfig.maskOffset     = 65U;
    collisionConfig.maskWidth      = 0U;
    collisionConfig.condition      = kPXP_HistogramMaskEqual;
    collisionConfig.totalHeight    = APP_PIC_SIZE * APP_SCALER;
    collisionConfig.totalWidth     = APP_PIC_SIZE * APP_SCALER;

    /* Use histogram engine A for collision detection. */
    PXP_SetHistogramConfig(PXP, 0U, &collisionConfig);
}

static void APP_PxpConfigPsAs(void)
{
    /* PS configure. */
    const pxp_ps_buffer_config_t psBufferConfig = {
        .pixelFormat = APP_PXP_PS_FORMAT,
        .swapByte    = false,
        .bufferAddr  = (uint32_t)s_psBufferPxp,
        .bufferAddrU = 0U,
        .bufferAddrV = 0U,
        .pitchBytes  = APP_PIC_SIZE * APP_BPP,
    };

#if defined(FSL_FEATURE_PXP_V3) && FSL_FEATURE_PXP_V3
    PXP_SetProcessSurfaceBackGroundColor(PXP, 0U, 0U);
#else
    PXP_SetProcessSurfaceBackGroundColor(PXP, 0U);
#endif
    PXP_SetProcessSurfaceBufferConfig(PXP, &psBufferConfig);
    PXP_SetProcessSurfaceScaler(PXP, APP_PIC_SIZE, APP_PIC_SIZE, APP_PIC_SIZE * APP_SCALER, APP_PIC_SIZE * APP_SCALER);
    PXP_SetProcessSurfacePosition(PXP, 0U, 0U, APP_PIC_SIZE * APP_SCALER - 1U, APP_PIC_SIZE * APP_SCALER - 1U);

    /* Disable AS. */
    PXP_SetAlphaSurfacePosition(PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);

    /* Output config. */
    static pxp_output_buffer_config_t outputBufferConfig;
    outputBufferConfig.pixelFormat    = APP_PXP_OUT_FORMAT;
    outputBufferConfig.interlacedMode = kPXP_OutputProgressive;
    outputBufferConfig.buffer0Addr    = (uint32_t)s_outputBuffer;
    outputBufferConfig.buffer1Addr    = 0U;
    outputBufferConfig.pitchBytes     = APP_PIC_SIZE * APP_SCALER;
    outputBufferConfig.width          = APP_PIC_SIZE * APP_SCALER;
    outputBufferConfig.height         = APP_PIC_SIZE * APP_SCALER;

    PXP_SetOutputBufferConfig(PXP, &outputBufferConfig);

    /* Disable CSC1, it is enabled by default. */
    PXP_EnableCsc1(PXP, false);
}

static void APP_PxpConfigureLut(void)
{
    pxp_lut_config_t lutConfig;
    (void)memset(&lutConfig, 0, sizeof(lutConfig));
    lutConfig.lookupMode = kPXP_LutDirectY8;
    lutConfig.outMode    = kPXP_LutOutY8;
    PXP_SetLutConfig(PXP, &lutConfig);

    /* Set the 256 byte of LUT table data. */
    for (uint32_t i = 0U; i < 256U; i++)
    {
        /* Do not change the upper 8-bit for LUT conversion. */
        s_lutData[i] = i;
    }

    PXP_LoadLutTable(PXP, kPXP_LutDirectY8, 256U, (uint32_t) & (s_lutData), 0x0U);

    /* Enable Lut engine */
    PXP_EnableLut(PXP, true);
}

/* Input image of RGB888 on process serface, resize the image and output Y8.
   The process engines include: PS->rotation1->alpha blending0->LUT->output buffer */
static void APP_PxpLegacyFlowProcess(void)
{
    APP_PxpConfigPsAs();
    APP_PxpConfigureLut();

    /* Start PXP process and wait for complete. */
    PXP_ClearStatusFlags(PXP, kPXP_CompleteFlag);
    PXP_Start(PXP);
    /* Wait for process complete. */
    while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(PXP)))
    {
    }
}

static void APP_PxpConfigureDitherFetch(void)
{
    pxp_fetch_engine_config_t fetchConfig;
    (void)memset(&fetchConfig, 0, sizeof(fetchConfig));

    fetchConfig.channelEnable           = true;
    fetchConfig.interface               = kPXP_FetchModeNormal;
    fetchConfig.fetchFormat.burstLength = kPXP_Scanline8bytes;
    fetchConfig.ulcX                    = 0U;
    fetchConfig.ulcY                    = 0U;
    fetchConfig.lrcX                    = APP_PIC_SIZE * APP_SCALER;
    fetchConfig.lrcY                    = APP_PIC_SIZE * APP_SCALER;
    fetchConfig.totalWidth              = APP_PIC_SIZE * APP_SCALER;
    fetchConfig.totalHeight             = APP_PIC_SIZE * APP_SCALER;
    fetchConfig.pitchBytes              = APP_PIC_SIZE * APP_SCALER;
    fetchConfig.activeBits              = kPXP_Active8Bits;
    fetchConfig.pixelFormat             = kPXP_FetchFormatRGB565;
    fetchConfig.shiftConfig.shiftBypass = true;
    fetchConfig.inputBaseAddr0          = (uint32_t)s_outputBuffer;
    fetchConfig.inputBaseAddr1          = 0U;

    /* Only need to configure the channel 0. Must not configure unused channel. */
    PXP_SetFetchEngineConfig(PXP, kPXP_FetchDither, 0U, &fetchConfig);
}

static void APP_PxpConfigureDitherStore()
{
    pxp_store_engine_config_t storeConfig;
    (void)memset(&storeConfig, 0, sizeof(storeConfig));

    storeConfig.channelEnable           = true;
    storeConfig.interface               = kPXP_StoreModeHandshake;
    storeConfig.storeFormat.burstLength = kPXP_Scanline8bytes;
    storeConfig.totalHeight             = APP_PIC_SIZE * APP_SCALER;
    storeConfig.totalWidth              = APP_PIC_SIZE * APP_SCALER;
    storeConfig.pitchBytes              = APP_PIC_SIZE * APP_SCALER;
    storeConfig.activeBits              = kPXP_Active8Bits;
    storeConfig.shiftConfig.shiftBypass = true;
    storeConfig.outputBaseAddr0         = (uint32_t)s_ditherOutputBuffer;
    storeConfig.outputBaseAddr1         = 0U;

    /* Only need to configure the channel 0. Must not configure unused channel */
    PXP_SetStoreEngineConfig(PXP, kPXP_StoreDither, 0U, &storeConfig);
}

static void APP_PxpDitherProcessFloydSteinberg(void)
{
    APP_PxpConfigureDitherFetch();
    APP_PxpConfigureDitherStore();

    const pxp_dither_config_t ditherConfig = {.enableDither0  = 1,
                                              .enableDither1  = 0,
                                              .enableDither2  = 0,
                                              .ditherMode0    = kPXP_DitherFloydSteinberg,
                                              .ditherMode1    = kPXP_DitherPassThrough,
                                              .ditherMode2    = kPXP_DitherPassThrough,
                                              .quantBitNum    = 4,
                                              .lutMode        = kPXP_DitherLutOff,
                                              .idxMatrixSize0 = kPXP_DitherMatrix8,
                                              .idxMatrixSize1 = kPXP_DitherMatrix8,
                                              .idxMatrixSize2 = kPXP_DitherMatrix8,
                                              .enableFinalLut = 0};
    PXP_SetDitherConfig(PXP, &ditherConfig);

    PXP_EnableDither(PXP, true);
}

static void APP_PxpWfeaProcessPic(uint16_t offsetX, uint16_t offsetY, uint8_t updateLut)
{
    pxp_wfea_engine_config_t wfeaConfig;
    (void)memset(&wfeaConfig, 0, sizeof(wfeaConfig));

    wfeaConfig.y4cAddr          = DEMO_Y4C_ADDR;
    wfeaConfig.wbAddr           = DEMO_WB_ADDR;
    wfeaConfig.y4Addr           = (uint32_t)s_ditherOutputBuffer;
    wfeaConfig.updateWidth      = APP_PIC_SIZE * APP_SCALER;
    wfeaConfig.updateHeight     = APP_PIC_SIZE * APP_SCALER;
    wfeaConfig.updatePitch      = APP_PIC_SIZE * APP_SCALER;
    wfeaConfig.ulcX             = offsetX;
    wfeaConfig.ulcY             = offsetY;
    wfeaConfig.resX             = PANEL_WIDTH;
    wfeaConfig.lutNum           = updateLut;
    wfeaConfig.fullUpdateEnable = false; /* Partial update */
    PXP_SetWfeaConfig(PXP, &wfeaConfig);

    /* Clear process completion status flags. */
    PXP_ClearStatusFlags(PXP, kPXP_WfeaStoreCompleteFlag);

    /* Clear histogram engine A&B results before start the process. */
    PXP_ClearHistogramResult(PXP, 0U);
    PXP_ClearHistogramResult(PXP, 1U);

    /* Start WFE-A process and wait for completion. */
    PXP_Start(PXP);
    while ((PXP_GetStatusFlags(PXP) & kPXP_WfeaStoreCompleteFlag) == 0U)
    {
    }
}

static void APP_PxpWfeaDrawDot(uint16_t offsetX, uint16_t offsetY, uint8_t updateLut)
{
    pxp_wfea_engine_config_t wfeaConfig;
    (void)memset(&wfeaConfig, 0, sizeof(wfeaConfig));

    wfeaConfig.y4cAddr          = DEMO_Y4C_ADDR;
    wfeaConfig.wbAddr           = DEMO_WB_ADDR;
    wfeaConfig.y4Addr           = (uint32_t)&blackDot;
    wfeaConfig.updateWidth      = 1U;
    wfeaConfig.updateHeight     = 1U;
    wfeaConfig.updatePitch      = 1U;
    wfeaConfig.ulcX             = offsetX;
    wfeaConfig.ulcY             = offsetY;
    wfeaConfig.resX             = PANEL_WIDTH;
    wfeaConfig.lutNum           = updateLut;
    wfeaConfig.fullUpdateEnable = false; /* Partial update */
    PXP_SetWfeaConfig(PXP, &wfeaConfig);

    /* Clear process completion status flags. */
    PXP_ClearStatusFlags(PXP, kPXP_WfeaStoreCompleteFlag);

    /* Clear histogram engine A&B results before start the process. */
    PXP_ClearHistogramResult(PXP, 0U);
    PXP_ClearHistogramResult(PXP, 1U);

    /* Start WFE-A process and wait for completion. */
    PXP_Start(PXP);
    while ((PXP_GetStatusFlags(PXP) & kPXP_WfeaStoreCompleteFlag) == 0U)
    {
    }
}

static void APP_EpdcUpdateImage(uint16_t offsetX, uint16_t offsetY, uint16_t size, uint8_t updateLut)
{
    /* Get histogram status to decide which waveform mode to use. */
    uint8_t histogramResult = PXP_GetHistogramMatchResult(PXP, 1U);

    if ((histogramResult & kPXP_Histogram2levelMatch) == 0U)
    {
        /* If image is not contained in 2 level pixel state, use waveform mode 2(high quelity image). */
        updateConfig.waveformMode = 2U;
    }
    else
    {
        /* Otherwise(only full black/white update) use waveform mode 1(pen input). */
        updateConfig.waveformMode = 1U;
    }

    updateConfig.lutNum = updateLut;
    updateConfig.width  = size;
    updateConfig.height = size;
    /* If the update width is not 8 byte aligned, stride needs to be configured as 8 byte aligned and larger than
     * width.*/
    updateConfig.stride           = size + (APP_FRAME_BUFFER_ALIGN - (size % APP_FRAME_BUFFER_ALIGN));
    updateConfig.fixedEnable      = false;
    updateConfig.fixedNpEnable    = false;
    updateConfig.npValue          = 0U;
    updateConfig.fixedCpEnable    = false;
    updateConfig.cpValue          = 0U;
    updateConfig.fullUpdateEnable = false;
    updateConfig.coordinateX      = offsetX;
    updateConfig.coordinateY      = offsetY;

    EPDC_ClearStatusFlags(EPDC, kEPDC_FrameCompleteInterruptFlag);

    EPDC_UpdateDisplay(EPDC, &updateConfig);

    while ((EPDC_GetStatusFlags(EPDC) & kEPDC_FrameCompleteInterruptFlag) == 0U)
    {
    }
}

/* LUT complete interrupt handler. */
void DEMO_EPDC_IRQHandler(void)
{
    /* Get the completed LUT(s) and clear the LUT complete interrupt(s). */
    uint64_t completedLut = EPDC_GetLutCompleteStatusFlags(EPDC);
    EPDC_ClearLutCompleteStatusFlags(EPDC, completedLut);

    /* Mark the completed LUT(s) as avaliable in WFE-A */
    PXP_ClearLutUsage(PXP, completedLut);

    SDK_ISR_EXIT_BARRIER;
}

void main()
{
    pxp_histogram_mask_result_t collisionResult;
    uint8_t lutNum;
    uint32_t x, y, i;

    BOARD_BootClockRUN();
    BOARD_InitDebugConsolePins();
    BOARD_InitDebugConsole();

    /* Enable GPIOA clock for panel front light power on. */
    CLOCK_EnableClock(kCLOCK_RgpioA);

    /* PCA6416A I2C. */
    CLOCK_SetIpSrc(kCLOCK_Lpi2c0, kCLOCK_Pcc1BusIpSrcSysOscDiv2);
    RESET_PeripheralReset(kRESET_Lpi2c0);
    BOARD_InitPCA6416A(&g_pca6416aHandle);

    /* EPDC I2C. */
    CLOCK_SetIpSrc(kCLOCK_Lpi2c1, kCLOCK_Pcc1BusIpSrcSysOscDiv2);
    RESET_PeripheralReset(kRESET_Lpi2c1);

    if (BOARD_IsLowPowerBootType() != true) /* not low power boot type */
    {
        BOARD_HandshakeWithUboot();
    }
    else /* low power boot type */
    {
        PRINTF("Pls run the demo with A Core(the demo depend on resource of A Core)\r\n");
        assert(false);
    }

    SDK_DelayAtLeastUs(4000000U,
                       SystemCoreClock); /* wait 4 seconds to make sure reset display components(EPDC, PXP) by mcore */

    /*
     * TODO: Allocate PXP, EPDC to real time domain, otherwise interrupt does not happen.
     * Need to add API in SIM module for this operation.
     */
    *(volatile uint32_t *)0x2802B04C &= ~0x00000020UL;
    *(volatile uint32_t *)0x2802B044 &= ~0x00000080; /* LPAV alloc to RTD */

    BOARD_InitBootPins();

    PRINTF("\r\nEPDC example start...\r\n");

    DEMO_PowerOnPxp();
    DEMO_PowerOnEpdc();
    DEMO_InitEpdcPanel();

    APP_SetupBuffer();
    APP_InitEpdc();

    /* 1. Initialize PXP module. */
    PXP_Init(PXP);

    /* 2. Initialize the panel, clear all EPDC LUT completion status and clear all the LUT occupation status in the
     * WFE-A engine. */
    PRINTF("\r\nInitializing the panel...\r\n");
    APP_PanelInitialization();
    EPDC_ClearLutCompleteStatusFlags(EPDC, 0xFFFFFFFFFFFFFFFFULL);
    PXP_ClearLutUsage(PXP, 0xFFFFFFFFFFFFFFFFULL);

    /* 3. Configure the data process path. */
    APP_PxpConfigurePath();

    /* 4. Use legacy flow to generate the resized image. */
    /* Change the configuration of the engines in legacy flow and repeat this step if user wants to generate different
     * image for each image update. */
    APP_PxpLegacyFlowProcess();
    /* After the Y8 image is generated, disable the legacy flow. */
    PXP_EnableProcessEngine(PXP, kPXP_PsAsOutEngine, false);

    /* 5. Configure dither algorighm. Change dither configuration and repeat this step before WFE-A process if user
     * wants to apply different algorithm for each image update. */
    APP_PxpDitherProcessFloydSteinberg();

    /* 6. Initialize WFE-A engine, enable WFE-A fetch handshake mode. */
    PXP_WfeaInit(PXP, true);

    /* 7. Configure histogram engine a and b for collision detection and histogram process. */
    APP_PxpConfigureHistogram();
    APP_PxpConfigureCollision();

    /* 8. Enable EPDC interrupt. */
    EnableIRQ(EPDC_IRQn);
    /* Enable EPDC LUT1-63 complete interrupt. */
    EPDC_EnableLutCompleteInterrupts(EPDC, 0xFFFFFFFFFFFFFFFEULL);

    /* 9.Use waveform mode 2 to display high quality image. */
    PRINTF("\r\nDisplaying the image to screen 4 times.\r\n");
    PRINTF(
        "\r\nNon-collided update(s) will appear on screen together within one fresh, the collided update(s) will wait "
        "until the previous update(s) to finish then refresh.\r\n");

    i = 4U;
    while (i--)
    {
        /* Get the next available EPDC LUT. */
        while ((EPDC_GetNextAvailableLUT(EPDC, &lutNum) != kStatus_Success) || (lutNum == 0U))
        {
        }

        /* Generate random update coordinates. */
        x = rand() % (PANEL_WIDTH - APP_PIC_SIZE * APP_SCALER);
        y = rand() % (PANEL_HEIGHT - APP_PIC_SIZE * APP_SCALER);

        /* Configure WFE-A, clear status and start the process. */
        APP_PxpWfeaProcessPic(x, y, lutNum);

        /* Get the collision result. If the update size is not the same each time, call PXP_SetHistogramSize to change
         * the histogram size before starting WFE-A process. */
        PXP_GetHistogramMaskResult(PXP, 0U, &collisionResult);

        /* Collision handling. If there is no collided pixel, use EPDC to apply the update to panel directly.
           Otherwise wait for the LUT(s) of the collided update(s) to finish, then re-apply the WFE-A update. */
        if (collisionResult.pixelCount != 0U)
        {
            while ((PXP_GetLutUsage(PXP) & collisionResult.lutlist) != 0U)
            {
            }
            APP_PxpWfeaProcessPic(x, y, lutNum);
        }

        /* Set the LUT as occupied in WFE-A engine */
        PXP_SetLutUsage(PXP, 0x1ULL << lutNum);

        /* Update the image to screen. */
        APP_EpdcUpdateImage(x, y, APP_PIC_SIZE * APP_SCALER, lutNum);
    }

    /* 10. Delay a while then clear the working buffer. */
    SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    for (i = 0; i < (PANEL_WIDTH * PANEL_HEIGHT); i++)
    {
        ((uint16_t *)DEMO_WB_ADDR)[i] = 0x00F0;
    }

    /* 11. Use waveform mode 1 to draw lines to simulate pen input. */
    PRINTF("\r\nDrawing lines to panel...\r\n");

    /* Clear the panel. */
    APP_PanelInitialization();

    /* No need to process pictures, disable dither engine. */
    PXP_EnableDither(PXP, false);

    /* Disable WFE-A handshake with dither. */
    PXP_WfeaEnableDitherHandshake(PXP, false);

    /* Re-configure the histogram size. */
    PXP_SetHistogramSize(PXP, 1U, 1U, 1U);

    i = 0U;
    while (1)
    {
        /* Calculate the dot coordinates. */
        if (((i / PANEL_WIDTH) % 2U) == 0U)
        {
            x = i % PANEL_WIDTH;
        }
        else
        {
            x = PANEL_WIDTH - i % PANEL_WIDTH;
        }
        if (((i / PANEL_HEIGHT) % 2U) == 0U)
        {
            y = i % PANEL_HEIGHT;
        }
        else
        {
            y = PANEL_HEIGHT - i % PANEL_HEIGHT;
        }

        /* Get the next available EPDC LUT */
        while ((EPDC_GetNextAvailableLUT(EPDC, &lutNum) != kStatus_Success) || (lutNum == 0U))
        {
        }

        /* Draw the dot to screen. */
        APP_PxpWfeaDrawDot(x, y, lutNum);

        /* Set the LUT as occupied in WFE-A engine */
        PXP_SetLutUsage(PXP, 0x1ULL << lutNum);

        /* Update the image to screen. */
        APP_EpdcUpdateImage(x, y, 1U, lutNum);

        i++;
    }
}
