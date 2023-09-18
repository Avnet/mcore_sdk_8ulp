/*
 * Octal SPI Norflash IS25LX256 test program
 *
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_flexspi.h"
#include "fsl_debug_console.h"
#include "flash.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t flexspi_nor_enable_4_Byte_Enter(FLEXSPI_Type *base);

/*******************************************************************************
 * Variables
 *****************************************************************************/
extern flexspi_device_config_t deviceconfig;
extern const uint32_t customLUT[CUSTOM_LUT_LENGTH];
/*
 * Device configuration to hold flash settings
 */
flexspi_device_config_t deviceconfig = {
    .flexspiRootClk       = 98000000,
    .flashSize            = FLASH_SIZE,
    .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
    .CSInterval           = 2,
    .CSHoldTime           = 3,
    .CSSetupTime          = 3,
    .dataValidTime        = 0,
    .columnspace          = 0,
    .enableWordAddress    = 0,
    .AWRSeqIndex          = 0,
    .AWRSeqNumber         = 0,
    .ARDSeqIndex          = NOR_CMD_LUT_SEQ_IDX_READ_FAST_OCTAL,
    .ARDSeqNumber         = 1,
    .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
    .AHBWriteWaitInterval = 0,
};
/*
 * CustomLUT to define flash operations
 */
const uint32_t customLUT[CUSTOM_LUT_LENGTH] = {
    /* 4-BYTE READ -SDR 1-1-1 */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x13, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x20),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* 4-BYTE FAST READ - SDR 1-1-1 */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0C, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x20),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x08, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* 4-BYTE OCTAL OUTPUT FAST READ - SDR 1-1-8 */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_OCTAL] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x7C, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x20),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_OCTAL + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_8PAD, 0x08, kFLEXSPI_Command_STOP, kFLEXSPI_8PAD, 0),

    /* 4-BYTE OCTAL I/O FAST READ - SDR 1-8-8 */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_OCTAL_IO] =
         FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xCC, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_8PAD, 0x20),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_OCTAL_IO + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_8PAD, 0x10, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read extend parameters */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xCB, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Write Enable */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Erase Sector-32KB  */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASE_32KB_SECTOR] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x5C, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x20),

    /* 4-BYTE PAGE PROGRAM SDR 1-1-1 */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x12, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x20),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* 4-BYTE OCTAL INPUT FAST PROGRAM SDR  1-1-8 */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_OCTAL_FAST] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x84, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x20),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_OCTAL_FAST + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_8PAD, 0),

    /* 4-BYTE EXTENDED OCTAL INPUT FAST PROGRAM SDR 1-8-8 */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_EXTENDED_OCTAL] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x8E, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_8PAD, 0x20),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_EXTENDED_OCTAL + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_8PAD, 0),

    /* Read ID */
    [4 * NOR_CMD_LUT_SEQ_IDX_READID] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enable OCTAL mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x01, kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04),

    /* Exit QPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_EXITQPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_4PAD, 0xE9, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read status register */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Erase whole chip */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASECHIP] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xC7, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Erase Sector-4KB  */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASE_4KB_SECTOR] =
         FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x21, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x20),

    /* Erase Sector-128KB  */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASE_128KB_SECTOR] =
         FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xDC, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x20),


    //DDR mode
    [4 * NOR_CMD_LUT_SEQ_IDX_OCTAL_DDR_MODE] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x81, kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x00),
    [4 * NOR_CMD_LUT_SEQ_IDX_OCTAL_DDR_MODE+1] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x00),
    [4 * NOR_CMD_LUT_SEQ_IDX_OCTAL_DDR_MODE+2] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x1, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    [4 * NOR_CMD_LUT_SEQ_IDX_ENTER_4BYTE] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x81, kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05),
    [4 * NOR_CMD_LUT_SEQ_IDX_ENTER_4BYTE+1] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x00, kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x00),
    [4 * NOR_CMD_LUT_SEQ_IDX_ENTER_4BYTE+2] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x1, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

};

static uint8_t s_nor_program_buffer[4096];
static uint8_t s_nor_read_buffer[4096];

extern void dump_buf(const char *prompt, char *data, int len);

int flash_test(void)
{
    uint32_t i = 0;
    status_t status;
    uint8_t  device_info[20] = {0};
    uint32_t sector = 0;
    uint32_t subsector, page, phy_addr;

    PRINTF("\r\n");
    PRINTF("+----------------------------------+\r\n");
    PRINTF("| SPI Norflash IS25LX256 Test Case |\r\n");
    PRINTF("+----------------------------------+\r\n");
    PRINTF("\r\n");

    /* 392MHz * 1U / 4U = 98MHz */
    BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, kCLOCK_Pcc0PlatIpSrcPll0Pfd3, 3U, 0U);
    flexspi_nor_flash_init(EXAMPLE_FLEXSPI);

    PRINTF("Octal SPI Norlfash IS25LX256 test                    ");

    /* Get vendor ID. */
    status = flexspi_nor_get_vendor_id(EXAMPLE_FLEXSPI, device_info);
    if (status != kStatus_Success)
    {
        goto cleanup;
    }

    sector = 0;
    status = flexspi_nor_flash_erase_sector(EXAMPLE_FLEXSPI, sector*128*1024, erase128KBSector);
    if( kStatus_Success != status )
    {
        goto cleanup;
    }

    /* 1Device=64Sectors(8MB), 1Sector=32SubSectors(128KB), 1Subsector=16Pages(4KB), 1Pages=256Bytes */

    for(i=0; i<4096; i++)
        s_nor_program_buffer[i] = i&0xFF;

    for(subsector=0; subsector<32; subsector++)
    {
        for (page=0; page<16; page++)
        {
            phy_addr=sector*128*1024+subsector*4*1024+page*FLASH_PAGE_SIZE;
            status = flexspi_nor_flash_program(EXAMPLE_FLEXSPI, phy_addr, (uint32_t *)s_nor_program_buffer, FLASH_PAGE_SIZE, OctalFastProgram);
            if( kStatus_Success != status )
            {
                goto cleanup;
            }
        }
    }


    for(subsector=0; subsector<32; subsector++)
    {
        memset(s_nor_read_buffer, 0, sizeof(s_nor_read_buffer));
        phy_addr=sector*128*1024+subsector*4*1024;

        status = flexspi_nor_flash_read(EXAMPLE_FLEXSPI, phy_addr, (uint32_t *)s_nor_read_buffer, 4096, octalFastRead);
        if( kStatus_Success != status )
            break;

        status = kStatus_Success;
        for(i=0; i<4096; i++)
        {
            if( s_nor_read_buffer[i] != s_nor_program_buffer[i] )
            {
                status = kStatus_Fail;
                goto cleanup;
            }
        }
    }

cleanup:
    PRINTF("%s", kStatus_Success==status ? "\e[32m[  OKAY  ]\e[0m\r\n" : "\e[31m[**FAIL**]\e[0m\r\n");
    return status;
}


/*******************************************************************************
 * Code
 ******************************************************************************/
/*****************************************************************************\
* Function:    flexspi_clock_init
* Input:       void
* Returns:     void
* Description:
*     This function initializes flexSPI clock
\*****************************************************************************/
static inline void flexspi_clock_init(void)
{
}
/*****************************************************************************\
* Function:    flexspi_nor_flash_init
* Input:       FLEXSPI_Type - handle pointer
* Returns:     void
* Description: This function initializes flexSPI clock and configures flash as per deviceconfig
*               Uses customLUT to perform PROGRAM/READ/ERASE operation
*
\*****************************************************************************/
void flexspi_nor_flash_init(FLEXSPI_Type *base)
{
    flexspi_config_t config;

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    bool DCacheEnableFlag = false;
    /* Disable D cache. */
    if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR))
    {
        SCB_DisableDCache();
        DCacheEnableFlag = true;
    }
#endif /* __DCACHE_PRESENT */

    flexspi_clock_init();

    /*Get FLEXSPI default settings and configure the flexspi. */
    FLEXSPI_GetDefaultConfig(&config);

    /*Set AHB buffer size for reading data through AHB bus. */
    config.ahbConfig.enableAHBPrefetch = true;
    config.rxSampleClock               = kFLEXSPI_ReadSampleClkExternalInputFromDqsPad;
#if !(defined(FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN) && FSL_FEATURE_FLEXSPI_HAS_NO_MCR0_COMBINATIONEN)
    config.enableCombination = true;
#endif
    config.ahbConfig.enableAHBBufferable = true;
    config.ahbConfig.enableAHBCachable   = true;
    FLEXSPI_Init(base, &config);

    /* Configure flash settings according to serial flash feature. */
    FLEXSPI_SetFlashConfig(base, &deviceconfig, kFLEXSPI_PortA1);

    /* Update LUT table. */
    FLEXSPI_UpdateLUT(base, 0, customLUT, CUSTOM_LUT_LENGTH);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(base);
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (DCacheEnableFlag)
    {
        /* Enable D cache. */
        SCB_EnableDCache();
    }
#endif /* __DCACHE_PRESENT */
}
/*****************************************************************************\
* Function:    flexspi_nor_write_enable
* Input:       FLEXSPI_Type - handle pointer;
*              baseAddr - Address of flash sector
* Returns:     status_t
* Description: This function writes to WRITE ENABLE register to enable PROGRAM/WRITE operation
*
\*****************************************************************************/
status_t flexspi_nor_write_enable(FLEXSPI_Type *base, uint32_t baseAddr)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Write enable */
    flashXfer.deviceAddress = baseAddr;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Command;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}
/*****************************************************************************\
* Function:    flexspi_nor_wait_bus_busy
* Input:       FLEXSPI_Type - handle pointer;
* Returns:     status_t
* Description: This function checks READSTATUS Register and wait until the current operation is completed
*
\*****************************************************************************/
status_t flexspi_nor_wait_bus_busy(FLEXSPI_Type *base)
{
    /* Wait status ready. */
    bool isBusy;
    uint32_t readValue;
    status_t status;
    flexspi_transfer_t flashXfer;

    flashXfer.deviceAddress = 0;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READSTATUSREG;
    flashXfer.data          = &readValue;
    flashXfer.dataSize      = 1;
    //Continuously poll data from READ STATUS register
    do
    {
        status = FLEXSPI_TransferBlocking(base, &flashXfer);

        if (status != kStatus_Success)
        {
            return status;
        }
        //Check for STATUS POLL polarity
        if (FLASH_BUSY_STATUS_POL)
        {
            if (readValue & (1U << FLASH_BUSY_STATUS_OFFSET))//Check for busy flag
            {
                isBusy = true;
            }
            else
            {
                isBusy = false;
            }
        }
        else
        {
            if (readValue & (1U << FLASH_BUSY_STATUS_OFFSET))//Check for busy flag
            {
                isBusy = false;
            }
            else
            {
                isBusy = true;
            }
        }

    } while (isBusy);

    return status;
}
/*****************************************************************************\
* Function:    flexspi_nor_wait_bus_busy
* Input:       FLEXSPI_Type - handle pointer;
* Returns:     status_t
* Description: This function checks READSTATUS Register and wait until the current operation is completed
*
\*****************************************************************************/
status_t flexspi_nor_clearBP(FLEXSPI_Type *base)
{
    /* Wait status ready. */
    uint32_t writeValue = 0x00;
    status_t status;
    flexspi_transfer_t flashXfer;

    flashXfer.deviceAddress = 0;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG;
    flashXfer.data          = &writeValue;
    flashXfer.dataSize      = 1;
    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}
/*****************************************************************************\
* Function:    flexspi_nor_get_vendor_id
* Input:       FLEXSPI_Type - handle pointer;
*              vendorID - 20 byte data includes information like density, operating voltage etc
* Returns:     status_t
* Description: This function reads Vendor/Manufacturer ID from READ ID Register
*
\*****************************************************************************/
status_t flexspi_nor_get_vendor_id(FLEXSPI_Type *base, uint8_t *device_info)
{
    flexspi_transfer_t flashXfer;
    flashXfer.deviceAddress = 0;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READID;
    flashXfer.data          = (uint32_t*)device_info;
    flashXfer.dataSize      = 20;

    status_t status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}
/*****************************************************************************\
* Function:    flexspi_nor_enable_DDR_mode
* Input:       FLEXSPI_Type - handle pointer;
* Returns:     status_t
* Description: This function enables DDR mode to support upto 200MHz clock
*
\*****************************************************************************/
status_t flexspi_nor_enable_DDR_mode(FLEXSPI_Type *base)
{
    flexspi_transfer_t flashXfer;
    status_t status;
    uint32_t writeValue = FLASH_OCTAL_DDR_ENABLE;

    /* Write enable */
    status = flexspi_nor_write_enable(base, 0);

    if (status != kStatus_Success)
    {
        return status;
    }

    /* Enable octal mode. */
    flashXfer.deviceAddress = 0;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_OCTAL_DDR_MODE;
    flashXfer.data          = &writeValue;
    flashXfer.dataSize      = 1;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_nor_wait_bus_busy(base);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(base);

    return status;
}
/*****************************************************************************\
 * Function:    flexspi_nor_enable_4_Byte_Enter
 * Input:       FLEXSPI_Type - handle pointer;
 * Returns:     status_t
 * Description: This function enables 4-byte address mode of operation
 *
\*****************************************************************************/
__attribute__((unused)) status_t flexspi_nor_enable_4_Byte_Enter(FLEXSPI_Type *base)
{

    flexspi_transfer_t flashXfer;
    status_t status;
    uint32_t writeValue = 0xFE;

    /* Write enable */
    status = flexspi_nor_write_enable(base, 0);

    if (status != kStatus_Success)
    {
        return status;
    }

    /* Enable octal mode. */
    flashXfer.deviceAddress = 0;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ENTER_4BYTE;
    flashXfer.data          = &writeValue;
    flashXfer.dataSize      = 1;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_nor_wait_bus_busy(base);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(base);

    return status;
}
/*****************************************************************************\
 * Function:    flexspi_nor_enable_4_Byte_Exit
* Input:       FLEXSPI_Type - handle pointer;
* Returns:     status_t
* Description: This function disables 4-byte address mode of operation
*
\*****************************************************************************/
status_t flexspi_nor_enable_4_Byte_Exit(FLEXSPI_Type *base)
{
    flexspi_transfer_t flashXfer;
    status_t status;
    uint32_t writeValue = FLASH_OCTAL_DDR_ENABLE;

    /* Write enable */
    status = flexspi_nor_write_enable(base, 0);

    if (status != kStatus_Success)
    {
        return status;
    }

    /* Exit 4Byte address mode. */
    flashXfer.deviceAddress = 0;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_EXITQPI;
    flashXfer.data          = &writeValue;
    flashXfer.dataSize      = 1;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_nor_wait_bus_busy(base);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(base);

    return status;
}
/*****************************************************************************\
* Function:    flexspi_nor_flash_erase_sector
* Input:       FLEXSPI_Type - handle pointer;
*              address - Starting address of flash sector;
*              type - Sector type includes 4KB,32KB and 128KB;
* Returns:     status_t
* Description: This function performs sector-wise flash erase
*
\*****************************************************************************/
status_t flexspi_nor_flash_erase_sector(FLEXSPI_Type *base, uint32_t address, uint8_t type)
{
    status_t status;
    flexspi_transfer_t flashXfer;

    /* Write enable */
    flashXfer.deviceAddress = address;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Command;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    if (status != kStatus_Success)
    {
        return status;
    }

    /* Select appropriate type to initiate the erase Sequence*/
        switch (type) {
        case erase4KBSector:
            flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ERASE_4KB_SECTOR;
            break;
        case erase32KBSector:
            flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ERASE_32KB_SECTOR;
            break;
        case erase128KBSector:
            flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ERASE_128KB_SECTOR;
            break;
        default:
                break;
        }
    flashXfer.deviceAddress = address;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Command;
    flashXfer.SeqNumber     = 1;
    status                  = FLEXSPI_TransferBlocking(base, &flashXfer);

    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_nor_wait_bus_busy(base);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(base);

    return status;
}
/*****************************************************************************\
* Function:    flexspi_nor_erase_chip
* Input:       FLEXSPI_Type - handle pointer;
* Returns:     status_t
* Description: This function performs full chip erase
*
\*****************************************************************************/
status_t flexspi_nor_erase_chip(FLEXSPI_Type *base)
{
    status_t status;
    flexspi_transfer_t flashXfer;
uint32_t address=0;

//    /* Write enable */
    flashXfer.deviceAddress = address;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Command;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    if (status != kStatus_Success)
    {
        return status;
    }
    //Convert flash size into KB
    while(address < (FLASH_SIZE/0x1000)){
        //      status = flexspi_nor_flash_erase_sector(base,address,erase128KBSector);


        flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ERASE_128KB_SECTOR;
        flashXfer.deviceAddress = address;
        flashXfer.port          = kFLEXSPI_PortA1;
        flashXfer.cmdType       = kFLEXSPI_Command;
        flashXfer.SeqNumber     = 1;
        status                  = FLEXSPI_TransferBlocking(base, &flashXfer);

        if (status != kStatus_Success)
        {
            return status;
        }

        status = flexspi_nor_wait_bus_busy(base);

        /* Do software reset. */
        FLEXSPI_SoftwareReset(base);
        address++;
    }
    return status;
}
/*****************************************************************************\
* Function:    flexspi_nor_flash_program
* Input:       FLEXSPI_Type - handle pointer;
*              dstAddr - Flash memory address;
*              src - Pointer to data set to be written;
*              dataSize - Size of data buffer;
* Returns:     status_t
* Description: This function is used to PROGRAM/WRITE data into flash
*
\*****************************************************************************/
status_t flexspi_nor_flash_program(FLEXSPI_Type *base, uint32_t dstAddr, uint32_t *src,size_t dataSize,uint8_t type)
{
    status_t status;
    flexspi_transfer_t flashXfer;

    /* Write enable */
    status = flexspi_nor_write_enable(base, dstAddr);

    if (status != kStatus_Success)
    {
        return status;
    }
    /* Select appropriate type to initiate the Octal Sequence*/
    switch (type) {
    case pageProgram:
        flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE;
        break;
    case OctalFastProgram:
        flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_OCTAL_FAST;
        break;
    case ExtendedOctalFastProgram:
        flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_EXTENDED_OCTAL;
        break;
    default:
            break;
    }
    /* Prepare program command */
    flashXfer.deviceAddress = dstAddr;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.data          = (uint32_t *)src;
    flashXfer.dataSize      = dataSize;
    status                  = FLEXSPI_TransferBlocking(base, &flashXfer);

    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_nor_wait_bus_busy(base);

    /* Do software reset or clear AHB buffer directly. */
#if defined(FSL_FEATURE_SOC_OTFAD_COUNT) && defined(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK) && \
    defined(FLEXSPI_AHBCR_CLRAHBTXBUF_MASK)
    base->AHBCR |= FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK;
    base->AHBCR &= ~(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK);
#else
    FLEXSPI_SoftwareReset(base);
#endif

    return status;
}
/*****************************************************************************\
* Function:    flexspi_nor_flash_read
* Input:       FLEXSPI_Type - handle pointer;
*              dstAddr - Flash memory address;
*              src - Data pointer read from flash memory;
*              dataSize - Size of data buffer;
* Returns:     status_t
* Description: This function is used to READ data from flash
*
\*****************************************************************************/
status_t flexspi_nor_flash_read(FLEXSPI_Type *base, uint32_t dstAddr, uint32_t *src,size_t dataSize, uint8_t type)
{
    status_t status;
    flexspi_transfer_t flashXfer;

    /* Select appropriate type to initiate the READ Sequence*/
    switch (type) {
    case flashRead:
        flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READ_NORMAL;
        break;
    case fastRead:
        flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READ_FAST;
        break;
    case octalFastRead:
        flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READ_FAST_OCTAL;
        break;
    case octalIOFastRead:
        flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READ_FAST_OCTAL_IO;
        break;
    default:
        break;
        }

    /* Prepare READ command */
    flashXfer.deviceAddress = dstAddr;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.data          = (uint32_t *)src;
    flashXfer.dataSize      = dataSize;
    status                  = FLEXSPI_TransferBlocking(base, &flashXfer);

    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_nor_wait_bus_busy(base);

    /* Do software reset or clear AHB buffer directly. */
#if defined(FSL_FEATURE_SOC_OTFAD_COUNT) && defined(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK) && \
    defined(FLEXSPI_AHBCR_CLRAHBTXBUF_MASK)
    base->AHBCR |= FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK;
    base->AHBCR &= ~(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK);
#else
    FLEXSPI_SoftwareReset(base);
#endif

    return status;
}
/*****************************************************************************\
* Function:    flexspi_nor_get_sfdp
* Input:       FLEXSPI_Type - handle pointer;
*              vendorId - Data byte with set of SFDP commands;
* Returns:     status_t
* Description: This function is used to READ SFDP commands from flash SFDP register
*
\*****************************************************************************/
status_t flexspi_nor_get_sfdp(FLEXSPI_Type *base, uint8_t *vendorId)
{
    uint32_t temp;
    flexspi_transfer_t flashXfer;
    flashXfer.deviceAddress = 0;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = 0x5A;
    flashXfer.data          = &temp;
    flashXfer.dataSize      = 1;

    status_t status = FLEXSPI_TransferBlocking(base, &flashXfer);

    *vendorId = temp;

    /* Do software reset or clear AHB buffer directly. */
#if defined(FSL_FEATURE_SOC_OTFAD_COUNT) && defined(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK) && \
    defined(FLEXSPI_AHBCR_CLRAHBTXBUF_MASK)
    base->AHBCR |= FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK;
    base->AHBCR &= ~(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK);
#else
    FLEXSPI_SoftwareReset(base);
#endif

    return status;
}
/*****************************************************************************\
* Function:    flexspi_clear_buffer
* Input:       FLEXSPI_Type - handle pointer;
* Returns:     void
* Description: This function vlears the flexSPI buffer
*
\*****************************************************************************/
void flexspi_clear_buffer(FLEXSPI_Type *base)
{
    /* Do software reset to clear flexspi buffer */
    FLEXSPI_SoftwareReset(base);
}
