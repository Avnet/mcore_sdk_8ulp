/*
 * PSRAM IS66WVO8M8DALL test program
 *
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "fsl_flexspi.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_common.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_FLEXSPI                    BOARD_FLEXSPI_PSRAM
#define EXAMPLE_FLEXSPI_AMBA_BASE          FlexSPI1_AMBA_BASE
#define HYPERRAM_CMD_LUT_SEQ_IDX_READDATA  0
#define HYPERRAM_CMD_LUT_SEQ_IDX_WRITEDATA 1
#define HYPERRAM_CMD_LUT_SEQ_IDX_READREG   2
#define HYPERRAM_CMD_LUT_SEQ_IDX_WRITEREG  3
#define HYPERRAM_CMD_LUT_SEQ_IDX_RESET     4
#define DRAM_SIZE                          0x800000U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t s_hyper_ram_write_buffer[1024];
static uint8_t s_hyper_ram_read_buffer[1024];

/*******************************************************************************
 * Code
 ******************************************************************************/


status_t flexspi_hyper_ram_ipcommand_write_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Write data */
    flashXfer.deviceAddress = address;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = HYPERRAM_CMD_LUT_SEQ_IDX_WRITEDATA;
    flashXfer.data          = buffer;
    flashXfer.dataSize      = length;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}

void flexspi_hyper_ram_ahbcommand_write_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length)
{
    uint32_t *startAddr = (uint32_t *)(EXAMPLE_FLEXSPI_AMBA_BASE + address);
    memcpy(startAddr, buffer, length);
}

status_t flexspi_hyper_ram_ipcommand_read_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Write data */
    flashXfer.deviceAddress = address;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = HYPERRAM_CMD_LUT_SEQ_IDX_READDATA;
    flashXfer.data          = buffer;
    flashXfer.dataSize      = length;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}

void flexspi_hyper_ram_ahbcommand_read_data(FLEXSPI_Type *base, uint32_t address, uint32_t *buffer, uint32_t length)
{
    uint32_t *startAddr = (uint32_t *)(EXAMPLE_FLEXSPI_AMBA_BASE + address);
    memcpy(buffer, startAddr, length);
}

static void FLEXSPI_OctalRAMReadWrite8Bit(void);
static void FLEXSPI_OctalRAMReadWrite16Bit(void);
static void FLEXSPI_OctalRAMReadWrite32Bit(void);
static void ipcommand_write_ahbcommand_read(void);

int psram_test(void)
{
    uint32_t i = 0;
    status_t status;

    PRINTF("\r\n");
    PRINTF("+----------------------------------+\r\n");
    PRINTF("|  PSRAM IS66WVO8M8DALL Test Case  |\r\n");
    PRINTF("+----------------------------------+\r\n");
    PRINTF("\r\n");


    PRINTF("Octal SPI PSRAM IS66WVO8M8DALL test                  ");
    status = BOARD_InitPsRam();
    if (status != kStatus_Success)
    {
        goto cleanup;
    }

#if 0
    ipcommand_write_ahbcommand_read();

    FLEXSPI_OctalRAMReadWrite8Bit();
    FLEXSPI_OctalRAMReadWrite16Bit();
    FLEXSPI_OctalRAMReadWrite32Bit();

    for (i = 0; i < sizeof(s_hyper_ram_write_buffer); i++)
    {
        s_hyper_ram_write_buffer[i] = i;
    }

    /* IP command write/read, should notice that the start address should be even address and the write address/size
     * should be 1024 aligned.*/
    for (i = 0; i < DRAM_SIZE; i += 1024)
    {
        flexspi_hyper_ram_ipcommand_write_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_write_buffer,
                                               sizeof(s_hyper_ram_write_buffer));
        flexspi_hyper_ram_ipcommand_read_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_read_buffer,
                                              sizeof(s_hyper_ram_read_buffer));

        if (memcmp(s_hyper_ram_read_buffer, s_hyper_ram_write_buffer, sizeof(s_hyper_ram_write_buffer)) != 0)
        {
            status = kStatus_Fail;
            goto cleanup;
        }
    }
#endif

    /* Need to reset FlexSPI controller between IP/AHB access. */
    FLEXSPI_SoftwareReset(EXAMPLE_FLEXSPI);

    for (i = 0; i < sizeof(s_hyper_ram_write_buffer); i++)
    {
        s_hyper_ram_write_buffer[i] = (i + 0xFFU);
    }

    /* AHB command write/read datas on all address */
    memset(s_hyper_ram_read_buffer, 0, sizeof(s_hyper_ram_read_buffer));

    for (i = 0; i < DRAM_SIZE; i += 1024)
    {
        flexspi_hyper_ram_ahbcommand_write_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_write_buffer,
                                                sizeof(s_hyper_ram_write_buffer));
        flexspi_hyper_ram_ahbcommand_read_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_read_buffer,
                                               sizeof(s_hyper_ram_write_buffer));

        if (memcmp(s_hyper_ram_read_buffer, s_hyper_ram_write_buffer, sizeof(s_hyper_ram_write_buffer)) != 0)
        {
            status = kStatus_Fail;
            goto cleanup;
        }
    }

    for (i = 0; i < sizeof(s_hyper_ram_write_buffer); i++)
    {
        s_hyper_ram_write_buffer[i] = i;
    }
    memset(s_hyper_ram_read_buffer, 0, sizeof(s_hyper_ram_read_buffer));

    for (i = 1; i < DRAM_SIZE - 1024; i += 1024)
    {
        flexspi_hyper_ram_ahbcommand_write_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_write_buffer,
                                                sizeof(s_hyper_ram_write_buffer));
        flexspi_hyper_ram_ahbcommand_read_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_read_buffer,
                                               sizeof(s_hyper_ram_read_buffer));

        if (memcmp(s_hyper_ram_read_buffer, s_hyper_ram_write_buffer, sizeof(s_hyper_ram_write_buffer)) != 0)
        {
            status = kStatus_Fail;
            goto cleanup;
        }
    }

    for (i = 0; i < sizeof(s_hyper_ram_write_buffer); i++)
    {
        s_hyper_ram_write_buffer[i] = (i + 0xFFU);
    }
    memset(s_hyper_ram_read_buffer, 0, sizeof(s_hyper_ram_read_buffer));

    for (i = 2; i < DRAM_SIZE - 1024; i += 1024)
    {
        flexspi_hyper_ram_ahbcommand_write_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_write_buffer,
                                                sizeof(s_hyper_ram_write_buffer));
        flexspi_hyper_ram_ahbcommand_read_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_read_buffer,
                                               sizeof(s_hyper_ram_read_buffer));

        if (memcmp(s_hyper_ram_read_buffer, s_hyper_ram_write_buffer, sizeof(s_hyper_ram_write_buffer)) != 0)
        {
            status = kStatus_Fail;
            goto cleanup;
        }
    }

    for (i = 0; i < sizeof(s_hyper_ram_write_buffer); i++)
    {
        s_hyper_ram_write_buffer[i] = i;
    }
    memset(s_hyper_ram_read_buffer, 0, sizeof(s_hyper_ram_read_buffer));

    for (i = 3; i < DRAM_SIZE - 1024; i += 1024)
    {
        flexspi_hyper_ram_ahbcommand_write_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_write_buffer,
                                                sizeof(s_hyper_ram_write_buffer));
        flexspi_hyper_ram_ahbcommand_read_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_read_buffer,
                                               sizeof(s_hyper_ram_read_buffer));

        if (memcmp(s_hyper_ram_read_buffer, s_hyper_ram_write_buffer, sizeof(s_hyper_ram_write_buffer)) != 0)
        {
            status = kStatus_Fail;
            goto cleanup;
        }
    }

cleanup:
    PRINTF("%s", kStatus_Success==status ? "\e[32m[  OKAY  ]\e[0m\r\n" : "\e[31m[**FAIL**]\e[0m\r\n");
    return status;
}

#define LINELEN 81
#define CHARS_PER_LINE 16
static char *print_char =
    "                "
    "                "
    " !\"#$%&'()*+,-./"
    "0123456789:;<=>?"
    "@ABCDEFGHIJKLMNO"
    "PQRSTUVWXYZ[\\]^_"
    "`abcdefghijklmno"
    "pqrstuvwxyz{|}~ "
    "                "
    "                "
    " ???????????????"
    "????????????????"
    "????????????????"
    "????????????????"
    "????????????????"
    "????????????????";

void dump_buf(const char *prompt, char *data, int len)
{
    int rc;
    int idx;
    char prn[LINELEN];
    char lit[CHARS_PER_LINE + 1];
    char hc[4];
    short line_done = 1;

    if( prompt )
    {
        PRINTF("%s\r\n", prompt);
    }

    rc = len;
    idx = 0;
    lit[CHARS_PER_LINE] = '\0';
    while (rc > 0)
    {
        if (line_done)
            snprintf(prn, LINELEN, "%08X: ", idx);
        do
        {
            unsigned char c = data[idx];
            snprintf(hc, 4, "%02X ", c);
            strncat(prn, hc, 4);
            lit[idx % CHARS_PER_LINE] = print_char[c];
            ++idx;
        } while (--rc > 0 && (idx % CHARS_PER_LINE != 0));
        line_done = (idx % CHARS_PER_LINE) == 0;
        if (line_done)
            PRINTF("%s  %s\r\n", prn, lit);
        else if (rc == 0)
            strncat(prn, "   ", LINELEN);
    }
    if (!line_done)
    {
        lit[(idx % CHARS_PER_LINE)] = '\0';
        while ((++idx % CHARS_PER_LINE) != 0)
            strncat(prn, "   ", LINELEN);

        PRINTF("%s  %s\r\n", prn, lit);
    }
}

#define DATA_LENGTH          128

__attribute__((unused)) static void FLEXSPI_OctalRAMReadWrite8Bit(void)
{
    uint32_t     i, failed;
    uint8_t *psram   = (uint8_t *)EXAMPLE_FLEXSPI_AMBA_BASE; /* PSRAM start address. */

    PRINTF("\r\nStart %s() test case\r\n", __func__);

    /* +----------------------------------------+
     * |  PSRAM full size read/write test case  |
     * +----------------------------------------+*/

    PRINTF("PSRAM  8Bit read/write addr@%x lenght@%x   ", psram, DRAM_SIZE);

    /* prepare data and write to PSRAM. */
    for(i=0; i<DRAM_SIZE; i++)
    {
        psram[i] = i&0xFF;
    }

    /* read and compare the PSRAM data */
    failed = 0;
    for(i=0; i<DRAM_SIZE; i++)
    {
        if (psram[i] != (i&0xFF))
        {
            failed = 1;
            goto cleanup;
        }
    }

    /* +----------------------------------------+
     * |  PSRAM overloop read/write test case   |
     * +----------------------------------------+*/

    /* clear all the PSRAM data as 0xAA */
    memset(s_hyper_ram_write_buffer, 0xAA, sizeof(s_hyper_ram_write_buffer));
    for(i=0; i<DRAM_SIZE; i+=sizeof(s_hyper_ram_write_buffer))
    {
        memcpy(psram+i, s_hyper_ram_write_buffer, sizeof(s_hyper_ram_write_buffer));
    }

    /* modify data and write to PSRAM. */
    for(i=0; i<DATA_LENGTH; i++)
    {
        s_hyper_ram_write_buffer[i]=i;
    }
    memcpy(psram, s_hyper_ram_write_buffer, DATA_LENGTH);


    /* read and compare the PSRAM data */
    failed = 0;
    for(i=DATA_LENGTH; i<DRAM_SIZE; i+=DATA_LENGTH)
    {
        /* all the left space data should be 0xAA */
        memset(s_hyper_ram_read_buffer, 0, sizeof(s_hyper_ram_read_buffer));
        memcpy(s_hyper_ram_read_buffer, psram+i, DATA_LENGTH);
        if( !memcmp(s_hyper_ram_read_buffer, s_hyper_ram_write_buffer, DATA_LENGTH))
        {
            failed = 1;
            break;
        }
    }

    if( failed )
    {
        PRINTF("PSRAM check failed on addr@%d size@%d:\r\n", i, DATA_LENGTH);
        dump_buf(NULL, (char *)s_hyper_ram_read_buffer, DATA_LENGTH);
    }

cleanup:
    PRINTF("[ %s ]\r\n", failed?"FAIL":" OK ");
    return ;
}


__attribute__((unused)) static void FLEXSPI_OctalRAMReadWrite16Bit(void)
{
    uint32_t     i, j, failed;
    uint32_t     len = (DRAM_SIZE-2)/2;
    uint16_t     *psram;


    PRINTF("\r\nStart %s() test case\r\n", __func__);

    /* +----------------------------------------+
     * |  PSRAM full size read/write test case  |
     * +----------------------------------------+*/

    for(j=0; j<2; j++)
    {
        psram = (uint16_t *)(EXAMPLE_FLEXSPI_AMBA_BASE + j); /* PSRAM start address. */

        PRINTF("PSRAM 16Bit read/write addr@%x lenght@%x   ", psram, len);

        /* prepare data and write to PSRAM. */
        for(i=0; i<len; i++)
        {
            psram[i] = i&0xFFFF;
        }

        /* read and compare the PSRAM data */
        failed = 0;
        for(i=0; i<len; i++)
        {
            if (psram[i] != (i&0xFFFF))
            {
                failed = 1;
                break;
            }
        }
        PRINTF("[ %s ]\r\n", failed?"FAIL":" OK ");
    }

    return ;
}

__attribute__((unused)) static void FLEXSPI_OctalRAMReadWrite32Bit(void)
{
    uint32_t     i, j, failed;
    uint32_t     len = (DRAM_SIZE-4)/4;
    uint32_t     *psram;


    PRINTF("\r\nStart %s() test case\r\n", __func__);

    /* +----------------------------------------+
     * |  PSRAM full size read/write test case  |
     * +----------------------------------------+*/

    for(j=0; j<4; j++)
    {
        psram = (uint32_t *)(EXAMPLE_FLEXSPI_AMBA_BASE + j); /* PSRAM start address. */

        PRINTF("PSRAM 32Bit read/write addr@%x lenght@%x   ", psram, len);

        /* prepare data and write to PSRAM. */
        for(i=0; i<len; i++)
        {
            psram[i] = (uint32_t)&psram[i];
        }

        /* read and compare the PSRAM data */
        failed = 0;
        for(i=0; i<len; i++)
        {
            if (psram[i] != (uint32_t)&psram[i])
            {
                failed = 1;
                break;
            }
        }
        PRINTF("[ %s ]\r\n", failed?"FAIL":" OK ");
    }

    return ;
}

__attribute__((unused)) static void ipcommand_write_ahbcommand_read(void)
{
    uint32_t      i;
    int           failed = 0;

    PRINTF("PSRAM IP command write and AHB command read test     ");

    /* +----------------------------------------+
     * |   IP command erase and write 0~128K    |
     * +----------------------------------------+*/

    /* memset() all the PSRAM as 0xAA */
    //PRINTF("PSRAM IP command clear all datas to be 0xAA.\r\n");
    memset(s_hyper_ram_write_buffer, 0xAA, sizeof(s_hyper_ram_write_buffer));
    for (i = 0; i < DRAM_SIZE; i += 1024)
    {
        flexspi_hyper_ram_ipcommand_write_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_write_buffer, sizeof(s_hyper_ram_write_buffer));
    }

    /* IP command write $DATA_LENGTH data */
    for(i=0; i<DATA_LENGTH; i++)
    {
        s_hyper_ram_write_buffer[i]=i;
    }
    //dump_buf("PSRAM IP command write Buffer addr@0x0 size@128:", (char *)s_hyper_ram_write_buffer, DATA_LENGTH);
    flexspi_hyper_ram_ipcommand_write_data(EXAMPLE_FLEXSPI, 0x0, (uint32_t *)s_hyper_ram_write_buffer, DATA_LENGTH);


    /* +----------------------------------------+
     * |   AHB command read 0~256K data PSRAM   |
     * +----------------------------------------+*/

    /* Need to reset FlexSPI controller between IP/AHB access. */
    FLEXSPI_SoftwareReset(EXAMPLE_FLEXSPI);

    memset(s_hyper_ram_read_buffer, 0x00, sizeof(s_hyper_ram_read_buffer));
    flexspi_hyper_ram_ahbcommand_read_data(EXAMPLE_FLEXSPI, 0x0, (uint32_t *)s_hyper_ram_read_buffer, sizeof(s_hyper_ram_read_buffer));
    //PRINTF("PSRAM AHB command read Buffer addr@0x0 size@%d:\r\n", DATA_LENGTH*2);
    //dump_buf(NULL, (char *)s_hyper_ram_read_buffer, DATA_LENGTH*2);

    if( memcmp(s_hyper_ram_read_buffer, s_hyper_ram_write_buffer, DATA_LENGTH))
    {
        PRINTF("failed    [ addr@0x0 ]\r\n");
        return ;
    }


    /* +----------------------------------------+
     * |  AHB command overloop read test case   |
     * +----------------------------------------+*/

    /* read and compare the PSRAM data */
    for(i=DATA_LENGTH; i<DRAM_SIZE; i+=DATA_LENGTH)
    {
        /* all the left space data should be 0xAA */
        memset(s_hyper_ram_read_buffer, 0, sizeof(s_hyper_ram_read_buffer));
        flexspi_hyper_ram_ahbcommand_read_data(EXAMPLE_FLEXSPI, i, (uint32_t *)s_hyper_ram_read_buffer, DATA_LENGTH);
        if( !memcmp(s_hyper_ram_read_buffer, s_hyper_ram_write_buffer, DATA_LENGTH))
        {
            failed = 1;
            break;
        }
    }

    if( failed )
    {
        PRINTF("failed    [ addr@0x0 ]\r\n", i);
        PRINTF("PSRAM IP command write and AHB command read test failed    [ addr@0x%x ]\r\n", i);
        dump_buf(NULL, (char *)s_hyper_ram_read_buffer, DATA_LENGTH);
    }

    PRINTF("[  OK  ]\r\n");
    return ;
}
