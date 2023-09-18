/*
 * Octal SPI Norflash IS25LX256 test program
 *
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef FLEXSPI_NOR_OCTAL_FLASH_H_
#define FLEXSPI_NOR_OCTAL_FLASH_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define EXAMPLE_FLEXSPI           FLEXSPI0
#define FLASH_SIZE                0x2000 /* 64Mb/KByte */
#define EXAMPLE_FLEXSPI_AMBA_BASE FlexSPI0_AMBA_BASE
#define FLASH_PAGE_SIZE           256
#define FLASH_SUBSEC_SIZE         4096    /* 4KB  */
#define SECTOR_SIZE               0x20000 /* 128K */
#define EXAMPLE_FLEXSPI_CLOCK     kCLOCK_Flexspi0

#define NOR_CMD_LUT_SEQ_IDX_READ_NORMAL                  7
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST                    13
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_OCTAL              0
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_OCTAL_IO           16
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS                   1
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE                  2
#define NOR_CMD_LUT_SEQ_IDX_ERASE_32KB_SECTOR            3
#define NOR_CMD_LUT_SEQ_IDX_ERASE_4KB_SECTOR             14
#define NOR_CMD_LUT_SEQ_IDX_ERASE_128KB_SECTOR           15
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE           6
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_OCTAL_FAST       4
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_EXTENDED_OCTAL   16
#define NOR_CMD_LUT_SEQ_IDX_READID                       8
#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG               9
#define NOR_CMD_LUT_SEQ_IDX_ENTER_4BYTE                  10
#define NOR_CMD_LUT_SEQ_IDX_EXITQPI                      11
#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG                12
#define NOR_CMD_LUT_SEQ_IDX_ERASECHIP                    5
#define NOR_CMD_LUT_SEQ_IDX_OCTAL_DDR_MODE               17

#define CUSTOM_LUT_LENGTH                                80
#define FLASH_OCTAL_DDR_ENABLE                           0xE7
#define FLASH_BUSY_STATUS_POL                            1
#define FLASH_BUSY_STATUS_OFFSET                         0

/*
 * Various Flash operation
 * - Program
 * - Read
 * - Erase
 */
enum flashOperation{
    pageProgram,
    OctalFastProgram,
    ExtendedOctalFastProgram,
    flashRead,
    fastRead,
    octalFastRead,
    octalIOFastRead,
    erase4KBSector,
    erase32KBSector,
    erase128KBSector
};

void flexspi_nor_flash_init(FLEXSPI_Type *base);
status_t flexspi_nor_get_vendor_id(FLEXSPI_Type *base, uint8_t *device_info);
void flexspi_clear_buffer(FLEXSPI_Type *base);
status_t flexspi_nor_flash_erase_sector(FLEXSPI_Type *base, uint32_t address, uint8_t type);
status_t flexspi_nor_erase_chip(FLEXSPI_Type *base);
status_t flexspi_nor_flash_program(FLEXSPI_Type *base, uint32_t dstAddr, uint32_t *src,size_t dataSize,uint8_t type);
status_t flexspi_nor_flash_read(FLEXSPI_Type *base, uint32_t dstAddr, uint32_t *src,size_t dataSize, uint8_t type);

#endif /* FLEXSPI_NOR_OCTAL_FLASH_H_ */
