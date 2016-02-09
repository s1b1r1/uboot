/*
 * Common definitions for LPC32XX board configurations
 *
 * Copyright (C) 2011-2015 Vladimir Zapolskiy <vz@mleia.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _LPC32XX_CONFIG_H
#define _LPC32XX_CONFIG_H

#define CONFIG_SYS_GENERIC_BOARD

/* Basic CPU architecture */
#define CONFIG_ARCH_CPU_INIT

#define CONFIG_NR_DRAM_BANKS_MAX	2

/* UART configuration */
#if (CONFIG_SYS_LPC32XX_UART >= 3) && (CONFIG_SYS_LPC32XX_UART <= 6)
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_CONS_INDEX		(CONFIG_SYS_LPC32XX_UART - 2)
#elif	(CONFIG_SYS_LPC32XX_UART == 1) || (CONFIG_SYS_LPC32XX_UART == 2) || \
	(CONFIG_SYS_LPC32XX_UART == 7)
#define CONFIG_LPC32XX_HSUART
#else
#error "define CONFIG_SYS_LPC32XX_UART in the range from 1 to 7"
#endif

#if defined(CONFIG_SYS_NS16550_SERIAL)
#define CONFIG_SYS_NS16550

#define CONFIG_SYS_NS16550_REG_SIZE	-4
#define CONFIG_SYS_NS16550_CLK		get_serial_clock()

#define CONFIG_SYS_NS16550_COM1		UART3_BASE
#define CONFIG_SYS_NS16550_COM2		UART4_BASE
#define CONFIG_SYS_NS16550_COM3		UART5_BASE
#define CONFIG_SYS_NS16550_COM4		UART6_BASE
#endif

#if defined(CONFIG_LPC32XX_HSUART)
#if	CONFIG_SYS_LPC32XX_UART == 1
#define HS_UART_BASE			HS_UART1_BASE
#elif	CONFIG_SYS_LPC32XX_UART == 2
#define HS_UART_BASE			HS_UART2_BASE
#else	/* CONFIG_SYS_LPC32XX_UART == 7 */
#define HS_UART_BASE			HS_UART7_BASE
#endif
#endif

#define CONFIG_SYS_BAUDRATE_TABLE	\
		{ 9600, 19200, 38400, 57600, 115200, 230400, 460800 }

/* Ethernet */
#define LPC32XX_ETH_BASE ETHERNET_BASE

/* NAND */
#if defined(CONFIG_NAND_LPC32XX_SLC)
#define NAND_LARGE_BLOCK_PAGE_SIZE	0x800
#define NAND_SMALL_BLOCK_PAGE_SIZE	0x200

#if !defined(CONFIG_SYS_NAND_PAGE_SIZE)
#define CONFIG_SYS_NAND_PAGE_SIZE	NAND_LARGE_BLOCK_PAGE_SIZE
#endif

#if (CONFIG_SYS_NAND_PAGE_SIZE == NAND_LARGE_BLOCK_PAGE_SIZE)
#define CONFIG_SYS_NAND_OOBSIZE		64
#define CONFIG_SYS_NAND_ECCPOS		{ 40, 41, 42, 43, 44, 45, 46, 47, \
					  48, 49, 50, 51, 52, 53, 54, 55, \
					  56, 57, 58, 59, 60, 61, 62, 63, }
#define CONFIG_SYS_NAND_BAD_BLOCK_POS	NAND_LARGE_BADBLOCK_POS
#elif (CONFIG_SYS_NAND_PAGE_SIZE == NAND_SMALL_BLOCK_PAGE_SIZE)
#define CONFIG_SYS_NAND_OOBSIZE		16
#define CONFIG_SYS_NAND_ECCPOS		{ 10, 11, 12, 13, 14, 15, }
#define CONFIG_SYS_NAND_BAD_BLOCK_POS	0
#else
#error "CONFIG_SYS_NAND_PAGE_SIZE set to an invalid value"
#endif

#define CONFIG_SYS_NAND_ECCSIZE		0x100
#define CONFIG_SYS_NAND_ECCBYTES	3
#define CONFIG_SYS_NAND_PAGE_COUNT	(CONFIG_SYS_NAND_BLOCK_SIZE / \
						CONFIG_SYS_NAND_PAGE_SIZE)
#endif	/* CONFIG_NAND_LPC32XX_SLC */

/* NOR Flash */
#if defined(CONFIG_SYS_FLASH_CFI)
#define CONFIG_FLASH_CFI_DRIVER
#define CONFIG_SYS_FLASH_PROTECTION
#endif

/* USB OHCI */
#if defined(CONFIG_USB_OHCI_LPC32XX)
#define CONFIG_USB_OHCI_NEW
#define CONFIG_SYS_USB_OHCI_CPU_INIT
#define CONFIG_SYS_USB_OHCI_MAX_ROOT_PORTS	1
#define CONFIG_SYS_USB_OHCI_REGS_BASE		USB_BASE
#define CONFIG_SYS_USB_OHCI_SLOT_NAME		"lpc32xx-ohci"
#endif

#endif /* _LPC32XX_CONFIG_H */
