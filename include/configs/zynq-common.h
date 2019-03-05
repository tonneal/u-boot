/*
 * (C) Copyright 2012 Michal Simek <monstr@monstr.eu>
 * (C) Copyright 2013 Xilinx, Inc.
 *
 * Common configuration options for all Zynq boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_ZYNQ_COMMON_H
#define __CONFIG_ZYNQ_COMMON_H

/* CPU clock */
#ifndef CONFIG_CPU_FREQ_HZ
# define CONFIG_CPU_FREQ_HZ	800000000
#endif

/* Cache options */
#define CONFIG_SYS_L2CACHE_OFF
#ifndef CONFIG_SYS_L2CACHE_OFF
# define CONFIG_SYS_L2_PL310
# define CONFIG_SYS_PL310_BASE		0xf8f02000
#endif

#define ZYNQ_SCUTIMER_BASEADDR		0xF8F00600
#define CONFIG_SYS_TIMERBASE		ZYNQ_SCUTIMER_BASEADDR
#define CONFIG_SYS_TIMER_COUNTS_DOWN
#define CONFIG_SYS_TIMER_COUNTER	(CONFIG_SYS_TIMERBASE + 0x4)

/* Serial drivers */
/* The following table includes the supported baudrates */
#define CONFIG_SYS_BAUDRATE_TABLE  \
	{300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400}

#define CONFIG_ARM_DCC

/* Ethernet driver */
#if defined(CONFIG_ZYNQ_GEM)
# define CONFIG_MII
# define CONFIG_SYS_FAULT_ECHO_LINK_DOWN
# define CONFIG_BOOTP_BOOTPATH
# define CONFIG_BOOTP_GATEWAY
# define CONFIG_BOOTP_HOSTNAME
# define CONFIG_BOOTP_MAY_FAIL
#endif

/* SPI */
#ifdef CONFIG_ZYNQ_SPI
#endif

/* QSPI */
#ifdef CONFIG_ZYNQ_QSPI
# define CONFIG_SF_DEFAULT_SPEED	30000000
#endif

/* NOR */
#ifdef CONFIG_MTD_NOR_FLASH
# define CONFIG_SYS_FLASH_BASE		0xE2000000
# define CONFIG_SYS_FLASH_SIZE		(16 * 1024 * 1024)
# define CONFIG_SYS_MAX_FLASH_BANKS	1
# define CONFIG_SYS_MAX_FLASH_SECT	512
# define CONFIG_SYS_FLASH_ERASE_TOUT	1000
# define CONFIG_SYS_FLASH_WRITE_TOUT	5000
# define CONFIG_FLASH_SHOW_PROGRESS	10
# define CONFIG_SYS_FLASH_CFI
# undef CONFIG_SYS_FLASH_EMPTY_INFO
# define CONFIG_FLASH_CFI_DRIVER
# undef CONFIG_SYS_FLASH_PROTECTION
# define CONFIG_SYS_FLASH_USE_BUFFER_WRITE
#endif

#ifdef CONFIG_NAND_ZYNQ
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_MTD_DEVICE
#endif

/* MMC */
#if defined(CONFIG_MMC_SDHCI_ZYNQ)
# define CONFIG_ZYNQ_SDHCI_MAX_FREQ	52000000
#endif

#ifdef CONFIG_USB_EHCI_ZYNQ
# define CONFIG_EHCI_IS_TDI

# define CONFIG_SYS_DFU_DATA_BUF_SIZE	0x600000
# define DFU_DEFAULT_POLL_TIMEOUT	300
# define CONFIG_USB_CABLE_CHECK
# define CONFIG_THOR_RESET_OFF
# define CONFIG_USB_FUNCTION_THOR
# define DFU_ALT_INFO_RAM \
	"dfu_ram_info=" \
	"set dfu_alt_info " \
	"${kernel_image} ram 0x3000000 0x500000\\\\;" \
	"${devicetree_image} ram 0x2A00000 0x20000\\\\;" \
	"${ramdisk_image} ram 0x2000000 0x600000\0" \
	"dfu_ram=run dfu_ram_info && dfu 0 ram 0\0" \
	"thor_ram=run dfu_ram_info && thordown 0 ram 0\0"

# if defined(CONFIG_MMC_SDHCI_ZYNQ)
#  define DFU_ALT_INFO_MMC \
	"dfu_mmc_info=" \
	"set dfu_alt_info " \
	"${kernel_image} fat 0 1\\\\;" \
	"${devicetree_image} fat 0 1\\\\;" \
	"${ramdisk_image} fat 0 1\0" \
	"dfu_mmc=run dfu_mmc_info && dfu 0 mmc 0\0" \
	"thor_mmc=run dfu_mmc_info && thordown 0 mmc 0\0"

#  define DFU_ALT_INFO	\
	DFU_ALT_INFO_RAM \
	DFU_ALT_INFO_MMC
# else
#  define DFU_ALT_INFO	\
	DFU_ALT_INFO_RAM
# endif
#endif

#if !defined(DFU_ALT_INFO)
# define DFU_ALT_INFO
#endif

#if defined(CONFIG_MMC_SDHCI_ZYNQ) || defined(CONFIG_ZYNQ_USB)
# define CONFIG_SUPPORT_VFAT
#endif

#if defined(CONFIG_ZYNQ_I2C0) || defined(CONFIG_ZYNQ_I2C1)
#define CONFIG_SYS_I2C_ZYNQ
#endif

/* I2C */
#if defined(CONFIG_SYS_I2C_ZYNQ)
# define CONFIG_SYS_I2C
# define CONFIG_SYS_I2C_ZYNQ_SPEED		100000
# define CONFIG_SYS_I2C_ZYNQ_SLAVE		0
#endif

/* EEPROM */
#ifdef CONFIG_ZYNQ_EEPROM
# define CONFIG_SYS_I2C_EEPROM_ADDR_LEN		1
# define CONFIG_SYS_I2C_EEPROM_ADDR		0x54
# define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	4
# define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	5
# define CONFIG_SYS_EEPROM_SIZE			1024 /* Bytes */
# define CONFIG_SYS_I2C_MUX_ADDR		0x74
# define CONFIG_SYS_I2C_MUX_EEPROM_SEL		0x4
#endif

/* Total Size of Environment Sector */
#define CONFIG_ENV_SIZE			(128 << 10)

/* Allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE

/* Environment */
#ifndef CONFIG_ENV_IS_NOWHERE
# define CONFIG_ENV_SECT_SIZE		CONFIG_ENV_SIZE

/* cc108 requires to be 0xF00000 to have boot.bin with bitstream included */
# ifndef CONFIG_ENV_OFFSET
#  define CONFIG_ENV_OFFSET		0x101C0000
# endif
#endif

/* enable preboot to be loaded before CONFIG_BOOTDELAY */
#define CONFIG_PREBOOT

#define CONFIG_SYS_LOAD_ADDR		0 /* default? */

/* Distro boot enablement */

#ifdef CONFIG_SPL_BUILD
#define BOOTENV
#else
#include <config_distro_defaults.h>

#ifdef CONFIG_CMD_MMC
#define BOOT_TARGET_DEVICES_MMC(func) func(MMC, mmc, 0)
#else
#define BOOT_TARGET_DEVICES_MMC(func)
#endif

#ifdef CONFIG_CMD_USB
#define BOOT_TARGET_DEVICES_USB(func) func(USB, usb, 0)
#else
#define BOOT_TARGET_DEVICES_USB(func)
#endif

#if defined(CONFIG_CMD_PXE)
#define BOOT_TARGET_DEVICES_PXE(func) func(PXE, pxe, na)
#else
#define BOOT_TARGET_DEVICES_PXE(func)
#endif

#if defined(CONFIG_CMD_DHCP)
#define BOOT_TARGET_DEVICES_DHCP(func) func(DHCP, dhcp, na)
#else
#define BOOT_TARGET_DEVICES_DHCP(func)
#endif

#define BOOT_TARGET_DEVICES(func) \
	BOOT_TARGET_DEVICES_MMC(func) \
	BOOT_TARGET_DEVICES_USB(func) \
	BOOT_TARGET_DEVICES_PXE(func) \
	BOOT_TARGET_DEVICES_DHCP(func)

#include <config_distro_bootcmd.h>
#endif /* CONFIG_SPL_BUILD */

/* Default environment */
#ifndef CONFIG_EXTRA_ENV_SETTINGS
#define CONFIG_EXTRA_ENV_SETTINGS	\
	"blr_fname=BOOT.bin\0" \
	"env_fname=uEnt.txt\0" \
	"dtb_fname=devicetree.dtb\0" \
	"bit_fname=system.bit\0" \
	"knl_fname=uImage\0" \
	"rfs_fname=rootfs.jffs2\0" \
	"opt_fname=opt.jffs2\0" \
	"ufs_fname=rootfs.cpio.uboot\0" \
	"blr_loadaddr=0x10000000\0" \
	"env_loadaddr=0x101C0000\0" \
	"dtb_loadaddr=0x10200000\0" \
	"bit_loadaddr=0x13300000\0" \
	"knl_loadaddr=0x10300000\0" \
	"rfs_loadaddr=0x10D00000\0" \
	"opt_loadaddr=0x12000000\0" \
	"blr_spifaddr=0x00000000\0" \
	"env_spifaddr=0x001C0000\0" \
	"dtb_spifaddr=0x00200000\0" \
	"bit_spifaddr=0x03300000\0" \
	"knl_spifaddr=0x00300000\0" \
	"rfs_spifaddr=0x00D00000\0" \
	"opt_spifaddr=0x02000000\0" \
	"blr_partsize=0x000E0000\0" \
	"env_partsize=0x00020000\0" \
	"dtb_partsize=0x00080000\0" \
	"bit_partsize=0x00D00000\0" \
	"knl_partsize=0x00500000\0" \
	"rfs_partsize=0x01300000\0" \
	"opt_partsize=0x01300000\0" \
	"erase_blr=echo Erase Bootloader......; sf erase ${blr_spifaddr} ${blr_partsize}\0" \
	"erase_dtb=echo Erase Devicetree......; sf erase ${dtb_spifaddr} ${dtb_partsize}\0" \
	"erase_bit=echo Erase Bitstream.......; sf erase ${bit_spifaddr} ${bit_partsize}\0" \
	"erase_knl=echo Erase Kernel..........; sf erase ${knl_spifaddr} ${knl_partsize}\0" \
	"erase_rfs=echo Erase Rootfs..........; sf erase ${rfs_spifaddr} ${rfs_partsize}\0" \
	"flash_blr=echo Flash Bootloader......; mw.b ${blr_loadaddr} 0xFF ${blr_partsize}; fatload mmc 0 ${blr_loadaddr} ${blr_fname}; sf write ${blr_loadaddr} ${blr_spifaddr} ${blr_partsize}\0" \
	"flash_dtb=echo Flash Devicetree......; mw.b ${dtb_loadaddr} 0xFF ${dtb_partsize}; fatload mmc 0 ${dtb_loadaddr} ${dtb_fname}; sf write ${dtb_loadaddr} ${dtb_spifaddr} ${dtb_partsize}\0" \
	"flash_bit=echo Flash Bitstream.......; mw.b ${bit_loadaddr} 0xFF ${bit_partsize}; fatload mmc 0 ${bit_loadaddr} ${bit_fname}; sf write ${bit_loadaddr} ${bit_spifaddr} ${bit_partsize}\0" \
	"flash_knl=echo Flash Kernel..........; mw.b ${knl_loadaddr} 0xFF ${knl_partsize}; fatload mmc 0 ${knl_loadaddr} ${knl_fname}; sf write ${knl_loadaddr} ${knl_spifaddr} ${knl_partsize}\0" \
	"flash_rfs=echo Flash Rootfs..........; mw.b ${rfs_loadaddr} 0xFF ${rfs_partsize}; fatload mmc 0 ${rfs_loadaddr} ${rfs_fname}; sf write ${rfs_loadaddr} ${rfs_spifaddr} ${rfs_partsize}\0" \
	"flash_opt=echo Flash Opt part........; mw.b ${opt_loadaddr} 0xFF ${opt_partsize}; fatload mmc 0 ${opt_loadaddr} ${opt_fname}; sf write ${opt_loadaddr} ${opt_spifaddr} ${opt_partsize}\0" \
	"updatesys=sf probe 0; sf erase 0 0x4000000; run flash_blr; run flash_dtb; run flash_bit; run flash_knl; run flash_rfs; run flash_opt; echo Flash Programming DONE \0" \
	"sf_boot=sf probe 0 ; " \
		"sf read ${bit_loadaddr} ${bit_spifaddr} ${bit_partsize}; " \
		"fpga loadb 0 ${bit_loadaddr} ${bit_partsize}; " \
		"sf read ${knl_loadaddr} ${knl_spifaddr} ${knl_partsize}; " \
		"sf read ${dtb_loadaddr} ${dtb_spifaddr} ${dtb_partsize}; " \
		"run jffs2_args; " \
		"bootm ${knl_loadaddr} - ${dtb_loadaddr}\0 " \
	"jffs2_args=setenv bootargs console=ttyPS0,115200 " \
		"root=/dev/mtdblock8 rootfstype=jffs2 rw rootwait uio_pdrv_genirq.of_id=generic-uio ${optargs} \0" \
	"ram_boot=if mmcinfo; then " \
		"run uenvboot; " \
		"mmc rescan; " \
		"mmc dev 0; " \
		"fatload mmc 0 ${bit_loadaddr} ${bit_fname}; " \
		"fpga loadb 0 ${bit_loadaddr} ${bit_partsize}; " \
		"fatload mmc 0 ${knl_loadaddr} ${knl_fname}; " \
		"fatload mmc 0 ${dtb_loadaddr} ${dtb_fname}; " \
		"fatload mmc 0 ${rfs_loadaddr} ${ufs_fname}; " \
		"run ram_args; " \
		"bootm ${knl_loadaddr} ${rfs_loadaddr} ${dtb_loadaddr}; " \
		"fi\0" \
	"ram_args=setenv bootargs console=ttyPS0,115200 " \
		"root=/dev/ram rw earlyprintk${optargs} \0" \
	"net_boot=\0" \
	"kernel_image=uImage\0"	\
	"kernel_load_address=0x2080000\0" \
	"ramdisk_image=uramdisk.image.gz\0"	\
	"ramdisk_load_address=0x4000000\0"	\
	"devicetree_image=devicetree.dtb\0"	\
	"devicetree_load_address=0x2000000\0"	\
	"bitstream_image=system.bit.bin\0"	\
	"boot_image=BOOT.bin\0"	\
	"loadbit_addr=0x100000\0"	\
	"loadbootenv_addr=0x2000000\0" \
	"kernel_size=0x500000\0"	\
	"devicetree_size=0x20000\0"	\
	"ramdisk_size=0x5E0000\0"	\
	"boot_size=0xF00000\0"	\
	"fdt_high=0x20000000\0"	\
	"initrd_high=0x20000000\0"	\
	"bootenv=uEnv.txt\0" \
	"loadbootenv=load mmc 0 ${loadbootenv_addr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from SD ...; " \
		"env import -t ${loadbootenv_addr} $filesize\0" \
	"sd_uEnvtxt_existence_test=test -e mmc 0 /uEnv.txt\0" \
	"preboot=if test $modeboot = sdboot && env run sd_uEnvtxt_existence_test; " \
			"then if env run loadbootenv; " \
				"then env run importbootenv; " \
			"fi; " \
		"fi; \0" \
	"mmc_loadbit=echo Loading bitstream from SD/MMC/eMMC to RAM.. && " \
		"mmcinfo && " \
		"load mmc 0 ${loadbit_addr} ${bitstream_image} && " \
		"fpga load 0 ${loadbit_addr} ${filesize}\0" \
	"uenvboot=" \
		"if run loadbootenv; then " \
			"echo Loaded environment from ${bootenv}; " \
			"run importbootenv; " \
		"fi; " \
		"if test -n $uenvcmd; then " \
			"echo Running uenvcmd ...; " \
			"run uenvcmd; " \
		"fi\0" \
	"sd_boot=if mmcinfo; then " \
			"echo Copying Linux from SD to RAM... && " \
			"load mmc 0 ${knl_loadaddr} ${knl_fname} && " \
			"load mmc 0 ${dtb_loadaddr} ${dtb_fname} && " \
			"load mmc 0 ${rfs_loadaddr} ${ufs_fname} && " \
			"bootm ${knl_loadaddr} ${rfs_loadaddr} ${dtb_loadaddr}; " \
		"fi\0" \
	"rsa_qspiboot=echo Copying Image from QSPI flash to RAM... && " \
		"sf probe 0 0 0 && " \
		"sf read 0x100000 0x0 ${boot_size} && " \
		"zynqrsa 0x100000 && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
	"rsa_sdboot=echo Copying Image from SD to RAM... && " \
		"load mmc 0 0x100000 ${boot_image} && " \
		"zynqrsa 0x100000 && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
		DFU_ALT_INFO \
		BOOTENV
#endif

/* Miscellaneous configurable options */

#define CONFIG_CMDLINE_EDITING
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_LONGHELP
#define CONFIG_CLOCKS
#define CONFIG_SYS_MAXARGS		32 /* max number of command args */
#define CONFIG_SYS_CBSIZE		2048 /* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

#ifndef CONFIG_NR_DRAM_BANKS
# define CONFIG_NR_DRAM_BANKS		1
#endif

#define CONFIG_SYS_MEMTEST_START	0
#define CONFIG_SYS_MEMTEST_END		0x1000

#define CONFIG_SYS_INIT_RAM_ADDR	0xFFFF0000
#define CONFIG_SYS_INIT_RAM_SIZE	0x2000
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_INIT_RAM_ADDR + \
					CONFIG_SYS_INIT_RAM_SIZE - \
					GENERATED_GBL_DATA_SIZE)

/* Enable the PL to be downloaded */
#define CONFIG_FPGA_ZYNQPL

/* FIT support */
#define CONFIG_IMAGE_FORMAT_LEGACY /* enable also legacy image format */

/* Extend size of kernel image for uncompression */
#define CONFIG_SYS_BOOTM_LEN	(60 * 1024 * 1024)

/* Boot FreeBSD/vxWorks from an ELF image */
#define CONFIG_SYS_MMC_MAX_DEVICE	1

#define CONFIG_SYS_LDSCRIPT  "arch/arm/mach-zynq/u-boot.lds"

#undef CONFIG_BOOTM_NETBSD

#define CONFIG_SYS_HZ			1000

/* For development/debugging */
#ifdef DEBUG
# define CONFIG_CMD_REGINFO
# define CONFIG_PANIC_HANG
#endif

/* SPL part */
#define CONFIG_SPL_FRAMEWORK

/* FPGA support */
#define CONFIG_SPL_FPGA_SUPPORT
#define CONFIG_SPL_FPGA_LOAD_ADDR      0x1000000
/* #define CONFIG_SPL_FPGA_BIT */
#ifdef CONFIG_SPL_FPGA_BIT
# define CONFIG_SPL_FPGA_LOAD_ARGS_NAME "download.bit"
#else
# define CONFIG_SPL_FPGA_LOAD_ARGS_NAME "fpga.bin"
#endif

/* MMC support */
#ifdef CONFIG_MMC_SDHCI_ZYNQ
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION     1
#define CONFIG_SPL_FS_LOAD_PAYLOAD_NAME     "u-boot.img"
#endif

/* Disable dcache for SPL just for sure */
#ifdef CONFIG_SPL_BUILD
#define CONFIG_SYS_DCACHE_OFF
#endif

/* Address in RAM where the parameters must be copied by SPL. */
#define CONFIG_SYS_SPL_ARGS_ADDR	0x10000000

#define CONFIG_SPL_FS_LOAD_ARGS_NAME		"system.dtb"
#define CONFIG_SPL_FS_LOAD_KERNEL_NAME		"uImage"

/* Not using MMC raw mode - just for compilation purpose */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR	0
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS	0
#define CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR	0

/* qspi mode is working fine */
#ifdef CONFIG_ZYNQ_QSPI
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x100000
#define CONFIG_SYS_SPI_ARGS_OFFS	0x200000
#define CONFIG_SYS_SPI_ARGS_SIZE	0x80000
#define CONFIG_SYS_SPI_KERNEL_OFFS	(CONFIG_SYS_SPI_ARGS_OFFS + \
					CONFIG_SYS_SPI_ARGS_SIZE)
#endif

/* for booting directly linux */

/* SP location before relocation, must use scratch RAM */
#define CONFIG_SPL_TEXT_BASE	0x0

/* 3 * 64kB blocks of OCM - one is on the top because of bootrom */
#define CONFIG_SPL_MAX_SIZE	0x30000

/* On the top of OCM space */
#define CONFIG_SYS_SPL_MALLOC_START	CONFIG_SPL_STACK_R_ADDR
#define CONFIG_SYS_SPL_MALLOC_SIZE	0x2000000

/*
 * SPL stack position - and stack goes down
 * 0xfffffe00 is used for putting wfi loop.
 * Set it up as limit for now.
 */
#define CONFIG_SPL_STACK	0xfffffe00

/* BSS setup */
#define CONFIG_SPL_BSS_START_ADDR	0x100000
#define CONFIG_SPL_BSS_MAX_SIZE		0x100000

#define CONFIG_SYS_UBOOT_START	CONFIG_SYS_TEXT_BASE

#endif /* __CONFIG_ZYNQ_COMMON_H */
