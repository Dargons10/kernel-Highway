/*
 * Bock-W board support
 *
 * Copyright (C) 2013  Renesas Solutions Corp.
 * Copyright (C) 2013  Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/platform_device.h>
#include <linux/smsc911x.h>
#include <mach/common.h>
#include <mach/irqs.h>
#include <mach/r8a7778.h>
#include <asm/mach/arch.h>

static struct smsc911x_platform_config smsc911x_data = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_32BIT,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
};

static struct resource smsc911x_resources[] = {
	DEFINE_RES_MEM(0x18300000, 0x1000),
	DEFINE_RES_IRQ(irq_pin(0)), /* IRQ 0 */
};

/* USB */
static struct rcar_phy_platform_data usb_phy_platform_data __initdata;

/* SDHI */
static struct sh_mobile_sdhi_info sdhi0_info = {
	.tmio_caps	= MMC_CAP_SD_HIGHSPEED,
	.tmio_ocr_mask	= MMC_VDD_165_195 | MMC_VDD_32_33 | MMC_VDD_33_34,
	.tmio_flags	= TMIO_MMC_HAS_IDLE_WAIT,
};

static struct sh_eth_plat_data ether_platform_data __initdata = {
	.phy		= 0x01,
	.edmac_endian	= EDMAC_LITTLE_ENDIAN,
	.register_type	= SH_ETH_REG_FAST_RCAR,
	.phy_interface	= PHY_INTERFACE_MODE_RMII,
	/*
	 * Although the LINK signal is available on the board, it's connected to
	 * the link/activity LED output of the PHY, thus the link disappears and
	 * reappears after each packet.  We'd be better off ignoring such signal
	 * and getting the link state from the PHY indirectly.
	 */
	.no_ether_link	= 1,
};

/* I2C */
static struct i2c_board_info i2c0_devices[] = {
	{
		I2C_BOARD_INFO("rx8581", 0x51),
	},
};

/* HSPI*/
static struct mtd_partition m25p80_spi_flash_partitions[] = {
	{
		.name	= "data(spi)",
		.size	= 0x0100000,
		.offset	= 0,
	},
};

static struct flash_platform_data spi_flash_data = {
	.name		= "m25p80",
	.type		= "s25fl008k",
	.parts		= m25p80_spi_flash_partitions,
	.nr_parts	= ARRAY_SIZE(m25p80_spi_flash_partitions),
};

static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias	= "m25p80",
		.max_speed_hz	= 104000000,
		.chip_select	= 0,
		.bus_num	= 0,
		.mode		= SPI_MODE_0,
		.platform_data	= &spi_flash_data,
	},
};

/* MMC */
static struct sh_mmcif_plat_data sh_mmcif_plat = {
	.sup_pclk	= 0,
	.ocr		= MMC_VDD_165_195 | MMC_VDD_32_33 | MMC_VDD_33_34,
	.caps		= MMC_CAP_4_BIT_DATA |
			  MMC_CAP_8_BIT_DATA |
			  MMC_CAP_NEEDS_POLL,
};

static const struct pinctrl_map bockw_pinctrl_map[] = {
	/* Ether */
	PIN_MAP_MUX_GROUP_DEFAULT("r8a777x-ether", "pfc-r8a7778",
				  "ether_rmii", "ether"),
	/* HSPI0 */
	PIN_MAP_MUX_GROUP_DEFAULT("sh-hspi.0", "pfc-r8a7778",
				  "hspi0_a", "hspi0"),
	/* MMC */
	PIN_MAP_MUX_GROUP_DEFAULT("sh_mmcif", "pfc-r8a7778",
				  "mmc_data8", "mmc"),
	PIN_MAP_MUX_GROUP_DEFAULT("sh_mmcif", "pfc-r8a7778",
				  "mmc_ctrl", "mmc"),
	/* SCIF0 */
	PIN_MAP_MUX_GROUP_DEFAULT("sh-sci.0", "pfc-r8a7778",
				  "scif0_data_a", "scif0"),
	PIN_MAP_MUX_GROUP_DEFAULT("sh-sci.0", "pfc-r8a7778",
				  "scif0_ctrl", "scif0"),
	/* USB */
	PIN_MAP_MUX_GROUP_DEFAULT("ehci-platform", "pfc-r8a7778",
				  "usb0", "usb0"),
	PIN_MAP_MUX_GROUP_DEFAULT("ehci-platform", "pfc-r8a7778",
				  "usb1", "usb1"),
	/* SDHI0 */
	PIN_MAP_MUX_GROUP_DEFAULT("sh_mobile_sdhi.0", "pfc-r8a7778",
				  "sdhi0_data4", "sdhi0"),
	PIN_MAP_MUX_GROUP_DEFAULT("sh_mobile_sdhi.0", "pfc-r8a7778",
				  "sdhi0_ctrl", "sdhi0"),
	PIN_MAP_MUX_GROUP_DEFAULT("sh_mobile_sdhi.0", "pfc-r8a7778",
				  "sdhi0_cd", "sdhi0"),
	PIN_MAP_MUX_GROUP_DEFAULT("sh_mobile_sdhi.0", "pfc-r8a7778",
				  "sdhi0_wp", "sdhi0"),
};

#define FPGA	0x18200000

#define IRQ0MR	0x30
static void __init bockw_init(void)
{
	void __iomem *fpga;

	r8a7778_clock_init();
	r8a7778_init_irq_extpin(1);
	r8a7778_add_standard_devices();

	fpga = ioremap_nocache(0x18200000, SZ_1M);
	if (fpga) {
		/*
		 * CAUTION
		 *
		 * IRQ0/1 is cascaded interrupt from FPGA.
		 * it should be cared in the future
		 * Now, it is assuming IRQ0 was used only from SMSC.
		 */
		u16 val = ioread16(fpga + IRQ0MR);
		val &= ~(1 << 4); /* enable SMSC911x */
		iowrite16(val, fpga + IRQ0MR);
		iounmap(fpga);

		platform_device_register_resndata(
			&platform_bus, "smsc911x", -1,
			smsc911x_resources, ARRAY_SIZE(smsc911x_resources),
			&smsc911x_data, sizeof(smsc911x_data));
	}
}

static const char *bockw_boards_compat_dt[] __initdata = {
	"renesas,bockw",
	NULL,
};

DT_MACHINE_START(BOCKW_DT, "bockw")
	.init_early	= r8a7778_init_delay,
	.init_irq	= r8a7778_init_irq_dt,
	.init_machine	= bockw_init,
	.init_time	= shmobile_timer_init,
	.dt_compat	= bockw_boards_compat_dt,
MACHINE_END
