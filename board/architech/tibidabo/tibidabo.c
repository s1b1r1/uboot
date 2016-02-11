/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Copyright (C) 2014 Avnet Emg
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6q_pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <malloc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

/********************* uart *********************/

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |   \
    PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |     \
    PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t
    const uart2_pads[]      =
{
    MX6_PAD_EIM_D26__UART2_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D27__UART2_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
    imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
}

/********************* mmc *********************/

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |    \
    PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |     \
    PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t
    const usdhc3_pads[]     = 
{
    MX6_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT5__GPIO_7_0    | MUX_PAD_CTRL(NO_PAD_CTRL),      /* CD */    
};

static struct fsl_esdhc_cfg
    usdhc_cfg;

int board_mmc_getcd(struct mmc *mmc)
{
    struct fsl_esdhc_cfg
        *cfg                = (struct fsl_esdhc_cfg *)mmc->priv;

    if( cfg->esdhc_base == USDHC3_BASE_ADDR )
    {
        gpio_direction_input( IMX_GPIO_NR( 7, 0 ) );
        return !gpio_get_value( IMX_GPIO_NR( 7, 0 ) );
    }
    else
    {
        return 1;
    }
}

int board_mmc_init( bd_t *bis )
{
    usdhc_cfg.esdhc_base        = USDHC3_BASE_ADDR;
    usdhc_cfg.sdhc_clk          = mxc_get_clock( MXC_ESDHC3_CLK );
    usdhc_cfg.max_bus_width     = 4;
    imx_iomux_v3_setup_multiple_pads( usdhc3_pads, ARRAY_SIZE(usdhc3_pads) );
    return fsl_esdhc_initialize( bis, &usdhc_cfg );
}

/********************* spi flash *********************/

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |     \
    PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

static iomux_v3_cfg_t
    const ecspi1_pads[]         = 
{
    /* SS1 */
    MX6_PAD_EIM_D19__GPIO_3_19   | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

static void setup_spi(void)
{
    imx_iomux_v3_setup_multiple_pads( ecspi1_pads, ARRAY_SIZE( ecspi1_pads ) );
}

/********************* network *********************/

#define ENET_PAD_CTRL       (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

static iomux_v3_cfg_t
    const enet_pads1[]          =
{
    MX6_PAD_ENET_MDIO__ENET_MDIO        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_MDC__ENET_MDC          | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TXC__ENET_RGMII_TXC   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD0__ENET_RGMII_TD0   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD1__ENET_RGMII_TD1   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD2__ENET_RGMII_TD2   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD3__ENET_RGMII_TD3   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL  | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_REF_CLK__ENET_TX_CLK   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RXC__GPIO_6_30        | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_RGMII_RD0__GPIO_6_25        | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_RGMII_RD1__GPIO_6_27        | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_RGMII_RD2__GPIO_6_28        | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_RGMII_RD3__GPIO_6_29        | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_RGMII_RX_CTL__GPIO_6_24     | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_EIM_D23__GPIO_3_23          | MUX_PAD_CTRL(NO_PAD_CTRL),            /* Chip nRST */
    MX6_PAD_ENET_TX_EN__GPIO_1_28       | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t
    const enet_pads2[]          =
{
    MX6_PAD_RGMII_RXC__ENET_RGMII_RXC   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD0__ENET_RGMII_RD0   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD1__ENET_RGMII_RD1   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD2__ENET_RGMII_RD2   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD3__ENET_RGMII_RD3   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL  | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
    gpio_direction_input(  IMX_GPIO_NR( 1, 28 ) );                              /* Used to read when the switch registers can be accessed */
    gpio_direction_output( IMX_GPIO_NR( 3, 23 ), 0 );                           /* RGMII reset */
    gpio_direction_output( IMX_GPIO_NR( 6, 30 ), 1 );                           /* RGMII RXC - P5_GTXCLK/AM_DIS-in - Marvell Auto-Media Detect capable PHYs are not attached */

    /*
        P5_MODE[2:0] = 001 - Port 5 is configured to be in RGMII mode
        (1000 Mbps, full-duplex only)
    */
    gpio_direction_output( IMX_GPIO_NR( 6, 25 ), 1 );                           /* RGMII RD0 - P5_MODE[0] */
    gpio_direction_output( IMX_GPIO_NR( 6, 27 ), 0 );                           /* RGMII RD1 - P5_MODE[1] */
    gpio_direction_output( IMX_GPIO_NR( 6, 28 ), 0 );                           /* RGMII RD2 - P5_MODE[2] */

    imx_iomux_v3_setup_multiple_pads( enet_pads1, ARRAY_SIZE( enet_pads1 ) );   /* Initializing TX pads   */
    gpio_direction_output( IMX_GPIO_NR( 6, 24 ), 1 );                           /* RGMI RX CTL            */
    udelay( 10 );                                                               /* Tsu delay - Configuration data valid prior to RESETn de-asserted */
    gpio_set_value( IMX_GPIO_NR( 3, 23 ), 1 );                                  /* De-asserting RESETn    */
    imx_iomux_v3_setup_multiple_pads( enet_pads2, ARRAY_SIZE( enet_pads2 ) );   /* Initializing RX pads   */
}

static int wait_for_switch(void)
{
    uint32_t
        start;
    start                       = get_timer(0);
    while( gpio_get_value( IMX_GPIO_NR( 1, 28 ) ) )
    {
        if( get_timer( start ) > 3000 )
        {
            printf( "The ethernet switch doesn't come up\n" );
            return -1;
        }
    }
    return 0;
}

int board_eth_init(bd_t *bis)
{
	int ret;

	setup_iomux_enet();
    if( wait_for_switch() )
    {
        printf( "Impossible to access mv88e6123 chip registers.\n" );
        return 0;
    }
	ret                         = cpu_eth_init(bis);
	if( ret )
    {
		printf("FEC MXC: %s:failed\n", __func__);
    }
	return ret;
}

static struct mv88e61xx_config 
    swcfg               = 
{
    .name               = "FEC",
    .vlancfg            = MV88E61XX_VLANCFG_ROUTER,                             /* you can configure it as MV88E61XX_VLANCFG_SWITCH as well */
    .rgmii_delay        = MV88E61XX_RGMII_DELAY_EN,
    .led_init           = MV88E61XX_LED_INIT_EN,
    .mdip               = MV88E61XX_MDIP_NOCHANGE,
    .portstate          = MV88E61XX_PORTSTT_FORWARDING,
    .cpuport            = ( 1 << 5 ),
    .ports_enabled      = ( 1 << 5 ) | ( 1 << 1 ) | ( 1 << 0 ),
};

void reset_phy(void)
{
    mv88e61xx_switch_initialize( &swcfg );
}

/********************* i2c *********************/

#define I2C_PAD_CTRL    (PAD_CTL_PUS_100K_UP |              \
    PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
    PAD_CTL_ODE       | PAD_CTL_SRE_FAST )

#define PC              MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1 (SILICA LVDS/Raspberry Pi expansion) */
static struct i2c_pads_info
    i2c1_pads_info      =
{
    .scl                =
    {
        .i2c_mode       = MX6_PAD_CSI0_DAT9__I2C1_SCL  | PC,
        .gpio_mode      = MX6_PAD_CSI0_DAT9__GPIO_5_27 | PC,
        .gp             = IMX_GPIO_NR( 5, 27 )
    },
    .sda                =
    {
        .i2c_mode       = MX6_PAD_CSI0_DAT8__I2C1_SDA  | PC,
        .gpio_mode      = MX6_PAD_CSI0_DAT8__GPIO_5_26 | PC,
        .gp             = IMX_GPIO_NR( 5, 26 )
    }
};

/* I2C2 (PMIC and HDMI) */
static struct i2c_pads_info
    i2c2_pads_info      =
{
    .scl                =
    {
        .i2c_mode       = MX6_PAD_KEY_COL3__I2C2_SCL  | PC,
        .gpio_mode      = MX6_PAD_KEY_COL3__GPIO_4_12 | PC,
        .gp             = IMX_GPIO_NR( 4, 12 )
    },
    .sda                =
    {
        .i2c_mode       = MX6_PAD_KEY_ROW3__I2C2_SDA  | PC,
        .gpio_mode      = MX6_PAD_KEY_ROW3__GPIO_4_13 | PC,
        .gp             = IMX_GPIO_NR( 4, 13 )
    }
};

/* I2C3 (miniPCIe) */
static struct i2c_pads_info
    i2c3_pads_info      =
{
    .scl                =
    {
        .i2c_mode       = MX6_PAD_GPIO_5__I2C3_SCL | PC,
        .gpio_mode      = MX6_PAD_GPIO_5__GPIO_1_5 | PC,
        .gp             = IMX_GPIO_NR( 1, 5 )
    },
    .sda                =
    {
        .i2c_mode       = MX6_PAD_GPIO_6__I2C3_SDA | PC,
        .gpio_mode      = MX6_PAD_GPIO_6__GPIO_1_6 | PC,
        .gp             = IMX_GPIO_NR( 1, 6 )
    }
};

static void setup_tibidabo_i2cs(void)
{
    setup_i2c( 0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c1_pads_info );
    setup_i2c( 1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c2_pads_info );
    setup_i2c( 2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c3_pads_info );
}

/********************* display *********************/

struct display_info_t 
{
    int     bus;
    int     addr;
    int     pixfmt;
    int     (*detect)(struct display_info_t const *dev);
    void    (*enable)(struct display_info_t const *dev);
    struct  fb_videomode mode;
};

static int detect_hdmi(struct display_info_t const *dev)
{
    struct hdmi_regs
        *hdmi               = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
    return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}

static void enable_hdmi(struct display_info_t const *dev)
{
    imx_enable_hdmi_phy();
}

static struct display_info_t
    const displays[]        = 
{
    {
        .bus                = -1,
        .addr               = 0,
        .pixfmt             = IPU_PIX_FMT_RGB24,
        .detect             = detect_hdmi,
        .enable             = enable_hdmi,
        .mode               = 
        {
            .name           = "HDMI",
            .refresh        = 60,
            .xres           = 1024,
            .yres           = 768,
            .pixclock       = 15385,
            .left_margin    = 220,
            .right_margin   = 40,
            .upper_margin   = 21,
            .lower_margin   = 7,
            .hsync_len      = 60,
            .vsync_len      = 10,
            .sync           = FB_SYNC_EXT,
            .vmode          = FB_VMODE_NONINTERLACED
        } 
    },
};

int board_video_skip(void)
{
    int
        i,
        ret;
    char
        const *panel        = getenv("panel");
    if( !panel )
    {
        for( i = 0; i < ARRAY_SIZE( displays ); i++ )
        {            
            struct display_info_t
                const *dev  = displays + i;
            if( dev->detect && dev->detect( dev ) )
            {
                panel       = dev->mode.name;
                printf( "auto-detected panel %s\n", panel );
                break;
            }
        }
        if( !panel )
        {
            panel           = displays[0].mode.name;
            printf( "No panel detected: default to %s\n", panel );
        }
    }
    else
    {
        for( i = 0; i < ARRAY_SIZE( displays ); i++ )
        {
            if( !strcmp( panel, displays[ i ].mode.name ) )
            {
                break;
            }
        }
    }
    if( i < ARRAY_SIZE( displays ) )
    {
        ret                 = ipuv3_fb_init( &displays[ i ].mode, 0, displays[ i ].pixfmt );
        if( !ret )
        {
            displays[ i ].enable( displays + i );
            printf( "Display: %s (%ux%u)\n", displays[ i ].mode.name, displays[ i ].mode.xres, displays[ i ].mode.yres );
        }
        else
        {
            printf( "LCD %s cannot be configured: %d\n", displays[ i ].mode.name, ret );
        }
    }
    else
    {
        printf( "unsupported panel %s\n", panel );
        return -EINVAL;
    }
    return 0;
}

static void setup_display(void)
{
	struct mxc_ccm_reg
        *mxc_ccm            = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc
        *iomux              = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int
        reg;

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg                     = __raw_readl( &mxc_ccm->CCGR3 );
	reg                     |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel( reg, &mxc_ccm->CCGR3 );

	/* set LDB0, LDB1 clk select to 011/011 */
	reg                     = readl( &mxc_ccm->cs2cdr );
	reg                     &= ~( MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK );
	reg                     |= ( 3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET ) | ( 3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET );
	writel( reg, &mxc_ccm->cs2cdr );

	reg                     = readl( &mxc_ccm->cscmr2 );
	reg                     |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel( reg, &mxc_ccm->cscmr2 );

	reg                     = readl( &mxc_ccm->chsccdr );
	reg                     |= ( CHSCCDR_CLK_SEL_LDB_DI0 << MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET );
	reg                     |= ( CHSCCDR_CLK_SEL_LDB_DI0 << MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET );
	writel( reg, &mxc_ccm->chsccdr );

	reg                     = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	                        | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	                        | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	                        | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	                        | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	                        | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	                        | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	                        | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	                        | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel( reg, &iomux->gpr[ 2 ] );

	reg                     = readl( &iomux->gpr[ 3 ] );
	reg                     = ( reg & ~( IOMUXC_GPR3_LVDS1_MUX_CTL_MASK | IOMUXC_GPR3_HDMI_MUX_CTL_MASK ) )
	                        | ( IOMUXC_GPR3_MUX_SRC_IPU1_DI0 << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET );
	writel( reg, &iomux->gpr[ 3 ] );
}

/********************* sata *********************/

static int setup_sata(void)
{
	struct iomuxc_base_regs
        *const iomuxc_regs  = (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	int
        ret                 = enable_sata_clock();
	if( ret )
    {
		return ret;
    }
	clrsetbits_le32(
                &iomuxc_regs->gpr[13],
                IOMUXC_GPR13_SATA_MASK,
                  IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB
                | IOMUXC_GPR13_SATA_PHY_7_SATA2M
                | IOMUXC_GPR13_SATA_SPEED_3G
                | ( 3 << IOMUXC_GPR13_SATA_PHY_6_SHIFT )
                | IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED
                | IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16
                | IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB
                | IOMUXC_GPR13_SATA_PHY_2_TX_1P104V
                | IOMUXC_GPR13_SATA_PHY_1_SLOW
            );

	return 0;
}

/********************* information *********************/

int checkboard(void)
{
    puts( "Board: Tibidabo\n" );
    return 0;
}

int overwrite_console(void)
{
    return 1;
}

/********************* board init *********************/

int dram_init(void)
{
    gd->ram_size                = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE); 
    return 0;
}

int board_early_init_f(void)
{
    setup_iomux_uart();
    setup_display();
    return 0;
}

int board_init(void)
{
    gd->bd->bi_boot_params      = PHYS_SDRAM + 0x100;                       /* address of boot parameters */
    setup_spi();
    setup_tibidabo_i2cs();
	setup_sata();
    return 0;
}

int board_late_init(void)
{
    return 0;
}

int misc_init_r(void)
{
    return 0;
}
