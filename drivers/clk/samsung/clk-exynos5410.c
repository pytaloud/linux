/*
 * Copyright (c) 2013 Hardkernel Co., Ltd.
 * Authors: Hakjoo Kim <ruppi.kim@hardkernel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Common Clock Framework support for Exynos5410 SoC.
*/
#include <dt-bindings/clock/exynos5410.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "clk.h"
#include "clk-pll.h"

#define APLL_LOCK               0x0
#define APLL_CON0               0x100
#define CPLL_LOCK               0x10020
#define CPLL_CON0               0x10120
#define MPLL_LOCK               0x4000
#define MPLL_CON0               0x4100
#define BPLL_LOCK               0x20010
#define BPLL_CON0               0x20110
#define KPLL_LOCK               0x28000
#define KPLL_CON0               0x28100
#define CPLL_LOCK               0x10020
#define CPLL_CON0               0x10120
#define DPLL_LOCK               0x10030
#define DPLL_CON0               0x10128
#define EPLL_LOCK               0x10040
#define EPLL_CON0               0x10130
#define IPLL_LOCK               0x10060
#define IPLL_CON0               0x10150
#define VPLL_LOCK               0x10050
#define VPLL_CON0               0x10140

#define SRC_CPU			0x200
#define DIV_CPU0		0x500
#define DIV_CPU1		0x504
#define GATE_BUS_CPU		0x700
#define GATE_SCLK_CPU		0x800
#define SRC_CPERI0		0x4200
#define SRC_CPERI1		0x4204
#define DIV_G2D			0x8500
#define DIV_ISP0		0xc300
#define DIV_ISP1		0xc304
#define DIV_ISP2		0xc308
#define SRC_TOP0		0x10210
#define SRC_TOP1		0x10214
#define SRC_TOP2		0x10218
#define SRC_TOP3		0x1021c
#define SRC_GSCL		0x10220
#define SRC_DISP10		0x1022c
#define SRC_MAU			0x10240
#define SRC_FSYS		0x10244
#define SRC_PERIC0		0x10250
#define SRC_PERIC1		0x10254
#define SRC_ISP			0x10270
#define	SRC_MASK_DISP10		0x1032c
#define SRC_MASK_FSYS		0x10340
#define SRC_MASK_PERIC0		0x10350
#define SRC_MASK_PERIC1		0x10354
#define DIV_TOP0		0x10510
#define DIV_TOP1		0x10514
#define DIV_TOP2		0x10518
#define DIV_TOP3		0x1051c
#define DIV_GSCL		0x10520
#define DIV_DISP10		0x1052c
#define DIV_DISP11		0x10530
#define DIV_GEN			0x1053c
#define DIV_MAU			0x10544
#define DIV_FSYS0		0x10548
#define DIV_FSYS1		0x1054c
#define DIV_FSYS2		0x10550
#define DIV_FSYS3		0x10554
#define DIV_PERIC0		0x10558
#define DIV_PERIC1		0x1055c
#define DIV_PERIC2		0x10560
#define DIV_PERIC3		0x10564
#define DIV_PERIC4		0x10568
#define DIV_PERIC5		0x1056c
#define CLK_DIV4		0x105a0
#define GATE_BUS_DISP1		0x10728
#define GATE_BUS_FSYS0		0x10740
#define GATE_BUS_PERIC		0x10750
#define GATE_BUS_PERIS0		0x10760
#define GATE_BUS_PERIS1		0x10764
#define GATE_IP_GSCL0		0x10910
#define GATE_IP_GSCL1		0x10920
#define GATE_IP_MFC		0x1092c
#define GATE_IP_DISP1		0x10928
#define GATE_IP_G3D		0x10930
#define GATE_IP_GEN		0x10934
#define GATE_IP_FSYS	0x10944
#define GATE_IP_PERIC		0x10950
#define GATE_TOP_SCLK_GSCL	0x10820
#define GATE_TOP_SCLK_DISP1	0x10828
#define GATE_TOP_SCLK_MAU	0x1083c
#define GATE_TOP_SCLK_FSYS	0x10840
#define GATE_TOP_SCLK_PERIC	0x10850
#define GATE_IP_PERIS		0x10960
#define SRC_CDREX		0x20200
#define SRC_KFC			0x28200
#define DIV_KFC0		0x28500

/* list of PLLs */
enum exynos5410_plls {
	apll, cpll, dpll, epll, ipll, vpll, mpll,
        bpll, kpll, 
	nr_plls			/* number of PLLs */
};

enum exynos5410_clks {
	none,

	/* core clocks */
	fin_pll, sclk_vpll,

	/* gate for special clocks (sclk) */
	sclk_uart0 = 128, sclk_uart1, sclk_uart2, sclk_uart3, sclk_mmc0,
	sclk_mmc1, sclk_mmc2, sclk_spi0, sclk_spi1, sclk_spi2, sclk_i2s1,
	sclk_i2s2, sclk_pcm1, sclk_pcm2, sclk_spdif, sclk_hdmi, sclk_pixel,
	sclk_dp1, sclk_mipi1, sclk_fimd1, sclk_maudio0, sclk_maupcm0,
	sclk_usbd300, sclk_usbd301, sclk_usbphy300, sclk_usbphy301, sclk_unipro,
	sclk_pwm, sclk_gscl_wa, sclk_gscl_wb, sclk_audio0, sclk_audio1, sclk_audio2,

	/* gate clocks */
	aclk66_peric = 256, uart0, uart1, uart2, uart3, i2c0, i2c1, i2c2, i2c3,
	i2c4, i2c5, i2c6, i2c7, i2c_hdmi, tsadc, spi0, spi1, spi2, keyif, i2s1,
	i2s2, pcm1, pcm2, pwm, spdif, ac97,
	aclk66 = 300, chipid, sysreg, tzpc0, tzpc1, tzpc2, tzpc3, tzpc4,
   	tzpc5, tzpc6, tzpc7, tzpc8, tzpc9, hdmi_cec, seckey, mct, wdt, rtc,
	tmu, tmu_gpu,
	pclk66_gpio = 330,
	aclk200_fsys2 = 350, mmc0, mmc1, mmc2, sromc, ufs,
	aclk200_fsys = 360, tsi, pdma0, pdma1, rtic, usbh20, usbd300, usbd301,
	aclk400_mscl = 380, mscl0, mscl1, mscl2, smmu_mscl0, smmu_mscl1,
	smmu_mscl2,
	aclk333 = 400, mfc, smmu_mfcl, smmu_mfcr,
	aclk200_disp1 = 410, dsim1, dp1, hdmi,
	aclk300_disp1 = 420, fimd1,	smmu_fimd1,
	aclk166 = 430, mixer,
	aclk266 = 440, rotator, mdma1, smmu_rotator, smmu_mdma1,
	aclk300_jpeg = 450, jpeg, jpeg2, smmu_jpeg,
	aclk300_gscl = 460, smmu_gscl0, smmu_gscl1, gscl_wa, gscl_wb, gscl0, 
	gscl1, clk_3aa,
	aclk266_g2d = 470, sss, slim_sss, mdma0,
	aclk333_g2d = 480, g2d,
	aclk333_432_gscl = 490, smmu_3aa, smmu_fimcl0, smmu_fimcl1, smmu_fimcl3,
	fimc_lite3,
	aclk_g3d = 500, g3d, smmu_tv, pkey0, pkey1, monocnt,

	/* mux clocks */
	mout_fimd1 = 1024, mout_maudio0, mout_hdmi, mout_spi0, mout_spi1,
	mout_spi2,

	nr_clks,
};

/*
 * list of controller registers to be saved and restored during a
 * suspend/resume cycle.
 */
static __initdata unsigned long exynos5410_clk_regs[] = {
	SRC_CPU,
	DIV_CPU0,
	DIV_CPU1,
	GATE_BUS_CPU,
	GATE_SCLK_CPU,
	SRC_TOP0,
	SRC_TOP1,
	SRC_TOP2,
	SRC_TOP3,
	SRC_DISP10,
	SRC_MAU,
	SRC_FSYS,
	SRC_PERIC0,
	SRC_PERIC1,
	SRC_MASK_DISP10,
	SRC_MASK_FSYS,
	SRC_MASK_PERIC0,
	SRC_MASK_PERIC1,
	DIV_TOP0,
	DIV_TOP1,
	DIV_TOP2,
	DIV_DISP10,
	DIV_MAU,
	DIV_FSYS0,
	DIV_FSYS1,
	DIV_FSYS2,
	DIV_PERIC0,
	DIV_PERIC1,
	DIV_PERIC2,
	DIV_PERIC3,
	DIV_PERIC4,
	GATE_BUS_FSYS0,
	GATE_BUS_PERIC,
	GATE_BUS_PERIS0,
	GATE_BUS_PERIS1,
	GATE_IP_GSCL0,
	GATE_IP_GSCL1,
	GATE_IP_MFC,
	GATE_IP_DISP1,
	GATE_IP_G3D,
	GATE_IP_GEN,
	GATE_IP_FSYS,
	GATE_IP_PERIC,
	GATE_TOP_SCLK_GSCL,
	GATE_TOP_SCLK_DISP1,
	GATE_TOP_SCLK_MAU,
	GATE_TOP_SCLK_FSYS,
	GATE_TOP_SCLK_PERIC,
	SRC_CDREX,
	SRC_KFC,
	DIV_KFC0,
};

/* list of all parent clocks */
PNAME(mout_apll_p)	= { "fin_pll", "fout_apll" };
PNAME(mout_arm_p)	= { "mout_apll", "sclk_mpll", };

PNAME(mout_kpll_p)	= { "fin_pll", "fout_kpll" };
PNAME(mout_cpu_kfc_p)	= { "mout_kpll" , "fout_mpll" };

PNAME(group1_p)	= { "fin_pll", "fin_pll", "sclk_hdmi27m",
				"sclk_dptxphy", "sclk_uhostphy", "sclk_hdmiphy",
				"sclk_mpll_bpll", "sclk_epll", "sclk_vpll",
				"sclk_cpll" };

PNAME(group2_p)		= { "sclk_mpll_user", "sclk_bpll_user" };

PNAME(group3_p)		= { "sclk_ipll", "sclk_vpll" };
PNAME(group4_p)		= { "sclk_cpll", "sclk_mpll_user" };
PNAME(usb_p)		= { "sclk_mpll_bpll", "fin_pll" };

PNAME(mout_cpll_p)	= { "fin_pll", "fout_cpll" };
PNAME(sclk_dpll_p)	= { "fin_pll", "fout_dpll" };
PNAME(sclk_epll_p)	= { "fin_pll", "fout_epll" };
PNAME(sclk_ipll_p)	= { "fin_pll", "fout_ipll" };
//PNAME(mout_vpll_p)	= { "mout_vpllsrc", "fout_vpll" };
PNAME(mout_vpll_p)	= { "fout_vpll", "mout_vpllsrc" };

PNAME(sclk_mpll_p)	= { "fin_pll", "fout_mpll" };
PNAME(sclk_bpll_p)	= { "fin_pll", "fout_bpll" };

PNAME(sclk_mpll_bpll_p) = { "sclk_mpll", "sclk_bpll" };

PNAME(vpllsrc_p)	= { "fin_pll", "sclk_hdmi27m" };

PNAME(user_aclk266_isp_p) = { "fin_pll", "mount_sw_aclk_isp" };
PNAME(user_aclk400_isp_p) = { "fin_pll", "mout_sw_aclk400_isp" };

PNAME(user_aclk333_432_isp_p) = { "fin_pll", "mout_sw_aclk333_432_isp" };
PNAME(user_aclk333_p)	= { "fin_pll", "dout_aclk333" };

PNAME(user_aclk333_432_gscl_p)	= { "fin_pll", "dout_aclk333_432_gscl" };

PNAME(user_aclk300_gscl_p)	= { "fin_pll", "dout_aclk300_gscl" };
PNAME(user_aclk200_disp1_p)	= { "fin_pll", "aclk200" };
PNAME(user_aclk300_disp1_p)	= { "fin_pll", "dout_aclk300_disp1" };

PNAME(user_aclk266_gscl_p)	= { "fin_pll", "dout_aclk266_gscl" };

PNAME(user_aclk300_jpeg_p)	= { "fin_pll", "dout_aclk300_jpeg" };

PNAME(user_aclk333_480_g3d_core_p)	= { "fin_pll", "dout_aclk333_480_g3d" };
PNAME(user_aclk333_480_g3d_hydra_p)	= { "fin_pll", "dout_aclk333_480_g3d" };

PNAME(audio0_p)	= { "cdclk0", "fin_pll", "sclk_hdmi27m", "sclk_dptxphy",
				"sclk_uhostphy", "sclk_hdmiphy",
				"sclk_mpll_bpll", "sclk_epll", "sclk_vpll",
				"sclk_cpll" };
PNAME(audio1_p)	= { "cdclk1", "fin_pll", "sclk_hdmi27m", "sclk_dptxphy",
				"sclk_uhostphy", "sclk_hdmiphy",
				"sclk_mpll_bpll", "sclk_epll", "sclk_vpll",
				"sclk_cpll" };
PNAME(audio2_p)	= { "cdclk2", "fin_pll", "sclk_hdmi27m", "sclk_dptxphy",
				"sclk_uhostphy", "sclk_hdmiphy",
				"sclk_mpll_bpll", "sclk_epll", "sclk_vpll",
				"sclk_cpll" };

PNAME(mout_hdmi_p)	= { "dout_hdmi_pixel", "sclk_hdmiphy" };
PNAME(spdif_p)	= { "sclk_audio0", "sclk_audio1", "sclk_audio2",
				"spdif_extclk" };
PNAME(maudio0_p)	= { "fin_pll", "maudio_clk", "sclk_mpll_bpll", 
				"sclk_epll", "sclk_vpll", "sclk_cpll" };

/* fixed rate clocks generated outside the soc */
struct samsung_fixed_rate_clock exynos5410_fixed_rate_ext_clks[] __initdata = {
	FRATE(fin_pll, "fin_pll", NULL, CLK_IS_ROOT, 24000000),
};

/* fixed rate clocks generated inside the soc */
struct samsung_fixed_rate_clock exynos5410_fixed_rate_clks[] __initdata = {
	FRATE(none, "sclk_hdmiphy", NULL, CLK_IS_ROOT, 24000000),
	FRATE(none, "sclk_hdmi27m", NULL, CLK_IS_ROOT, 24000000),
	FRATE(none, "sclk_dptxphy", NULL, CLK_IS_ROOT, 24000000),
	FRATE(none, "sclk_uhostphy", NULL, CLK_IS_ROOT, 48000000),
};

static struct samsung_mux_clock exynos5410_pll_mux_clks[] __initdata = {
	MUX(none, "mout_vpllsrc", vpllsrc_p, SRC_TOP2, 0, 1),
};

struct samsung_mux_clock exynos5410_mux_clks[] __initdata = {
	MUX_A(none, "mout_apll", mout_apll_p, SRC_CPU, 0, 1, "mout_apll"),
	MUX_A(none, "mout_arm", mout_arm_p, SRC_CPU, 16, 1, "mout_arm"),
	MUX_A(none, "mout_arm_hpm", mout_arm_p, SRC_CPU, 20, 1, "mout_arm_hpm"),

	MUX(none, "mout_kpll", mout_kpll_p, SRC_KFC, 0, 1),
	MUX(none, "mout_hpm_kfc", mout_cpu_kfc_p, SRC_KFC, 15, 1),
	MUX(none, "mout_cpu_kfc", mout_cpu_kfc_p, SRC_KFC, 16, 1),

	MUX(none, "sclk_mpll", sclk_mpll_p, SRC_CPERI1, 8, 1),
	MUX(none, "sclk_bpll", sclk_bpll_p, SRC_CDREX, 0, 1),

	/* CMU_TOP Block */
	MUX(none, "sclk_mpll_bpll", sclk_mpll_bpll_p, SRC_TOP1, 20, 1),

	MUX(none, "sclk_cpll", mout_cpll_p, SRC_TOP2, 8, 1),
	MUX(none, "sclk_dpll", sclk_dpll_p, SRC_TOP2, 10, 1),
	MUX(none, "sclk_epll", sclk_epll_p, SRC_TOP2, 12, 1),
	MUX(none, "sclk_ipll", sclk_ipll_p, SRC_TOP2, 14, 1),
	MUX_F(sclk_vpll, "sclk_vpll", mout_vpll_p, SRC_TOP2, 16, 1,
			CLK_SET_RATE_PARENT, 0),
	MUX(none, "sclk_mpll_user", sclk_mpll_p, SRC_TOP2, 20, 1),
	MUX(none, "sclk_bpll_user", sclk_bpll_p, SRC_TOP2, 24, 1),

	/* MFC Block */
	MUX(none, "mout_aclk333", group4_p, SRC_TOP0, 16, 1),
	MUX_A(none, "aclk333",user_aclk333_p, SRC_TOP3, 24, 1, "aclk333"),

	/* ISP Block */
	MUX(none, "mout_sw_aclk400_isp", sclk_mpll_bpll_p,
			SRC_TOP1, 24, 1),
	MUX(none, "mout_user_aclk266_isp",user_aclk266_isp_p, 
			SRC_TOP3, 16, 3),
	MUX(none, "mout_user_aclk400_isp",user_aclk400_isp_p, 
			SRC_TOP3, 20, 3),
	MUX(none, "mout_user_aclk333_432_isp",user_aclk333_432_isp_p, 
			SRC_TOP3, 25, 1),
	MUX(none, "mout_spi0_isp", group1_p, SRC_ISP, 0, 4),
	MUX(none, "mout_spi1_isp", group1_p, SRC_ISP, 4, 4),
	MUX(none, "mout_uart_isp", group1_p, SRC_ISP, 8, 4),
	MUX(none, "mout_pwm_isp", group1_p, SRC_ISP, 12, 4),

	/* PSGEN Block */
	MUX(none, "mout_aclk166", group4_p,	SRC_TOP0, 8, 1),
	MUX_A(none, "aclk300_jpeg", user_aclk300_jpeg_p, 
			SRC_TOP3, 27, 1, "aclk300_jpeg"),

	/* G3D Block */
	MUX(none, "mout_aclk333_480", group2_p,	SRC_TOP0, 20, 1),
	MUX(none, "mout_user_aclk333_480_g3d_core",user_aclk333_480_g3d_core_p, 
			SRC_TOP3, 28, 1),
	MUX(none, "mout_user_aclk333_480_g3d_hydra",user_aclk333_480_g3d_hydra_p, 
			SRC_TOP3, 29, 1),

	/* GSCL Block */
	MUX(none, "mout_aclk266_gscl", group2_p,
			SRC_TOP1, 28, 1),
	MUX(none, "mout_user_aclk266_gscl",user_aclk266_gscl_p, 
			SRC_TOP3, 8, 3),
	MUX(none, "mout_user_aclk333_480_gscl",user_aclk333_432_gscl_p, 
			SRC_TOP3, 26, 1),


	MUX(none, "mout_user_aclk300_gscl",user_aclk300_gscl_p, 
			SRC_TOP3, 17, 1),

	/* DISP1 Block */
	MUX(none, "mout_aclk200", group2_p,
			SRC_TOP0, 12, 1),
	MUX_A(none, "aclk200_disp1",user_aclk200_disp1_p, 
			SRC_TOP3, 4, 1, "aclk200_disp1"),
	MUX_A(none, "aclk300_disp1",user_aclk300_disp1_p, 
			SRC_TOP3, 19, 1, "aclk300_disp1"),
	MUX_F(mout_fimd1, "mout_fimd1", group1_p, SRC_DISP10, 0, 3,
			CLK_SET_RATE_PARENT, 0),
	MUX(none, "mout_mipi1", group1_p, SRC_DISP10, 12, 4),
	MUX(none, "mout_dp1", group1_p, SRC_DISP10, 16, 4),
	MUX(mout_hdmi, "mout_hdmi", mout_hdmi_p, SRC_DISP10, 20, 1),

	/* MAU Block */
	MUX(none, "mout_maudio0", maudio0_p, SRC_MAU, 28, 3),

	/* FSYS Block */
	MUX(none, "mout_aclk400", group2_p, SRC_TOP0, 20, 1),
	MUX(none, "mout_aclk_mipi_txbase", group2_p, SRC_TOP1, 16, 1),
	MUX(none, "mout_mmc0", group1_p, SRC_FSYS, 0, 4),
	MUX(none, "mout_mmc1", group1_p, SRC_FSYS, 4, 4),
	MUX(none, "mout_mmc2", group1_p, SRC_FSYS, 8, 4),

	MUX(none, "mout_usbd300", usb_p, SRC_FSYS, 28, 1),
	MUX(none, "mout_usbd301", usb_p, SRC_FSYS, 29, 1),

	/* PERIC Block */
	MUX(none, "mout_uart0", group1_p, SRC_PERIC0, 0, 4),
	MUX(none, "mout_uart1", group1_p, SRC_PERIC0, 4, 4),
	MUX(none, "mout_uart2", group1_p, SRC_PERIC0, 8, 4),
	MUX(none, "mout_uart3", group1_p, SRC_PERIC0, 12, 4),
	MUX(none, "mout_pwm", group1_p, SRC_PERIC0, 24, 4),
	MUX(none, "mout_audio1", audio1_p, SRC_PERIC1, 0, 4),
	MUX(none, "mout_audio2", audio2_p, SRC_PERIC1, 4, 4),
	MUX(none, "mout_spdif", spdif_p, SRC_PERIC1, 8, 2),
	MUX(none, "mout_audio0", audio0_p, SRC_PERIC1, 12, 4),
	MUX(none, "mout_spi0", group1_p, SRC_PERIC1, 16, 4),
	MUX(none, "mout_spi1", group1_p, SRC_PERIC1, 20, 4),
	MUX(none, "mout_spi2", group1_p, SRC_PERIC1, 24, 4),

	/* ETC Block */
	MUX(none, "mout_isp_sensor", group3_p, SRC_ISP, 20, 1),
	MUX(none, "mout_cam_bayer", group1_p, SRC_GSCL, 12, 4),
	MUX(none, "mout_cam1", group1_p, SRC_GSCL, 20, 4),

};

struct samsung_div_clock exynos5410_div_clks[] __initdata = {
	/* EAGE Block */
	DIV(none, "aclk_cpud", "armclk2", DIV_CPU0, 4, 3),
	DIV(none, "atclk", "armclk2", DIV_CPU0, 16, 3),
	DIV(none, "pclk_dbg", "armclk2", DIV_CPU0, 20, 3),
	DIV(none, "sclk_apll", "mout_apll", DIV_CPU0, 24, 3),
	DIV(none, "armclk2", "mout_arm", DIV_CPU0, 28, 3),

	/* KFC Block */
	DIV(none, "div_kfc", "mout_cpu_kfc", DIV_KFC0, 0, 3),
	DIV(none, "sclk_kpll", "mout_kpll", DIV_KFC0, 24, 3),

	DIV(none, "dout_copy", "mout_apll", DIV_CPU1, 0, 3),
	DIV(none, "sclk_hpm_cpu", "dout_copy", DIV_CPU1, 4, 3),

	DIV(none, "aclk_acp", "sclk_mpll", DIV_G2D, 0, 3),
	DIV(none, "pclk_acp", "aclk_acp", DIV_G2D, 4, 3),

	DIV(none, "aclk_div0", "ack333_432_isp", DIV_ISP0, 0, 3),
	DIV(none, "aclk_div1", "ack333_432_isp", DIV_ISP0, 4, 3),

	DIV(none, "aclk_mcuisp_div0", "aclk_400_isp", DIV_ISP1, 0, 3),
	DIV(none, "aclk_mcuisp_div1", "aclk_400_isp", DIV_ISP1, 4, 3),

	DIV(none, "aclk_div2", "aclk_div1", DIV_ISP2, 4, 3),

	DIV(none, "dout_aclk66_pre", "sclk_mpll_user", DIV_TOP1, 24, 3),
	DIV(none, "aclk66", "dout_aclk66_pre", DIV_TOP0, 0, 3),
	DIV(none, "aclk166", "mout_aclk166", DIV_TOP0, 8, 3),
	DIV(none, "aclk200", "mout_aclk200", DIV_TOP0, 12, 3),
	DIV(none, "aclk266", "sclk_mpll_user", DIV_TOP0, 16, 3),
	DIV(none, "dout_aclk333", "mout_aclk333", DIV_TOP0, 20, 3),
	DIV(none, "aclk400", "mout_aclk400", DIV_TOP0, 24, 3),

	DIV(none, "dout_aclk_400_isp", "aclk400_isp", DIV_TOP1, 20, 3),
	DIV(none, "dout_aclk_mipi_txbase", "mout_aclk_mipi_txbase",
			DIV_TOP1, 28, 3),

	DIV(none, "aclk200_gscl", "mout_aclk200_gscl", DIV_TOP2, 0, 3),
	DIV(none, "aclk266_gscl", "mout_aclk266_gscl", DIV_TOP2, 4, 3),
	DIV(none, "dout_aclk300_disp1", "sclk_dpll_p",
			DIV_TOP2, 16, 3),

	DIV(none, "dout_aclk333_432_isp", "sclk_ipll_p", DIV_TOP2, 20, 3),

	DIV(none, "dout_aclk333_432_gsclk", "sclk_ipll_p", DIV_TOP2, 24, 3),
	DIV(none, "dout_aclk300_jpeg", "sclk_dpll_p", DIV_TOP2, 28, 3),

	DIV(none, "aclk300_480_g3d_core", "mout_aclk_333_480", DIV_TOP3, 0, 3),
	DIV(none, "aclk300_480_g3d_hydra", "mout_aclk_333_480", DIV_TOP3, 5, 3),

	DIV(none, "sclk_cam_bayer", "mout_cam_bayer", DIV_GSCL, 12, 3),
	DIV(none, "sclk_cam1", "mout_cam1", DIV_GSCL, 20, 3),

	/* DISP1 Block */
	DIV_F(none, "dout_fimd1", "mout_fimd1", DIV_DISP10, 0, 4,
			CLK_SET_RATE_PARENT, 0),

	DIV(none, "dout_mipi1", "mout_mipi1", DIV_DISP10, 16, 4),
	DIV(none, "dout_mipi1_pre", "mout_mipi1_pre", DIV_DISP10, 20, 4),
	DIV(none, "dout_dp1", "mout_dp1", DIV_DISP10, 24, 4),
	DIV(sclk_pixel, "dout_hdmi_pixel", "sclk_vpll", DIV_DISP10, 28, 4),

	DIV(none, "sclk_jpeg", "sclk_cpll", DIV_GEN, 4, 4),

	/* Audio Block */
	DIV(none, "sclk_maudio", "mout_maudio0", DIV_MAU, 0, 4),
	DIV(none, "sclk_maupcm0", "sclk_maudio0", DIV_MAU, 11, 8),

	/* USB3.0 */
	DIV(none, "sclk_usbphy300", "mout_usbd300", DIV_FSYS0, 16, 4),
	DIV(none, "sclk_usbphy301", "mout_usbd301", DIV_FSYS0, 20, 4),
	DIV(none, "sclk_usbd300", "mout_usbd300", DIV_FSYS0, 24, 4),
	DIV(none, "sclk_usbd301", "mout_usbd301", DIV_FSYS0, 28, 4),

	/* MMC */
	DIV(none, "dout_mmc0", "mout_mmc0", DIV_FSYS1, 0, 4),
	DIV(none, "dout_mmc1", "mout_mmc1", DIV_FSYS1, 16, 4),
	DIV(none, "dout_mmc2", "mout_mmc2", DIV_FSYS2, 0, 4),

	/* UART and PWM */
	DIV(none, "dout_uart0", "mout_uart0", DIV_PERIC0, 0, 4),
	DIV(none, "dout_uart1", "mout_uart1", DIV_PERIC0, 4, 4),
	DIV(none, "dout_uart2", "mout_uart2", DIV_PERIC0, 8, 4),
	DIV(none, "dout_uart3", "mout_uart3", DIV_PERIC0, 12, 4),
	DIV(none, "dout_pwm", "mout_pwm", DIV_PERIC3, 0, 4),

	/* SPI */
	DIV(none, "dout_spi0", "mout_spi0", DIV_PERIC1, 0, 4),
	DIV(none, "dout_spi1", "mout_spi1", DIV_PERIC1, 16, 4),
	DIV(none, "dout_spi2", "mout_spi2", DIV_PERIC2, 0, 4),

	/* PCM */
	DIV(none, "dout_pcm1", "sclk_audio1", DIV_PERIC4, 11, 8),
	DIV(none, "dout_pcm2", "sclk_audio2", DIV_PERIC4, 12, 8),

	/* Audio - I2S */
	DIV(none, "dout_i2s0", "sclk_audio0", DIV_PERIC5, 0, 6),
	DIV(none, "dout_i2s1", "sclk_audio1", DIV_PERIC5, 8, 6),
	DIV(none, "dout_i2s2", "sclk_audio2", DIV_PERIC5, 16, 6),
	DIV(none, "dout_audio0", "mout_audio0", DIV_PERIC5, 24, 4),
	DIV(none, "dout_audio1", "mout_audio1", DIV_PERIC4, 0, 4),
	DIV(none, "dout_audio2", "mout_audio2", DIV_PERIC4, 28, 4),

	/* SPI Pre-Ratio */
	DIV(none, "dout_spi0_pre", "dout_spi0", DIV_PERIC1, 8, 8),
	DIV(none, "dout_spi1_pre", "dout_spi1", DIV_PERIC1, 24, 8),
	DIV(none, "dout_spi2_pre", "dout_spi2", DIV_PERIC2, 8, 8),
	DIV_F(none, "dout_pre_mmc0", "dout_mmc0",
			DIV_FSYS1, 8, 8, CLK_SET_RATE_PARENT, 0),
	DIV_F(none, "dout_pre_mmc1", "dout_mmc1",
			DIV_FSYS1, 24, 8, CLK_SET_RATE_PARENT, 0),
	DIV_F(none, "dout_pre_mmc2", "dout_mmc2",
			DIV_FSYS2, 8, 8, CLK_SET_RATE_PARENT, 0),
	};

struct samsung_gate_clock exynos5410_gate_clks[] __initdata = {
	/* TODO: Re-verify the CG bits for all the gate clocks */
	GATE_A(mct, "pclk_st", "aclk66", GATE_BUS_PERIS1, 2, 0, 0, "mct"),

	/* sclk */
	GATE(sclk_uart0, "sclk_uart0", "dout_uart0",
		SRC_MASK_PERIC0, 0, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_uart1, "sclk_uart1", "dout_uart1",
		SRC_MASK_PERIC0, 4, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_uart2, "sclk_uart2", "dout_uart2",
		SRC_MASK_PERIC0, 8, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_uart3, "sclk_uart3", "dout_uart3",
		SRC_MASK_PERIC0, 12, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_pwm, "sclk_pwm", "dout_pwm",
		SRC_MASK_PERIC0, 24, CLK_SET_RATE_PARENT, 0),

	GATE(sclk_spi0, "sclk_spi0", "dout_spi0_pre",
		SRC_MASK_PERIC1, 16, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_spi1, "sclk_spi1", "dout_spi1_pre",
		SRC_MASK_PERIC1, 20, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_spi2, "sclk_spi2", "dout_spi2_pre",
		SRC_MASK_PERIC1, 24, CLK_SET_RATE_PARENT, 0),

	GATE(sclk_audio0, "sclk_audio0", "dout_audio0",
		SRC_MASK_PERIC1, 12, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_audio1, "sclk_audio1", "dout_audio1",
		SRC_MASK_PERIC1, 0, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_audio2, "sclk_audio2", "dout_audio2",
		SRC_MASK_PERIC1, 4, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_spdif, "sclk_spdif", "mout_spdif",
		SRC_MASK_PERIC1, 8, CLK_SET_RATE_PARENT, 0),

	GATE(sclk_pcm1, "sclk_pcm1", "dout_pcm1",
		GATE_TOP_SCLK_PERIC, 15, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_pcm2, "sclk_pcm2", "dout_pcm2",
		GATE_TOP_SCLK_PERIC, 16, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_i2s1, "sclk_i2s1", "dout_pre_i2s1",
		GATE_TOP_SCLK_PERIC, 17, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_i2s2, "sclk_i2s2", "dout_pre_i2s2",
		GATE_TOP_SCLK_PERIC, 18, CLK_SET_RATE_PARENT, 0),

	/* MMC */
	GATE(sclk_mmc0, "sclk_mmc0", "dout_pre_mmc0",
		SRC_MASK_FSYS, 0, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_mmc1, "sclk_mmc1", "dout_pre_mmc1",
		SRC_MASK_FSYS, 4, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_mmc2, "sclk_mmc2", "dout_pre_mmc2",
		SRC_MASK_FSYS, 8, CLK_SET_RATE_PARENT, 0),

	/* USB 2.0/3.0 */
	GATE(usbh20, "usbh20", "aclk200",
		GATE_IP_FSYS, 19, CLK_SET_RATE_PARENT, 0),
	GATE(usbd300, "usbd300", "aclk200",
		GATE_IP_FSYS, 19, CLK_SET_RATE_PARENT, 0),
	GATE(usbd301, "usbd301", "aclk200",
		GATE_IP_FSYS, 20, CLK_SET_RATE_PARENT, 0),


	GATE(sclk_gscl_wa, "sclk_gscl_wa", "aclK333_432_gscl",
		GATE_TOP_SCLK_GSCL, 6, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_gscl_wb, "sclk_gscl_wb", "aclk333_432_gscl",
		GATE_TOP_SCLK_GSCL, 7, CLK_SET_RATE_PARENT, 0),

	/* Display */
	GATE(sclk_fimd1, "sclk_fimd1", "dout_fimd1",
		GATE_TOP_SCLK_DISP1, 0, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_mipi1, "sclk_mipi1", "dout_mipi1",
		GATE_TOP_SCLK_DISP1, 3, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_hdmi, "sclk_hdmi", "mout_hdmi",
		GATE_TOP_SCLK_DISP1, 9, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_pixel, "sclk_pixel", "dout_hdmi_pixel",
		GATE_TOP_SCLK_DISP1, 10, CLK_SET_RATE_PARENT, 0),
	GATE(sclk_dp1, "sclk_dp1", "dout_dp1",
		GATE_TOP_SCLK_DISP1, 20, CLK_SET_RATE_PARENT, 0),

	/* Maudio Block */

	/* FSYS */
	GATE(tsi, "tsi", "aclk200", GATE_BUS_FSYS0, 0, 0, 0),
	GATE(pdma0, "pdma0", "aclk200", GATE_BUS_FSYS0, 1, 0, 0),
	GATE(pdma1, "pdma1", "aclk200", GATE_BUS_FSYS0, 2, 0, 0),
	GATE(ufs, "ufs", "aclk200", GATE_BUS_FSYS0, 3, 0, 0),
	GATE(rtic, "rtic", "aclk200", GATE_BUS_FSYS0, 5, 0, 0),
	GATE(mmc0, "mmc0", "aclk200", GATE_BUS_FSYS0, 12, 0, 0),
	GATE(mmc1, "mmc1", "aclk200", GATE_BUS_FSYS0, 13, 0, 0),
	GATE(mmc2, "mmc2", "aclk200", GATE_BUS_FSYS0, 14, 0, 0),
	GATE(sromc, "sromc", "aclk200",	GATE_BUS_FSYS0, 19, CLK_IGNORE_UNUSED, 0),


	GATE(smmu_tv, "smmu_tv", "aclk200_disp1", GATE_BUS_DISP1, 9, 0, 0),

	/* UART */
	GATE(uart0, "uart0", "aclk66", GATE_IP_PERIC, 0, 0, 0),
	GATE(uart1, "uart1", "aclk66", GATE_IP_PERIC, 1, 0, 0),
	GATE(uart2, "uart2", "aclk66", GATE_IP_PERIC, 2, 0, 0),
	GATE(uart3, "uart3", "aclk66", GATE_IP_PERIC, 3, 0, 0),

	/* I2C */
	GATE(i2c0, "i2c0", "aclk66", GATE_IP_PERIC, 6, 0, 0),
	GATE(i2c1, "i2c1", "aclk66", GATE_IP_PERIC, 7, 0, 0),
	GATE(i2c2, "i2c2", "aclk66", GATE_IP_PERIC, 8, 0, 0),
	GATE(i2c3, "i2c3", "aclk66", GATE_IP_PERIC, 9, 0, 0),
	GATE(i2c4, "i2c4", "aclk66", GATE_IP_PERIC, 10, 0, 0),
	GATE(i2c5, "i2c5", "aclk66", GATE_IP_PERIC, 11, 0, 0),
	GATE(i2c6, "i2c6", "aclk66", GATE_IP_PERIC, 12, 0, 0),
	GATE(i2c7, "i2c7", "aclk66", GATE_IP_PERIC, 13, 0, 0),
	GATE(i2c_hdmi, "i2c_hdmi", "aclk66", GATE_IP_PERIC, 14, 0, 0),
	
	GATE(tsadc, "tsadc", "aclk66", GATE_IP_PERIC, 15, 0, 0),
	/* SPI */
	GATE(spi0, "spi0", "aclk66", GATE_IP_PERIC, 16, 0, 0),
	GATE(spi1, "spi1", "aclk66", GATE_IP_PERIC, 17, 0, 0),
	GATE(spi2, "spi2", "aclk66", GATE_IP_PERIC, 18, 0, 0),
	GATE(ac97, "ac97", "aclk66", GATE_IP_PERIC, 27, 0, 0),
	
	/* I2S */
	GATE(i2s1, "i2s1", "aclk66", GATE_IP_PERIC, 20, 0, 0),
	GATE(i2s2, "i2s2", "aclk66", GATE_IP_PERIC, 21, 0, 0),
	/* PCM */
	GATE(pcm1, "pcm1", "aclk66", GATE_IP_PERIC, 22, 0, 0),
	GATE(pcm2, "pcm2", "aclk66", GATE_IP_PERIC, 23, 0, 0),
	/* PWM */
	GATE(pwm, "pwm", "aclk66", GATE_IP_PERIC, 24, 0, 0),
	/* SPDIF */
	GATE(spdif, "spdif", "aclk66", GATE_IP_PERIC, 26, 0, 0),


	GATE(ac97, "ac97", "aclk66", GATE_IP_PERIC, 27, 0, 0),
	GATE(chipid, "chipid", "aclk66", GATE_IP_PERIS, 0, CLK_IGNORE_UNUSED, 0),
	GATE(sysreg, "sysreg", "aclk66", GATE_IP_PERIS, 1, 0, 0),

	GATE(tzpc0, "tzpc0", "aclk66", GATE_IP_PERIS, 6, 0, 0),
	GATE(tzpc1, "tzpc1", "aclk66", GATE_IP_PERIS, 7, 0, 0),
	GATE(tzpc2, "tzpc2", "aclk66", GATE_IP_PERIS, 8, 0, 0),
	GATE(tzpc3, "tzpc3", "aclk66", GATE_IP_PERIS, 9, 0, 0),
	GATE(tzpc4, "tzpc4", "aclk66", GATE_IP_PERIS, 10, 0, 0),
	GATE(tzpc5, "tzpc5", "aclk66", GATE_IP_PERIS, 11, 0, 0),
	GATE(tzpc6, "tzpc6", "aclk66", GATE_IP_PERIS, 12, 0, 0),
	GATE(tzpc7, "tzpc7", "aclk66", GATE_IP_PERIS, 13, 0, 0),
	GATE(tzpc8, "tzpc8", "aclk66", GATE_IP_PERIS, 14, 0, 0),
	GATE(tzpc9, "tzpc9", "aclk66", GATE_IP_PERIS, 15, 0, 0),
	GATE(hdmi_cec, "hdmi_cec", "aclk66", GATE_IP_PERIS, 16, 0, 0),
	GATE(seckey, "seckey", "aclk66", GATE_IP_PERIS, 17, 0, 0),
	GATE(mct, "mct", "aclk66", GATE_IP_PERIS, 18, 0, 0),
	GATE(wdt, "wdt", "aclk66", GATE_IP_PERIS, 19, 0, 0),
	GATE(rtc, "rtc", "aclk66", GATE_IP_PERIS, 20, 0, 0),
	GATE(tmu, "tmu", "aclk66", GATE_IP_PERIS, 21, 0, 0),
	GATE(pkey0, "pkey0", "aclk66", GATE_IP_PERIS, 22, 0, 0),
	GATE(pkey1, "pkey1", "aclk66", GATE_IP_PERIS, 23, 0, 0),
	GATE(monocnt, "monocnt", "aclk66", GATE_IP_PERIS, 24, 0, 0),

	GATE(gscl0, "gscl0", "aclk300_gscl", GATE_IP_GSCL0, 0, 0, 0),
	GATE(gscl1, "gscl1", "aclk300_gscl", GATE_IP_GSCL0, 1, 0, 0),
	GATE(clk_3aa, "clk_3aa", "aclk300_gscl", GATE_IP_GSCL0, 4, 0, 0),

	GATE(mfc, "mfc", "aclk333", GATE_IP_MFC, 0, 0, 0),
	GATE(smmu_mfcl, "smmu_mfcl", "mfc", GATE_IP_MFC, 1, 0, 0),
	GATE(smmu_mfcr, "smmu_mfcr", "mfc", GATE_IP_MFC, 2, 0, 0),
	GATE(smmu_3aa, "smmu_3aa", "aclk333_432_gscl", GATE_IP_GSCL1, 2, 0, 0),
	GATE(smmu_fimcl0, "smmu_fimcl0", "aclk333_432_gscl",
			GATE_IP_GSCL1, 3, 0, 0),
	GATE(smmu_fimcl1, "smmu_fimcl1", "aclk333_432_gscl",
			GATE_IP_GSCL1, 4, 0, 0),
	GATE(smmu_gscl0, "smmu_gscl0", "gscl0", GATE_IP_GSCL1, 6, 0, 0),
	GATE(smmu_gscl1, "smmu_gscl1", "gscl1", GATE_IP_GSCL1, 7, 0, 0),
	GATE(gscl_wa, "gscl_wa", "aclk300_gscl", GATE_IP_GSCL1, 12, 0, 0),
	GATE(gscl_wb, "gscl_wb", "aclk300_gscl", GATE_IP_GSCL1, 13, 0, 0),
	GATE(smmu_fimcl3, "smmu_fimcl3,", "aclk333_432_gscl",
			GATE_IP_GSCL1, 16, 0, 0),
	GATE(fimc_lite3, "fimc_lite3", "aclk333_432_gscl",
			GATE_IP_GSCL1, 17, 0, 0),

	GATE(fimd1, "fimd1", "aclk300_disp1", GATE_IP_DISP1, 0, 0, 0),
	GATE(dsim1, "dsim1", "aclk200_disp1", GATE_IP_DISP1, 3, 0, 0),
	GATE(dp1, "dp1", "aclk200_disp1", GATE_IP_DISP1, 4, 0, 0),
	GATE(mixer, "mixer", "aclk66", GATE_IP_DISP1, 5, 0, 0),
	GATE(hdmi, "hdmi", "aclk200_disp1", GATE_IP_DISP1, 6, 0, 0),
	GATE(smmu_fimd1, "smmu_fimd1", "aclk300_disp1", GATE_IP_DISP1, 8, 0, 0),

	GATE(g3d, "g3d", "aclkg3d", GATE_IP_G3D, 9, 0, 0),

	GATE(rotator, "rotator", "aclk266", GATE_IP_GEN, 1, 0, 0),
	GATE(jpeg, "jpeg", "aclk300_jpeg", GATE_IP_GEN, 2, 0, 0),
	GATE(jpeg2, "jpeg2", "aclk300_jpeg", GATE_IP_GEN, 3, 0, 0),
	GATE(mdma1, "mdma1", "aclk266", GATE_IP_GEN, 4, 0, 0),
	GATE(smmu_rotator, "smmu_rotator", "aclk266", GATE_IP_GEN, 6, 0, 0),
	GATE(smmu_jpeg, "smmu_jpeg", "aclk300_jpeg", GATE_IP_GEN, 7, 0, 0),
	GATE(smmu_mdma1, "smmu_mdma1", "aclk266", GATE_IP_GEN, 9, 0, 0),
};

static struct samsung_pll_clock exynos5410_plls[nr_plls] __initdata = {
	[apll] = PLL(pll_35xx, CLK_FOUT_APLL, "fout_apll", "fin_pll", APLL_LOCK,
		APLL_CON0, NULL),
	[cpll] = PLL(pll_35xx, CLK_FOUT_CPLL, "fout_cpll", "fin_pll", CPLL_LOCK,
		CPLL_CON0, NULL),
	[dpll] = PLL(pll_35xx, CLK_FOUT_DPLL, "fout_dpll", "fin_pll", DPLL_LOCK,
		DPLL_CON0, NULL),
	[epll] = PLL(pll_35xx, CLK_FOUT_EPLL, "fout_epll", "fin_pll", EPLL_LOCK,
		EPLL_CON0, NULL),
	[ipll] = PLL(pll_35xx, CLK_FOUT_IPLL, "fout_ipll", "fin_pll", IPLL_LOCK,
		IPLL_CON0, NULL),
	[vpll] = PLL(pll_35xx, CLK_FOUT_VPLL, "fout_vpll", "mout_vpllsrc", VPLL_LOCK,
		VPLL_CON0, NULL),
	[mpll] = PLL(pll_35xx, CLK_FOUT_MPLL, "fout_mpll", "fin_pll", MPLL_LOCK,
		MPLL_CON0, NULL),
	[bpll] = PLL(pll_35xx, CLK_FOUT_BPLL, "fout_bpll", "fin_pll", BPLL_LOCK,
		BPLL_CON0, NULL),
	[kpll] = PLL(pll_35xx, CLK_FOUT_KPLL, "fout_kpll", "fin_pll", KPLL_LOCK,
		KPLL_CON0, NULL),
};


static __initdata struct of_device_id ext_clk_match[] = {
	{ .compatible = "samsung,exynos5410-oscclk", .data = (void *)0, },
	{ },
};

/* register exynos5410 clocks */
void __init exynos5410_clk_init(struct device_node *np)
{
	void __iomem *reg_base;

	if (np) {
		reg_base = of_iomap(np, 0);
		if (!reg_base)
			panic("%s: failed to map registers\n", __func__);
	} else {
		panic("%s: unable to determine soc\n", __func__);
	}

	samsung_clk_init(np, reg_base, nr_clks,
			exynos5410_clk_regs, ARRAY_SIZE(exynos5410_clk_regs),
			NULL, 0);
	samsung_clk_of_register_fixed_ext(exynos5410_fixed_rate_ext_clks,
			ARRAY_SIZE(exynos5410_fixed_rate_ext_clks),
			ext_clk_match);
	samsung_clk_register_mux(exynos5410_pll_mux_clks,
		ARRAY_SIZE(exynos5410_pll_mux_clks));
	samsung_clk_register_pll(exynos5410_plls, ARRAY_SIZE(exynos5410_plls),
		reg_base);

	samsung_clk_register_fixed_rate(exynos5410_fixed_rate_clks,
			ARRAY_SIZE(exynos5410_fixed_rate_clks));
	samsung_clk_register_mux(exynos5410_mux_clks,
			ARRAY_SIZE(exynos5410_mux_clks));
	samsung_clk_register_div(exynos5410_div_clks,
			ARRAY_SIZE(exynos5410_div_clks));
	samsung_clk_register_gate(exynos5410_gate_clks,
			ARRAY_SIZE(exynos5410_gate_clks));
}
CLK_OF_DECLARE(exynos5410_clk, "samsung,exynos5410-clock", exynos5410_clk_init);
