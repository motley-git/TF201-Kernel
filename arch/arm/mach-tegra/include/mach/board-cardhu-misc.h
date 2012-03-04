/*
 * arch/arm/mach-tegra/include/mach/board-cardhu-misc.h
 *
 * Copyright (C) 2011-2012 ASUSTek Computer Incorporation
 * Author: Paris Yeh <paris_yeh@asus.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* The cardhu_pcbid is hexadecimal representation as follows.
 *
 *	PCB_ID[1:0] is KB_ROW[5:4], and
 *	PCB_ID[2] is KB_COL[4], and
 *	PCB_ID[3] is KB_COL[7], and
 *	PCB_ID[4] is KB_ROW[2], and
 *	PCB_ID[5] is KB_COL[5], and
 *	PCB_ID[6] is GMI_CS0_N, and
 *	PCB_ID[7] is GMI_CS1_N, and
 *	PCB_ID[8] is GMI_CS2_N,
 *
 *		Project ID
 *	=======================================
 *	PCB_ID[5] PCB_ID[4] PCB_ID[3]	Project
 *	0	  0	    0		TF201
 *	0	  0	    1		ME271
 *	0	  1	    0		TF200X
 *	0	  1	    1		TF200XG
 *	1	  0	    0		TF202T
 *	1	  1	    1		TF200
 *	=======================================
 *
 * The cardhu_projectid format should be like as follows
 *
 *	PROJECT[:INFO]*
 *
 *	where
 *		PROJECT field is mandatory.
 *		[:INFO]* field is optional, seperated by a delimiter ':',
 *			and allows to repeat at least 0 times
 */
#ifndef ASUS_TEGRA_DEVKIT_MISC_HW_H
#define ASUS_TEGRA_DEVKIT_MISC_HW_H

#if defined(__cplusplus)
extern "C"
{
#endif
/*
 * The HW_FIELD_* macros are helper macros for the public HW_DRF_* macros.
 */
#define HW_FIELD_LOWBIT(x)      (0?x)
#define HW_FIELD_HIGHBIT(x)     (1?x)
#define HW_FIELD_SIZE(x)        (HW_FIELD_HIGHBIT(x)-HW_FIELD_LOWBIT(x)+1)
#define HW_FIELD_SHIFT(x)       ((0?x)%32)
#define HW_FIELD_MASK(x)        (0xFFFFFFFFUL>>(31-((1?x)%32)+((0?x)%32)))
#define HW_FIELD_BITS(val, x)   (((val) & HW_FIELD_MASK(x))<<HW_FIELD_SHIFT(x))
#define HW_FIELD_SHIFTMASK(x)   (HW_FIELD_MASK(x)<< (HW_FIELD_SHIFT(x)))

/** HW_DRF_VAL - read a field from a register.

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param v register value
 */
#define HW_DRF_VAL(d,r,f,v) \
    (((v)>> HW_FIELD_SHIFT(d##_##r##_0_##f##_RANGE)) & \
        HW_FIELD_MASK(d##_##r##_0_##f##_RANGE))

/** HW_DRF_SIZE - read a occupied length from a register.

    @param d register domain (hardware block)
    @param r register name
    @param f register field
 */
#define HW_DRF_SIZE(d,r,f) \
    (HW_FIELD_SIZE(d##_##r##_0_##f##_RANGE))

#define TEGRA3_DEVKIT_MISC_PCBID_NUM	9

/* WIFI SKU identifications */
#define TEGRA3_DEVKIT_MISC_HW_0_WIFI_RANGE	1:0
#define TEGRA3_DEVKIT_MISC_HW_0_WIFI_DEFAULT	0x0UL //NH660
#define TEGRA3_DEVKIT_MISC_HW_0_WIFI_1		0x1UL //NH615
#define TEGRA3_DEVKIT_MISC_HW_0_WIFI_2		0x2UL //NH669/NH665
#define TEGRA3_DEVKIT_MISC_HW_0_WIFI_3		0x3UL //LBEH19UQJC

/* GPS SKU identifications on TF200X, and TF200XG */
#define TEGRA3_DEVKIT_MISC_HW_0_GPS_RANGE	2:2
#define TEGRA3_DEVKIT_MISC_HW_0_GPS_DEFAULT	0x0UL //BCM47511
#define TEGRA3_DEVKIT_MISC_HW_0_GPS_1		0x1UL //BCM4751

/* Touch panel SKU identifications on TF201, and TF202T */
#define TEGRA3_DEVKIT_MISC_HW_0_TOUCHL_RANGE	2:2
#define TEGRA3_DEVKIT_MISC_HW_0_TOUCHL_DEFAULT	0x0UL
#define TEGRA3_DEVKIT_MISC_HW_0_TOUCHL_1	0x1UL
#define TEGRA3_DEVKIT_MISC_HW_0_TOUCHH_RANGE	6:6
#define TEGRA3_DEVKIT_MISC_HW_0_TOUCHH_DEFAULT	0x0UL
#define TEGRA3_DEVKIT_MISC_HW_0_TOUCHH_1	0x1UL

/* Project identifications on Tegra3 platform */
#define TEGRA3_DEVKIT_MISC_HW_0_PROJECT_RANGE	5:3
#define TEGRA3_DEVKIT_MISC_HW_0_PROJECT_DEFAULT	0x0UL //TF201
#define TEGRA3_DEVKIT_MISC_HW_0_PROJECT_1	0x1UL //ME271
#define TEGRA3_DEVKIT_MISC_HW_0_PROJECT_2	0x2UL //TF200X
#define TEGRA3_DEVKIT_MISC_HW_0_PROJECT_3	0x3UL //TF200XG
#define TEGRA3_DEVKIT_MISC_HW_0_PROJECT_4	0x4UL //TF202T
#define TEGRA3_DEVKIT_MISC_HW_0_PROJECT_5	0x5UL //Reserve
#define TEGRA3_DEVKIT_MISC_HW_0_PROJECT_6	0x6UL //Reserve
#define TEGRA3_DEVKIT_MISC_HW_0_PROJECT_7	0x7UL //TF200

/* Declare maximum length of project identification for strncmp() */
#define TEGRA3_PROJECT_NAME_MAX_LEN	16

/* Audio Codec SKU identifications on TF200X, and TF200XG */
#define TEGRA3_DEVKIT_MISC_HW_0_ACODEC_RANGE	7:6
#define TEGRA3_DEVKIT_MISC_HW_0_ACODEC_DEFAULT	0x0UL //ALC5631Q
#define TEGRA3_DEVKIT_MISC_HW_0_ACODEC_1	0x1UL //WM8903
#define TEGRA3_DEVKIT_MISC_HW_0_ACODEC_2	0x2UL //ALC5642
#define TEGRA3_DEVKIT_MISC_HW_0_ACODEC_3	0x3UL //Reserve

/* MP identifications */
#define TEGRA3_DEVKIT_MISC_HW_0_MP_RANGE	8:8
#define TEGRA3_DEVKIT_MISC_HW_0_MP_DEFAULT	0x0UL //Engineering
#define TEGRA3_DEVKIT_MISC_HW_0_MP_1		0x1UL //Mass Production

extern unsigned char cardhu_chipid[17];

enum tegra3_project {
	TEGRA3_PROJECT_TF201 = 0,
	TEGRA3_PROJECT_ME271,
	TEGRA3_PROJECT_TF200X,
	TEGRA3_PROJECT_TF200XG,
	TEGRA3_PROJECT_TF202T,
	TEGRA3_PROJECT_ReserveA,
	TEGRA3_PROJECT_ReserveB,
	TEGRA3_PROJECT_TF200,
	TEGRA3_PROJECT_MAX,
};

int __init cardhu_misc_init(void);

/* Acquire project identification
 *   @ret cont char *
 *      Project identification will be returned.
 */
const char *tegra3_get_project_name(void);

/* Query pin status of equipped touch module defined in PCB pins.
 *   @ret u32
 *      Return unsigned integer to reflect the PCB pin status of equipped touch module.
 *      Otherwise -1 (Not supported) will be returned.
 */
unsigned int tegra3_query_touch_module_pcbid(void);

/* Query pin status of equipped input audio codec defined in PCB pins.
 *   @ret bool
 *      Return unsigned integer to reflect the PCB pin status of equipped audio codec.
 *      Otherwise -1 (Not supported) will be returned.
 */
unsigned int tegra3_query_audio_codec_pcbid(void);

/* Query pin status of equipped wifi module defined in PCB pins.
 *   @ret bool
 *      Return unsigned integer to reflect the PCB pin status of equipped wifi module.
 *      Otherwise -1 (Not supported) will be returned.
 */
unsigned int tegra3_query_wifi_module_pcbid(void);

#if defined(__cplusplus)
}
#endif

#endif

