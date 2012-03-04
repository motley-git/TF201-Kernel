/*
 * arch/arm/mach-tegra/board-cardhu-kbc.c
 * Keys configuration for Nvidia tegra3 cardhu platform.
 *
 * Copyright (C) 2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/tps6591x.h>
#include <linux/mfd/max77663-core.h>
#include <linux/interrupt_keys.h>
#include <linux/gpio_scrollwheel.h>

#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include <mach/board-cardhu-misc.h>
#include "board.h"
#include "board-cardhu.h"

#include "gpio-names.h"
#include "devices.h"

#define CARDHU_PM269_ROW_COUNT	1
#define CARDHU_PM269_COL_COUNT	4

static const u32 kbd_keymap[] = {
	KEY(0, 0, KEY_RESERVED),
	KEY(0, 1, KEY_RESERVED),
	KEY(0, 2, KEY_VOLUMEUP),
	KEY(0, 3, KEY_VOLUMEDOWN),
};

static const struct matrix_keymap_data keymap_data = {
	.keymap	 = kbd_keymap,
	.keymap_size    = ARRAY_SIZE(kbd_keymap),
};

static struct tegra_kbc_wake_key cardhu_wake_cfg[] = {
	[0] = {
		.row = 0,
		.col = 0,
	},
};

static struct tegra_kbc_platform_data cardhu_kbc_platform_data = {
	.debounce_cnt = 20,
	.repeat_cnt = 1,
	.scan_count = 30,
	.wakeup = false,
	.keymap_data = &keymap_data,
	.wake_cnt = 0,
};

int __init cardhu_kbc_init(void)
{
	struct tegra_kbc_platform_data *data = &cardhu_kbc_platform_data;
	int i;
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_E1198) ||
			(board_info.board_id == BOARD_E1291))
		return 0;

	pr_info("Registering tegra-kbc\n");
	tegra_kbc_device.dev.platform_data = &cardhu_kbc_platform_data;

	for (i = 0; i < CARDHU_PM269_ROW_COUNT; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].is_row = true;
		data->pin_cfg[i].en = true;
	}
	for (i = 0; i < CARDHU_PM269_COL_COUNT; i++) {
		/*
		 * Avoid keypad scan (ROW0,COL0) and (ROW0, COL1)
		 * KBC-COL1 (GPIO-Q-01) is used for Pad EC request#
		 * KBC-COL0(GPIO-Q-00) and AP_ONKEY#(GPIO-P-00) are
		 * both wired with power button, but we configure
		 * AP_ONKEY pin for power button instead.
		 */
		if (i <= 1) continue;
		data->pin_cfg[i + KBC_PIN_GPIO_16].num = i;
		data->pin_cfg[i + KBC_PIN_GPIO_16].en = true;
	}

	platform_device_register(&tegra_kbc_device);
	return 0;
}

int __init cardhu_scroll_init(void)
{
	return 0;
}

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button cardhu_keys_e1198[] = {
	[0] = GPIO_KEY(KEY_HOME, PQ0, 0),
	[1] = GPIO_KEY(KEY_BACK, PQ1, 0),
	[2] = GPIO_KEY(KEY_MENU, PQ2, 0),
	[3] = GPIO_KEY(KEY_SEARCH, PQ3, 0),
	[4] = GPIO_KEY(KEY_VOLUMEUP, PR0, 0),
	[5] = GPIO_KEY(KEY_VOLUMEDOWN, PR1, 0),
	[6] = GPIO_KEY(KEY_POWER, PV0, 1),
};

static struct gpio_keys_platform_data cardhu_keys_e1198_platform_data = {
	.buttons	= cardhu_keys_e1198,
	.nbuttons	= ARRAY_SIZE(cardhu_keys_e1198),
};

static struct platform_device cardhu_keys_e1198_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &cardhu_keys_e1198_platform_data,
	},
};

static struct gpio_keys_button cardhu_keys_e1291[] = {
	[0] = GPIO_KEY(KEY_MENU, PR0, 0),
	[1] = GPIO_KEY(KEY_BACK, PR1, 0),
	[2] = GPIO_KEY(KEY_HOME, PR2, 0),
	[3] = GPIO_KEY(KEY_SEARCH, PQ3, 0),
	[4] = GPIO_KEY(KEY_VOLUMEUP, PQ0, 0),
	[5] = GPIO_KEY(KEY_VOLUMEDOWN, PQ1, 0),
};

static struct gpio_keys_button cardhu_keys_e1291_a04[] = {
	[0] = GPIO_KEY(KEY_MENU, PR0, 0),
	[1] = GPIO_KEY(KEY_BACK, PR1, 0),
	[2] = GPIO_KEY(KEY_HOME, PQ2, 0),
	[3] = GPIO_KEY(KEY_SEARCH, PQ3, 0),
	[4] = GPIO_KEY(KEY_VOLUMEUP, PQ0, 0),
	[5] = GPIO_KEY(KEY_VOLUMEDOWN, PQ1, 0),
};

static struct gpio_keys_platform_data cardhu_keys_e1291_platform_data = {
	.buttons	= cardhu_keys_e1291,
	.nbuttons	= ARRAY_SIZE(cardhu_keys_e1291),
};

static struct platform_device cardhu_keys_e1291_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &cardhu_keys_e1291_platform_data,
	},
};

static struct gpio_keys_button cardhu_keys_tf201[] = {
	[0] = GPIO_KEY(KEY_POWER, PV0, 1),
};

static struct gpio_keys_platform_data cardhu_keys_tf201_platform_data = {
	.buttons	= cardhu_keys_tf201,
	.nbuttons	= ARRAY_SIZE(cardhu_keys_tf201),
};

static struct platform_device cardhu_keys_tf201_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &cardhu_keys_tf201_platform_data,
	},
};
#define INT_KEY(_id, _irq, _iswake, _deb_int)	\
	{					\
		.code = _id,			\
		.irq = _irq,			\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = _deb_int,	\
	}
static struct interrupt_keys_button cardhu_int_keys[] = {
	[0] = INT_KEY(KEY_POWER, TPS6591X_IRQ_BASE + TPS6591X_INT_PWRON, 0, 100),
};

static struct interrupt_keys_button cardhu_pm298_int_keys[] = {
	[0] = INT_KEY(KEY_POWER, MAX77663_IRQ_BASE + MAX77663_IRQ_ONOFF_EN0_FALLING, 0, 100),
	[1] = INT_KEY(KEY_POWER, MAX77663_IRQ_BASE + MAX77663_IRQ_ONOFF_EN0_1SEC, 0, 3000),
};

static struct interrupt_keys_button cardhu_pm299_int_keys[] = {
	[0] = INT_KEY(KEY_POWER, RICOH583_IRQ_BASE + RICOH583_IRQ_ONKEY, 0, 100),
};

static struct interrupt_keys_platform_data cardhu_int_keys_pdata = {
	.int_buttons	= cardhu_int_keys,
	.nbuttons       = ARRAY_SIZE(cardhu_int_keys),
};

static struct platform_device cardhu_int_keys_device = {
	.name   = "interrupt-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &cardhu_int_keys_pdata,
	},
};

int __init cardhu_keys_init(void)
{
	int i;
	struct board_info board_info;
	struct board_info pmu_board_info;

	tegra_get_board_info(&board_info);
	if (!((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291) ||
		(board_info.board_id == BOARD_E1186) ||
		(board_info.board_id == BOARD_E1257) ||
		(board_info.board_id == BOARD_PM305) ||
		(board_info.board_id == BOARD_PM311) ||
		(board_info.board_id == BOARD_PM269)))
		return 0;

	pr_info("Registering gpio keys\n");

	if (board_info.board_id == BOARD_E1291) {
		if (board_info.fab >= BOARD_FAB_A04) {
			cardhu_keys_e1291_platform_data.buttons =
					cardhu_keys_e1291_a04;
			cardhu_keys_e1291_platform_data.nbuttons =
					ARRAY_SIZE(cardhu_keys_e1291_a04);
		}

		/* Enable gpio mode for other pins */
		for (i = 0; i < cardhu_keys_e1291_platform_data.nbuttons; i++)
			tegra_gpio_enable(cardhu_keys_e1291_platform_data.
						buttons[i].gpio);

		platform_device_register(&cardhu_keys_e1291_device);
	} else if (board_info.board_id == BOARD_E1198) {
		/* For E1198 */
		for (i = 0; i < ARRAY_SIZE(cardhu_keys_e1198); i++)
			tegra_gpio_enable(cardhu_keys_e1198[i].gpio);

		platform_device_register(&cardhu_keys_e1198_device);
	} else if ((board_info.board_id == BOARD_PM269) &&
		(!strcmp(tegra3_get_project_name(), "TF201") ||
		!strcmp(tegra3_get_project_name(), "TF200X") ||
		!strcmp(tegra3_get_project_name(), "TF200XG") ||
		!strcmp(tegra3_get_project_name(), "TF202T") ||
		!strcmp(tegra3_get_project_name(), "TF200")))
	{
		/* For PM269-alike TF201/TF200X/TF200XG/TF202T/TF200 products */
		for (i = 0; i < ARRAY_SIZE(cardhu_keys_tf201); i++)
			tegra_gpio_enable(cardhu_keys_tf201[i].gpio);

		platform_device_register(&cardhu_keys_tf201_device);
	}

	/* Register on-key through pmu interrupt */
	tegra_get_pmu_board_info(&pmu_board_info);

	if (pmu_board_info.board_id == BOARD_PMU_PM298) {
		cardhu_int_keys_pdata.int_buttons = cardhu_pm298_int_keys;
		cardhu_int_keys_pdata.nbuttons =
					ARRAY_SIZE(cardhu_pm298_int_keys);
	}

	if (pmu_board_info.board_id == BOARD_PMU_PM299) {
		cardhu_int_keys_pdata.int_buttons = cardhu_pm299_int_keys;
		cardhu_int_keys_pdata.nbuttons =
					ARRAY_SIZE(cardhu_pm299_int_keys);
	}

	if ((board_info.board_id == BOARD_E1291) ||
		(board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1257) ||
		(board_info.board_id == BOARD_E1186) ||
		(board_info.board_id == BOARD_PM305) ||
		(board_info.board_id == BOARD_PM311))
		platform_device_register(&cardhu_int_keys_device);
	return 0;
}
