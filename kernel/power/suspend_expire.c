/*
 * kernel/power/suspend_expire.c - Suspend to RAM and timeout constraints.
 *
 * Copyright (c) 2011 Paris Yeh <paris_yeh@asus.com>
 *
 * This file is released under the GPLv2.
 */

#include <linux/timer.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>

#include "power.h"
/* We limit the freezer code by setting an hardware watchdog a short
 * time in the furtue, then freezing. Freezing the tasks won't normaly
 * take long ... only need a few milliseconds.
 */
#define EXPIRE_FREEZER_SECONDS	40
/*
 * We limit the system suspend code by setting an kernel timer a short
 * time in the future, then suspending.  Suspending the devices won't
 * normally take long ... some systems only need a few milliseconds.
 */
#define EXPIRE_SUSPEND_SECONDS	10
/*
 * Additional timeout were considered for hardware watchdog setup
 */
#define EXPIRE_WDT_SECONDS	5

struct tegra_mini_wdt {
	struct resource         *res_src;
	struct resource         *res_wdt;
	void __iomem            *wdt_source;
	void __iomem            *wdt_timer;
	int                     timeout;
	bool			enabled;
};

static struct platform_device *tegra_wdt_dev;

#if defined(CONFIG_ARCH_TEGRA_3x_SOC)
#define TIMER_PTV                       0
 #define TIMER_EN                       (1 << 31)
 #define TIMER_PERIODIC                 (1 << 30)
#define TIMER_PCR                       0x4
 #define TIMER_PCR_INTR                 (1 << 30)
#define WDT_CFG                         (0)
 #define WDT_CFG_TMR_SRC                (0 << 0) /* for TMR10. */
 #define WDT_CFG_PERIOD                 (1 << 4)
 #define WDT_CFG_INT_EN                 (1 << 12)
 #define WDT_CFG_SYS_RST_EN             (1 << 14)
 #define WDT_CFG_PMC2CAR_RST_EN         (1 << 15)
#define WDT_CMD                         (8)
 #define WDT_CMD_START_COUNTER          (1 << 0)
 #define WDT_CMD_DISABLE_COUNTER        (1 << 1)
#define WDT_UNLOCK                      (0xC)
 #define WDT_UNLOCK_PATTERN             (0xC45A << 0)

static void tegra_wdt_enable(struct tegra_mini_wdt *wdt)
{
	u32 val;

	val = (wdt->timeout * 1000000ul) / 4;
	val |= (TIMER_EN | TIMER_PERIODIC);
	writel(val, wdt->wdt_timer + TIMER_PTV);

	val = WDT_CFG_TMR_SRC | WDT_CFG_PERIOD | WDT_CFG_INT_EN |
                /*WDT_CFG_SYS_RST_EN |*/ WDT_CFG_PMC2CAR_RST_EN;
	writel(val, wdt->wdt_source + WDT_CFG);
	writel(WDT_CMD_START_COUNTER, wdt->wdt_source + WDT_CMD);
}

static void tegra_wdt_disable(struct tegra_mini_wdt *wdt)
{
	writel(WDT_UNLOCK_PATTERN, wdt->wdt_source + WDT_UNLOCK);
	writel(WDT_CMD_DISABLE_COUNTER, wdt->wdt_source + WDT_CMD);

	writel(0, wdt->wdt_timer + TIMER_PTV);
}
#endif

static void expire_suspend(unsigned long data)
{
	/* Warning on suspend means the kernel timer period needs to be
	 * larger -- the system was sooo slooowwww to suspend that the
	 * timer (should have) exipred before the system went to sleep!
	 *
	 * Warning on either suspend or resume also means the system
	 * has some performance issues, or even meets system hang.
	 * The stack dump of a WARN_ON is more likely to get the right
	 * attention than a printk...
	 */
	WARN_ON(1);
}

static DEFINE_TIMER(expire_timer, expire_suspend, 0, 0);

void freezer_expire_start(void)
{
	if (tegra_wdt_dev != NULL) {
		struct tegra_mini_wdt *wdt = platform_get_drvdata(tegra_wdt_dev);
		if (wdt->enabled) {
			pr_info("PM: stop enabled watchdog");
			tegra_wdt_disable(wdt);
			wdt->enabled = false;
		}
		wdt->timeout = EXPIRE_FREEZER_SECONDS + EXPIRE_WDT_SECONDS;
		wdt->enabled = true;
		pr_info("PM: start timeout watchdog %d seconds\n",
			wdt->timeout);
		tegra_wdt_enable(wdt);
	}
}

void freezer_expire_finish(const char *label)
{
	if (tegra_wdt_dev != NULL) {
		struct tegra_mini_wdt *wdt = platform_get_drvdata(tegra_wdt_dev);
		pr_info("PM: %s stop timeout watchdog", label);
		tegra_wdt_disable(wdt);
		wdt->enabled = false;
	}
}

void suspend_expire_start(void)
{
	/* FIXME Use better timebase than "jiffies", ideally a clocksource.
	 * What we want is a hardware counter that will work correctly even
	 * during the irqs-are-off stages of the suspend/resume cycle...
	 */
	pr_info("PM: start expire timer %d seconds\n", EXPIRE_SUSPEND_SECONDS);
		mod_timer(&expire_timer, jiffies + EXPIRE_SUSPEND_SECONDS * HZ);

	if (tegra_wdt_dev != NULL) {
		struct tegra_mini_wdt *wdt = platform_get_drvdata(tegra_wdt_dev);
		if (wdt->enabled) {
			pr_info("PM: stop enabled watchdog");
			tegra_wdt_disable(wdt);
			wdt->enabled = false;
		}
		wdt->timeout = EXPIRE_SUSPEND_SECONDS + EXPIRE_WDT_SECONDS;
		wdt->enabled = true;
		pr_info("PM: start timeout watchdog %d seconds\n",
			wdt->timeout);
		tegra_wdt_enable(wdt);
	}
}

void suspend_expire_finish(const char *label)
{
	if (tegra_wdt_dev != NULL) {
		struct tegra_mini_wdt *wdt = platform_get_drvdata(tegra_wdt_dev);
		pr_info("PM: %s stop timeout watchdog", label);
		tegra_wdt_disable(wdt);
		wdt->enabled = false;
	}

	if (del_timer(&expire_timer))
		pr_info("PM: %s stop expire timer\n", label);
}

static int tegra_mini_wdt_probe(struct platform_device *pdev)
{
        struct resource *res_src, *res_wdt;
        struct tegra_mini_wdt *wdt;
        int ret = 0;

        if (pdev->id != -1) {
                dev_err(&pdev->dev, "only id -1 supported\n");
                return -ENODEV;
        }

        if (tegra_wdt_dev != NULL) {
                dev_err(&pdev->dev, "watchdog already registered\n");
                return -EIO;
        }

        res_src = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        res_wdt = platform_get_resource(pdev, IORESOURCE_MEM, 1);

        if (!res_src || !res_wdt ) {
                dev_err(&pdev->dev, "incorrect resources\n");
                return -ENOENT;
        }

        wdt = kzalloc(sizeof(*wdt), GFP_KERNEL);
        if (!wdt) {
                dev_err(&pdev->dev, "out of memory\n");
                return -ENOMEM;
        }

        res_src = request_mem_region(res_src->start, resource_size(res_src),
                                     pdev->name);
        res_wdt = request_mem_region(res_wdt->start, resource_size(res_wdt),
                                     pdev->name);

        if (!res_src || !res_wdt) {
                dev_err(&pdev->dev, "unable to request memory resources\n");
                ret = -EBUSY;
                goto fail;
        }

        wdt->wdt_source = ioremap(res_src->start, resource_size(res_src));
        wdt->wdt_timer = ioremap(res_wdt->start, resource_size(res_wdt));
        if (!wdt->wdt_source || !wdt->wdt_timer) {
                dev_err(&pdev->dev, "unable to map registers\n");
                ret = -ENOMEM;
                goto fail;
        }

        wdt->res_src = res_src;
        wdt->res_wdt = res_wdt;

        platform_set_drvdata(pdev, wdt);
        tegra_wdt_dev = pdev;

        return 0;
fail:
        if (wdt->wdt_source)
		iounmap(wdt->wdt_source);
        if (wdt->wdt_timer)
                iounmap(wdt->wdt_timer);
        if (res_src)
                release_mem_region(res_src->start, resource_size(res_src));
        if (res_wdt)
                release_mem_region(res_wdt->start, resource_size(res_wdt));
        kfree(wdt);
        return ret;
}

static int tegra_mini_wdt_remove(struct platform_device *pdev)
{
        struct tegra_mini_wdt *wdt = platform_get_drvdata(pdev);

        tegra_wdt_disable(wdt);

        iounmap(wdt->wdt_source);
        iounmap(wdt->wdt_timer);
        release_mem_region(wdt->res_src->start, resource_size(wdt->res_src));
        release_mem_region(wdt->res_wdt->start, resource_size(wdt->res_wdt));
        kfree(wdt);
        platform_set_drvdata(pdev, NULL);
        return 0;
}

static struct platform_driver tegra_mini_wdt_driver = {
        .probe          = tegra_mini_wdt_probe,
        .remove         = __devexit_p(tegra_mini_wdt_remove),
        .driver         = {
                .owner  = THIS_MODULE,
                .name   = "tegra_wdt",
        },

};

static int __init tegra_mini_wdt_init(void)
{
	return platform_driver_register(&tegra_mini_wdt_driver);

}

static void __exit tegra_mini_wdt_exit(void)
{
	platform_driver_unregister(&tegra_mini_wdt_driver);
}

module_init(tegra_mini_wdt_init);
module_exit(tegra_mini_wdt_exit);

MODULE_AUTHOR("ASUS Corporation");
MODULE_DESCRIPTION("Tegra Mini Watchdog Driver");
