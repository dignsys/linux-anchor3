// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Jongkeun, Choi <jkchoi@nexell.co.kr>
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/clk.h>
#include <linux/log2.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/irq.h>

/*------------------------------------------------------------------------------
 * local data and macro
 */

#define	RTC_TIME_YEAR		(1970)		/* 1970.01.01 00:00:00 */
#define RTC_TIME_MAX		0x69546780	/* 2025.12.31 00:00:00 */
#define RTC_TIME_MIN		0x52c35a80	/* 2014.01.01 00:00:00 */
#define RTC_TIME_DFT		0x4a9c6400	/* 2009.09.01 00:00:00 */

#define RTC_BUSY_CHECK_COUNT	150

#define	RTC_COUNT_BIT		(0)
#define	RTC_ALARM_BIT		(1)

#define RTC_CNT_WRITE		0x00
#define RTC_CNT_READ		0x04
#define RTC_ALARM		0x08
#define RTC_CTRL		0x0C
#define RTC_INT_ENB		0x10
#define RTC_INT_PND		0x14
#define RTC_SCRATCH		0x1C
#define RTC_USERTC		0x24

#define RTC_CNTWAIT             BIT(4)
#define ALARM_CNTWAIT           BIT(3)
#define RTC_CNTWRITEENB         BIT(0)
#define ALARM_INTENB            BIT(1)
#define RTC_INTENB              BIT(0)
#define INTENB_MASK             (ALARM_INTENB | RTC_INTENB)
#define ALARM_INTPEND           BIT(1)
#define RTC_INTPEND             BIT(0)
#define INTPEND_MASK            (ALARM_INTPEND | RTC_INTPEND)

struct nx_rtc {
	struct device *dev;
	struct rtc_device *rtc;
	void __iomem *base;
	spinlock_t rtc_lock;
	unsigned long rtc_time_offs;
	int irq_rtc;
	int rtc_enable_irq;
	int alm_enable_irq;
};

enum use_rtc_type {
	OSCILLATOR_24M = 0,
	RTC_32768HZ,
};

static void nx_rtc_set_interrupt_enable(struct nx_rtc *info, int32_t int_num,
				 bool enable)
{
	u32 regvalue;

	regvalue = readl(info->base + RTC_INT_ENB);
	regvalue &= ~(1ul << int_num);
	regvalue |= (u32)enable << int_num;
	writel(regvalue, info->base + RTC_INT_ENB);
}

static u32 nx_rtc_get_each_interrupt_enable(struct nx_rtc *info,
		int32_t int_num)
{
	return (readl(info->base + RTC_INT_ENB) >> int_num) & 0x01;
}

static u32 nx_rtc_get_interrupt_enable(struct nx_rtc *info)
{
	return (readl(info->base + RTC_INT_ENB) & INTENB_MASK);
}

static u32 nx_rtc_get_interrupt_pending(struct nx_rtc *info, int32_t int_num)
{
	return (readl(info->base + RTC_INT_PND) >> int_num) & 0x01;
}

static void nx_rtc_clear_interrupt_pending(struct nx_rtc *info,
		int32_t int_num)
{
	writel((u32)(1 << int_num),  info->base + RTC_INT_PND);
}

static void nx_rtc_clear_interrupt_pending_all(struct nx_rtc *info)
{
	writel(0x03, info->base + RTC_INT_PND);
}

static void nx_rtc_set_interrupt_enable_all(struct nx_rtc *info, bool enable)
{
	if (enable)
		writel(0x03, info->base + RTC_INT_ENB);
	else
		writel(0x00, info->base + RTC_INT_ENB);
}

static u32 nx_rtc_get_rtc_counter(struct nx_rtc *info)
{
	return readl(info->base + RTC_CNT_READ);
}

static bool nx_rtc_is_busy_rtc_counter(struct nx_rtc *info)
{
	return (readl(info->base + RTC_CTRL) & RTC_CNTWAIT) ? true : false;
}

static int nx_rtc_wait_for_busy(struct nx_rtc *info)
{
	int i;

	for (i = 0; i < RTC_BUSY_CHECK_COUNT; i++) {
		if (!nx_rtc_is_busy_rtc_counter(info))
			return 0;

		udelay(1);
		continue;
	}

	return -EBUSY;
}

static int nx_rtc_set_rtc_counter_write_enable(struct nx_rtc *info,
		bool enable)
{
	u32 regvalue;
	int ret;

	ret = nx_rtc_wait_for_busy(info);
	if (ret)
		return ret;

	regvalue = readl(info->base + RTC_CTRL);
	regvalue &= ~RTC_CNTWRITEENB;
	regvalue |= enable;
	writel(regvalue, info->base + RTC_CTRL);

	return nx_rtc_wait_for_busy(info);
}

static int nx_rtc_set_rtc_counter(struct nx_rtc *info, u32 rtc_counter)
{
	int ret;

	spin_lock_irq(&info->rtc_lock);

	ret = nx_rtc_set_rtc_counter_write_enable(info, true);

	if (!ret)
		writel(rtc_counter, info->base + RTC_CNT_WRITE);
	else
		dev_dbg(info->dev, "%s: RTC is busy and can not be counted.\n",
				__func__);

	nx_rtc_set_rtc_counter_write_enable(info, false);

	spin_unlock_irq(&info->rtc_lock);

	return ret;
}

static u32 nx_rtc_get_alarm_counter(struct nx_rtc *info)
{
	return readl(info->base + RTC_ALARM);
}

static bool nx_rtc_is_busy_alarm_counter(struct nx_rtc *info)
{
	return (readl(info->base + RTC_CTRL) & ALARM_CNTWAIT) ? true : false;
}

static int nx_rtc_wait_for_alarm_busy(struct nx_rtc *info)
{
	int i;

	for (i = 0; i < RTC_BUSY_CHECK_COUNT; i++) {
		if (!nx_rtc_is_busy_alarm_counter(info))
			return 0;

		udelay(1);
		continue;
	}

	return -EBUSY;
}

static void nx_rtc_set_use_rtc(struct nx_rtc *info, enum use_rtc_type value)
{
	writel((value << 0), info->base + RTC_USERTC);
}

static void nx_rtc_setup(struct nx_rtc *info)
{
	unsigned long rtc, curr;
	struct rtc_time rtc_tm;
	int ret;

	dev_dbg(info->dev, "%s\n", __func__);

	nx_rtc_clear_interrupt_pending_all(info);
	nx_rtc_set_interrupt_enable_all(info, false);

	info->rtc_time_offs = mktime(RTC_TIME_YEAR, 1, 1, 0, 0, 0);
	rtc = nx_rtc_get_rtc_counter(info);
	curr = rtc + info->rtc_time_offs;

	if ((curr > RTC_TIME_MAX) || (curr < RTC_TIME_MIN)) {
		/* set to select RTC clock */
		nx_rtc_set_use_rtc(info, RTC_32768HZ);

		/* set hw rtc */
		ret = nx_rtc_set_rtc_counter(info, RTC_TIME_DFT -
				info->rtc_time_offs);

		if (ret) {
			dev_info(info->dev, "%s: RTC is busy.\n", __func__);
			return;
		}
		rtc = nx_rtc_get_rtc_counter(info);
	}

	rtc_time_to_tm(rtc + info->rtc_time_offs, &rtc_tm);

	dev_info(info->dev, "[RTC] day=%04d.%02d.%02d time=%02d:%02d:%02d\n",
		 rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
		 rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec);
}

static int nx_rtc_irq_enable(int alarm, struct device *dev,
			      unsigned int enable)
{
	struct nx_rtc *info = dev_get_drvdata(dev);
	int bit = alarm ? RTC_ALARM_BIT : RTC_COUNT_BIT;

	dev_dbg(info->dev, "%s %s (enb:%d)\n",
		__func__, alarm?"alarm":"count", enable);

	if (enable)
		nx_rtc_set_interrupt_enable(info, bit, true);
	else
		nx_rtc_set_interrupt_enable(info, bit, false);

	if (alarm)
		info->alm_enable_irq = enable;
	else
		info->rtc_enable_irq = enable;

	return 0;
}

static irqreturn_t nx_rtc_interrupt(int irq, void *id)
{
	struct nx_rtc *info = (struct nx_rtc *)id;
	int pend = nx_rtc_get_interrupt_enable(info);

	if (info->rtc_enable_irq && (pend & (1 << RTC_COUNT_BIT))) {
		rtc_update_irq(info->rtc, 1, RTC_PF | RTC_UF | RTC_IRQF);
		nx_rtc_clear_interrupt_pending(info, RTC_COUNT_BIT);
		dev_dbg(info->dev, "RTC Count (PND:0x%x, ENB:%d)\n",
			pend,
			nx_rtc_get_each_interrupt_enable(info, RTC_COUNT_BIT));

		return IRQ_HANDLED;
	}

	if (info->alm_enable_irq && (pend & (1 << RTC_ALARM_BIT))) {
		rtc_update_irq(info->rtc, 1, RTC_AF | RTC_IRQF);
		nx_rtc_clear_interrupt_pending(info, RTC_ALARM_BIT);
		dev_dbg(info->dev, "RTC Alarm (PND:0x%x, ENB:%d)\n",
			pend,
			nx_rtc_get_each_interrupt_enable(info, RTC_ALARM_BIT));

		return IRQ_HANDLED;
	}

	dev_info(info->dev, "RTC Unknown(PND:0x%x,RTC ENB:%d,ALARM ENB:%d)\n",
		 pend, nx_rtc_get_each_interrupt_enable(info, RTC_COUNT_BIT),
		 nx_rtc_get_each_interrupt_enable(info, RTC_ALARM_BIT));

	nx_rtc_clear_interrupt_pending_all(info);

	return IRQ_NONE;
}

/*
 * RTC OPS
 */
static int nx_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct nx_rtc *info = dev_get_drvdata(dev);
	unsigned long rtc;

	spin_lock_irq(&info->rtc_lock);

	rtc = nx_rtc_get_rtc_counter(info);
	rtc_time_to_tm(rtc + info->rtc_time_offs, tm);
	dev_dbg(info->dev, "read time day=%04d.%02d.%02d\n",
			tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday);
	dev_dbg(info->dev, "time=%02d:%02d:%02d\n",
			tm->tm_hour, tm->tm_min, tm->tm_sec);
	dev_dbg(info->dev, "rtc 0x%x\n", (uint)rtc);

	spin_unlock_irq(&info->rtc_lock);

	return 0;
}

static int nx_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct nx_rtc *info = dev_get_drvdata(dev);
	unsigned long rtc, curr_sec;

	curr_sec = mktime(tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
			  tm->tm_hour, tm->tm_min, tm->tm_sec);

	rtc = curr_sec - info->rtc_time_offs;

	dev_dbg(info->dev, "set time day=%02d.%02d.%02d\n",
			tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday);
	dev_dbg(info->dev, "time=%02d:%02d:%02d\n",
			tm->tm_hour, tm->tm_min, tm->tm_sec);
	dev_dbg(info->dev, "rtc 0x%x\n", (uint)rtc);

	/* set hw rtc */
	return nx_rtc_set_rtc_counter(info, rtc);
}

static int nx_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct nx_rtc *info = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	unsigned long count;

	spin_lock_irq(&info->rtc_lock);

	count = nx_rtc_get_alarm_counter(info);

	rtc_time_to_tm(count + info->rtc_time_offs, tm);

	alrm->enabled =
		nx_rtc_get_each_interrupt_enable(info, RTC_ALARM_BIT) ? 1 : 0;
	alrm->pending =
		nx_rtc_get_interrupt_pending(info, RTC_ALARM_BIT) ? 1 : 0;

	dev_dbg(info->dev, "read alarm day=%04d.%02d.%02d time=%02d:%02d:%02d,",
		 tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
		 tm->tm_hour, tm->tm_min, tm->tm_sec);
	dev_dbg(info->dev, "alarm 0x%08x\n", (uint)count);


	spin_unlock_irq(&info->rtc_lock);

	return 0;
}

static int nx_rtc_set_alarm_counter(struct nx_rtc *info, unsigned long count)
{
	int ret;

	/* set hw rtc */
	ret = nx_rtc_wait_for_alarm_busy(info);
	if (ret < 0)
		return -EBUSY;

	writel(count, info->base + RTC_ALARM);

	/* Confirm the write value. */
	return nx_rtc_wait_for_alarm_busy(info);
}

static int nx_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct nx_rtc *info = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	unsigned long count, seconds;
	int ret;

	spin_lock_irq(&info->rtc_lock);

	seconds = mktime(tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
				tm->tm_hour, tm->tm_min, tm->tm_sec);

	count = seconds - info->rtc_time_offs;

	dev_dbg(info->dev, "set alarm day=%04d.%02d.%02d time=%02d:%02d:%02d, ",
		 tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
		 tm->tm_hour, tm->tm_min, tm->tm_sec);
	dev_dbg(info->dev, "alarm %lu, %s\n", count, alrm->enabled?"ON":"OFF");

	ret = nx_rtc_set_alarm_counter(info, count);

	nx_rtc_clear_interrupt_pending(info, RTC_ALARM_BIT);

	/* 0: RTC Counter, 1: RTC Alarm */
	nx_rtc_set_interrupt_enable(info, RTC_ALARM_BIT,
				  alrm->enabled ? true : false);

	info->alm_enable_irq = alrm->enabled;

	spin_unlock_irq(&info->rtc_lock);

	return ret;
}

static int nx_rtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	struct nx_rtc *info = dev_get_drvdata(dev);

	dev_dbg(info->dev, "%s (enb:%d)\n", __func__, enable);
	return nx_rtc_irq_enable(1, dev, enable);
}

/*
 * Provide additional RTC information in /proc/driver/rtc
 */
static const struct rtc_class_ops nx_rtc_ops = {
	.read_time		= nx_rtc_read_time,
	.set_time		= nx_rtc_set_time,
	.read_alarm		= nx_rtc_read_alarm,
	.set_alarm		= nx_rtc_set_alarm,
	.alarm_irq_enable	= nx_rtc_alarm_irq_enable,
};

/*
 * RTC platform driver functions
 */
static int nx_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct nx_rtc *info = platform_get_drvdata(pdev);

	dev_dbg(info->dev, "+%s (rtc irq:%s, alarm irq:%s, wakeup=%d)\n",
		__func__, info->rtc_enable_irq?"on":"off",
		info->alm_enable_irq?"on":"off", device_may_wakeup(&pdev->dev));

	if (info->rtc_enable_irq) {
		nx_rtc_clear_interrupt_pending(info, RTC_COUNT_BIT);
		nx_rtc_set_interrupt_enable(info, RTC_COUNT_BIT, false);
	}

	if (info->alm_enable_irq) {
		unsigned long count = nx_rtc_get_alarm_counter(info);
		struct rtc_time tm;

		rtc_time_to_tm(count, &tm);
		dev_dbg(info->dev, "%s (alarm day=%04d.%02d.%02d time=%02d:%02d:%02d, ",
			__func__, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec);
		dev_dbg(info->dev, "alarm %lu)\n", count);
	}

	dev_dbg(info->dev, "-%s\n", __func__);
	return 0;
}

static int nx_rtc_resume(struct platform_device *pdev)
{
	struct nx_rtc *info = platform_get_drvdata(pdev);

	dev_dbg(info->dev, "+%s (rtc irq:%s, alarm irq:%s)\n",
		__func__, info->rtc_enable_irq?"on":"off",
		info->alm_enable_irq?"on":"off");

	if (info->rtc_enable_irq) {
		nx_rtc_clear_interrupt_pending(info, RTC_COUNT_BIT);
		nx_rtc_set_interrupt_enable(info, RTC_COUNT_BIT, true);
	}
	dev_dbg(info->dev, "-%s\n", __func__);
	return 0;
}

static int nx_rtc_probe(struct platform_device *pdev)
{
	struct nx_rtc *info = NULL;
	struct resource *res;
	int ret;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	/* find the IRQs */
	info->irq_rtc = platform_get_irq(pdev, 0);
	if (info->irq_rtc < 0) {
		dev_err(&pdev->dev, "no irq for rtc\n");
		return info->irq_rtc;
	}

	info->dev = &pdev->dev;
	spin_lock_init(&info->rtc_lock);

	platform_set_drvdata(pdev, info);

	dev_dbg(info->dev, "%s: rtc irq %d\n", __func__, info->irq_rtc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(info->base))
		return PTR_ERR(info->base);

	nx_rtc_setup(info);

	/* cpu init code should really have flagged this device as
	 * being wake-capable; if it didn't, do that here.
	 */
	if (!device_can_wakeup(&pdev->dev))
		device_init_wakeup(&pdev->dev, 1);

	/* register RTC and exit */
	info->rtc = devm_rtc_device_register(&pdev->dev, "nx", &nx_rtc_ops,
				  THIS_MODULE);
	if (IS_ERR(info->rtc)) {
		dev_err(&pdev->dev, "cannot attach rtc\n");
		ret = PTR_ERR(info->rtc);
		return ret;
	}

	/* register disabled irq */
	ret = devm_request_irq(&pdev->dev, info->irq_rtc,
					 nx_rtc_interrupt, 0, "rtc 1hz", info);
	if (ret) {
		dev_err(&pdev->dev, "IRQ%d error %d\n", info->irq_rtc, ret);
		return ret;
	}

	/* set rtc frequency value */
	info->rtc->irq_freq	   = 1;
	info->rtc->max_user_freq = 1;

	dev_dbg(info->dev, "done: rtc probe ...\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id nx_rtc_dt_match[] = {
	{ .compatible = "nexell,nx-rtc"},
	{ },
};
MODULE_DEVICE_TABLE(of, nx_rtc_dt_match);
#else
#define nx_rtc_dt_match NULL
#endif

static struct platform_driver nx_rtc_driver = {
	.probe		= nx_rtc_probe,
	.suspend	= nx_rtc_suspend,
	.resume		= nx_rtc_resume,
	.driver		= {
		.name	= "nx-rtc",
		.of_match_table	= of_match_ptr(nx_rtc_dt_match),
	},
};
module_platform_driver(nx_rtc_driver);

MODULE_DESCRIPTION("RTC driver for the Nexell");
MODULE_AUTHOR("JongKeun Choi<jkchoi@nexell.co.kr>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:nexell-rtc");
