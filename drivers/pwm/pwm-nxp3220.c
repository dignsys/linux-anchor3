// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>

#define PWM_NUM				4

#define REG_TCFG0(ch)			(0x100 * (ch) + 0x00)
#define REG_TCFG1(ch)			(0x100 * (ch) + 0x04)
#define REG_TCON(ch)			(0x100 * (ch) + 0x08)
#define REG_TCNTB(ch)			(0x100 * (ch) + 0x0c)
#define REG_TCMPB(ch)			(0x100 * (ch) + 0x10)
#define REG_TINTR(ch)			(0x100 * (ch) + 0x18)

#define TCFG0_PRESCALER_MASK		0xff
#define TCFG1_MUX_MASK			0x7

#define TCON_START			BIT(0)
#define TCON_MANUALUPDATE		BIT(1)
#define TCON_INVERT			BIT(2)
#define TCON_AUTORELOAD			BIT(3)

/**
 * struct nexell_pwm_channel - private data of PWM channel
 * @period_ns:	current period in nanoseconds programmed to the hardware
 * @duty_ns:	current duty time in nanoseconds programmed to the hardware
 * @tin_ns:	time of one timer tick in nanoseconds with current timer rate
 */
struct nexell_pwm_channel {
	u32 period_ns;
	u32 duty_ns;
	u32 tin_ns;
};

/**
 * struct nexell_pwm_chip - private data of PWM chip
 * @chip:		generic PWM chip
 * @inverter_mask:	inverter status for all channels - one bit per channel
 * @disabled_mask:	disabled status for all channels - one bit per channel
 * @output_mask:        output enable for all channels - one bit per channel
 * @base:		base address of mapped PWM registers
 * @tclk:		tin clock source
 * @freq:		the input clock to use for each pwm channel
 * @tcfg0:              temporary storage variable for tcfg0 register
 * @tcfg1:              temporary storage variable for tcfg1 register
 * @tcon:               temporary storage variable for tcon register
 * @tcntb:              temporary storage variable for tcntb register
 * @tcmpb:              temporary storage variable for tcmpb register
 * @tintr:              temporary storage variable for tintr register
 */
struct nexell_pwm_chip {
	struct pwm_chip chip;
	u8 inverter_mask;
	u8 disabled_mask;
	u8 output_mask;
	void __iomem *base;
	struct clk *apbclk;
	struct clk *tclk[PWM_NUM];
	u32 freq[PWM_NUM];

	/* for suspend/resume */
	u32 tcfg0[PWM_NUM];
	u32 tcfg1[PWM_NUM];
	u32 tcon[PWM_NUM];
	u32 tcntb[PWM_NUM];
	u32 tcmpb[PWM_NUM];
	u32 tintr[PWM_NUM];
};

const char *pwm_tclk_name[] = {
	"pwm_tclk0", "pwm_tclk1", "pwm_tclk2", "pwm_tclk3"
};

static DEFINE_SPINLOCK(nx_pwm_lock);

static inline
struct nexell_pwm_chip *to_nexell_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct nexell_pwm_chip, chip);
}

static void pwm_nexell_set_divisor(struct nexell_pwm_chip *nx_pwm,
				   unsigned int ch, u8 divisor)
{
	unsigned long flags;
	u32 reg;
	u8 bits;

	bits = fls(divisor) - 1;

	spin_lock_irqsave(&nx_pwm_lock, flags);

	reg = readl(nx_pwm->base + REG_TCFG1(ch));
	reg &= ~TCFG1_MUX_MASK;
	reg |= bits;
	writel(reg, nx_pwm->base + REG_TCFG1(ch));

	spin_unlock_irqrestore(&nx_pwm_lock, flags);
}

static unsigned long pwm_nexell_get_tin_rate(struct nexell_pwm_chip *nx_pwm,
					     unsigned int ch)
{
	unsigned long rate;
	u32 reg;

	rate = clk_get_rate(nx_pwm->tclk[ch]);

	reg = readl(nx_pwm->base + REG_TCFG0(ch));
	reg &= TCFG0_PRESCALER_MASK;

	return rate / (reg + 1);
}

static unsigned long pwm_nexell_calc_tin(struct nexell_pwm_chip *nx_pwm,
					 unsigned int ch)
{
	unsigned long rate;
	u8 div = 0;

	rate = pwm_nexell_get_tin_rate(nx_pwm, ch);
	dev_dbg(nx_pwm->chip.dev, "tin rate %lu\n", rate);

	pwm_nexell_set_divisor(nx_pwm, ch, BIT(div));

	return rate >> div;
}

static int pwm_nexell_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct nexell_pwm_chip *nx_pwm = to_nexell_pwm_chip(chip);
	struct nexell_pwm_channel *nx_chan;
	unsigned int ch = pwm->hwpwm;

	if (!(nx_pwm->output_mask & BIT(ch))) {
		dev_warn(chip->dev,
			"tried to request PWM channel %d without output\n", ch);
		return -EINVAL;
	}

	nx_chan = devm_kzalloc(chip->dev, sizeof(*nx_chan), GFP_KERNEL);
	if (!nx_chan)
		return -ENOMEM;

	pwm_set_chip_data(pwm, nx_chan);

	return 0;
}

static void pwm_nexell_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	devm_kfree(chip->dev, pwm_get_chip_data(pwm));
	pwm_set_chip_data(pwm, NULL);
}

static int pwm_nexell_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct nexell_pwm_chip *nx_pwm = to_nexell_pwm_chip(chip);
	unsigned long flags;
	u32 tcon;
	unsigned int ch = pwm->hwpwm;

	spin_lock_irqsave(&nx_pwm_lock, flags);

	tcon = readl(nx_pwm->base + REG_TCON(ch));

	tcon &= ~TCON_START;
	tcon |= TCON_MANUALUPDATE;
	writel(tcon, nx_pwm->base + REG_TCON(ch));

	tcon &= ~TCON_MANUALUPDATE;
	tcon |= TCON_START | TCON_AUTORELOAD;
	writel(tcon, nx_pwm->base + REG_TCON(ch));

	nx_pwm->disabled_mask &= ~BIT(ch);

	spin_unlock_irqrestore(&nx_pwm_lock, flags);

	return 0;
}

static void pwm_nexell_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct nexell_pwm_chip *nx_pwm = to_nexell_pwm_chip(chip);
	unsigned long flags;
	u32 tcon;
	unsigned int ch = pwm->hwpwm;

	spin_lock_irqsave(&nx_pwm_lock, flags);

	tcon = readl(nx_pwm->base + REG_TCON(ch));
	tcon &= ~TCON_AUTORELOAD;
	writel(tcon, nx_pwm->base + REG_TCON(ch));

	nx_pwm->disabled_mask |= BIT(ch);

	spin_unlock_irqrestore(&nx_pwm_lock, flags);
}

static void pwm_nexell_manual_update(struct nexell_pwm_chip *nx_pwm,
				     struct pwm_device *pwm)
{
	u32 tcon;
	unsigned long flags;
	unsigned int ch = pwm->hwpwm;

	spin_lock_irqsave(&nx_pwm_lock, flags);

	tcon = readl(nx_pwm->base + REG_TCON(ch));
	tcon |= TCON_MANUALUPDATE;
	writel(tcon, nx_pwm->base + REG_TCON(ch));

	tcon &= ~TCON_MANUALUPDATE;
	writel(tcon, nx_pwm->base + REG_TCON(ch));

	spin_unlock_irqrestore(&nx_pwm_lock, flags);
}

static int __pwm_nexell_config(struct pwm_chip *chip, struct pwm_device *pwm,
			       int duty_ns, int period_ns, bool force_period)
{
	struct nexell_pwm_chip *nx_pwm = to_nexell_pwm_chip(chip);
	struct nexell_pwm_channel *nx_chan = pwm_get_chip_data(pwm);
	unsigned int ch = pwm->hwpwm;
	u32 tin_ns = nx_chan->tin_ns, tcnt, tcmp, tduty, oldtcmp;
	unsigned long tin_rate;
	u64 correction;

	/*
	 * We currently avoid using 64bit arithmetic by using the
	 * fact that anything faster than 1Hz is easily representable
	 * by 32bits.
	 */
	if (period_ns > NSEC_PER_SEC)
		return -ERANGE;

	tcnt = readl(nx_pwm->base + REG_TCNTB(ch));
	oldtcmp = readl(nx_pwm->base + REG_TCMPB(ch));

	/* We need tick count for calculation, not last tick. */
	++tcnt;

	tin_rate = pwm_nexell_calc_tin(nx_pwm, ch);

	/* Check to see if we are changing the clock rate of the PWM. */
	if (nx_chan->period_ns != period_ns || force_period) {
		u32 period;

		period = NSEC_PER_SEC / period_ns;

		dev_dbg(nx_pwm->chip.dev, "duty_ns=%d, period_ns=%d (%u)\n",
						duty_ns, period_ns, period);

		dev_dbg(nx_pwm->chip.dev, "tin_rate=%lu\n", tin_rate);

		tin_ns = NSEC_PER_SEC / tin_rate;
		correction = div64_u64((uint64_t)tin_rate * period_ns,
				NSEC_PER_SEC);
		tcnt = (u32)correction;
	}

	/* Period is too short. */
	if (tcnt <= 1)
		return -ERANGE;

	/* Note that counters count down. */
	correction = div64_u64((uint64_t)tin_rate * duty_ns, NSEC_PER_SEC);
	tduty = (u32)correction;

	/* 0% duty is not available */
	if (!tduty)
		++tduty;

	tcmp = tcnt - tduty;

	/* Decrement to get tick numbers, instead of tick counts. */
	--tcnt;
	/* -1UL will give 100% duty. */
	--tcmp;

	if (tcmp == -1)
		tcmp = 0;

	dev_dbg(nx_pwm->chip.dev,
		"tin_ns=%u, tcnt=%u, tcmp=%u\n", tin_ns, tcnt, tcmp);

	/* Update PWM registers. */
	writel(tcnt, nx_pwm->base + REG_TCNTB(ch));
	writel(tcmp, nx_pwm->base + REG_TCMPB(ch));

	/*
	 * In case the PWM is currently at 100% duty cycle, force a manual
	 * update to prevent the signal staying high if the PWM is disabled
	 * shortly afer this update (before it autoreloaded the new values).
	 */
	if (oldtcmp == (u32) -1) {
		dev_dbg(nx_pwm->chip.dev, "Forcing manual update");
		pwm_nexell_manual_update(nx_pwm, pwm);
	}

	nx_chan->period_ns = period_ns;
	nx_chan->tin_ns = tin_ns;
	nx_chan->duty_ns = duty_ns;

	return 0;
}

static void pwm_nexell_set_invert(struct nexell_pwm_chip *nx_pwm,
				  unsigned int ch, bool invert)
{
	unsigned long flags;
	u32 tcon;

	dev_dbg(nx_pwm->chip.dev, "ch=%d, invert=%d\n", ch, invert);

	spin_lock_irqsave(&nx_pwm_lock, flags);

	tcon = readl(nx_pwm->base + REG_TCON(ch));

	if (invert) {
		nx_pwm->inverter_mask |= BIT(ch);
		tcon |= TCON_INVERT;
	} else {
		nx_pwm->inverter_mask &= ~BIT(ch);
		tcon &= ~TCON_INVERT;
	}

	writel(tcon, nx_pwm->base + REG_TCON(ch));

	spin_unlock_irqrestore(&nx_pwm_lock, flags);
}

static int pwm_nexell_set_polarity(struct pwm_chip *chip,
				    struct pwm_device *pwm,
				    enum pwm_polarity polarity)
{
	struct nexell_pwm_chip *nx_pwm = to_nexell_pwm_chip(chip);
	bool invert = (polarity == PWM_POLARITY_NORMAL);

	/* Inverted means normal in the hardware. */
	pwm_nexell_set_invert(nx_pwm, pwm->hwpwm, invert);

	return 0;
}

static int pwm_nexell_config(struct pwm_chip *chip, struct pwm_device *pwm,
			      int duty_ns, int period_ns)
{
	pwm_nexell_set_invert(to_nexell_pwm_chip(chip),
			      pwm->hwpwm, pwm->state.polarity ? false : true);

	return __pwm_nexell_config(chip, pwm, duty_ns, period_ns, false);
}

static const struct pwm_ops pwm_nexell_ops = {
	.request	= pwm_nexell_request,
	.free		= pwm_nexell_free,
	.enable		= pwm_nexell_enable,
	.disable	= pwm_nexell_disable,
	.config		= pwm_nexell_config,
	.set_polarity	= pwm_nexell_set_polarity,
	.owner		= THIS_MODULE,
};

static const struct of_device_id nexell_pwm_matches[] = {
	{ .compatible = "nexell,nxp3220-pwm" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nexell_pwm_matches);

static int pwm_nexell_parse_dt(struct nexell_pwm_chip *nx_pwm)
{
	struct device_node *np = nx_pwm->chip.dev->of_node;
	const struct of_device_id *match;
	struct property *prop;
	const __be32 *cur;
	u32 val;
	int ch = 0;

	match = of_match_node(nexell_pwm_matches, np);
	if (!match)
		return -ENODEV;

	of_property_for_each_u32(np, "nexell,pwm-outputs", prop, cur, val) {
		if (val >= PWM_NUM) {
			dev_err(nx_pwm->chip.dev,
				"%s: invalid channel index in pwm-outputs\n",
								__func__);
			continue;
		}
		nx_pwm->output_mask |= BIT(val);
	}

	of_property_for_each_u32(np, "tclk_freq", prop, cur, val) {
		if (ch >= PWM_NUM)
			break;

		for ( ; ch < PWM_NUM; ch++) {
			if (nx_pwm->output_mask & BIT(ch)) {
				nx_pwm->freq[ch++] = val;
				break;
			}
		}
	}

	return 0;
}

static int pwm_nexell_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nexell_pwm_chip *nx_pwm;
	struct resource *res;
	unsigned int ch;
	int ret;

	nx_pwm = devm_kzalloc(dev, sizeof(*nx_pwm), GFP_KERNEL);
	if (nx_pwm == NULL)
		return -ENOMEM;

	nx_pwm->chip.dev = dev;
	nx_pwm->chip.ops = &pwm_nexell_ops;
	nx_pwm->chip.base = -1;
	nx_pwm->chip.npwm = PWM_NUM;
	nx_pwm->inverter_mask = BIT(PWM_NUM) - 1;

	if (IS_ENABLED(CONFIG_OF) && dev->of_node) {
		ret = pwm_nexell_parse_dt(nx_pwm);
		if (ret)
			return ret;

		nx_pwm->chip.of_xlate = of_pwm_xlate_with_flags;
		nx_pwm->chip.of_pwm_n_cells = 3;
	} else {
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nx_pwm->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(nx_pwm->base))
		return PTR_ERR(nx_pwm->base);

	nx_pwm->apbclk = devm_clk_get(dev, "pwm_apb");
	if (IS_ERR(nx_pwm->apbclk)) {
		dev_err(dev, "failed to get pwm apb clk\n");
		return PTR_ERR(nx_pwm->apbclk);
	}

	for (ch = 0; ch < PWM_NUM; ++ch) {
		if (!(nx_pwm->output_mask & BIT(ch)))
			continue;

		nx_pwm->tclk[ch] = devm_clk_get(dev, pwm_tclk_name[ch]);
		if (IS_ERR(nx_pwm->tclk[ch])) {
			dev_err(dev, "failed to get pwm tclk %d\n", ch);
			return PTR_ERR(nx_pwm->tclk[ch]);
		}

		ret = clk_set_rate(nx_pwm->tclk[ch], nx_pwm->freq[ch]);
		if (ret < 0) {
			dev_err(dev, "Failed to set freq (ret:%d)\n", ret);
			return ret;
		}

		ret = clk_prepare_enable(nx_pwm->tclk[ch]);
		if (ret < 0) {
			dev_err(dev, "failed to enable clock %d\n", ch);
			return ret;
		}

		pwm_nexell_set_invert(nx_pwm, ch, true);
	}

	platform_set_drvdata(pdev, nx_pwm);

	ret = pwmchip_add(&nx_pwm->chip);
	if (ret < 0) {
		dev_err(dev, "failed to register PWM nx_pwm\n");
		for (ch = 0; ch < PWM_NUM; ++ch)
			if (nx_pwm->output_mask & BIT(ch))
				clk_disable_unprepare(nx_pwm->tclk[ch]);
		return ret;
	}

	for (ch = 0; ch < PWM_NUM; ch++) {
		dev_dbg(dev, "tclk%d at %lu\n",
				ch,
				IS_ERR_OR_NULL(nx_pwm->tclk[ch]) ? 0 :
				clk_get_rate(nx_pwm->tclk[ch]));
	}

	return 0;
}

static int pwm_nexell_remove(struct platform_device *pdev)
{
	struct nexell_pwm_chip *nx_pwm = platform_get_drvdata(pdev);
	int ch;
	int ret;

	ret = pwmchip_remove(&nx_pwm->chip);
	if (ret < 0)
		return ret;

	for (ch = 0; ch < PWM_NUM; ++ch)
		if (nx_pwm->output_mask & BIT(ch))
			clk_disable_unprepare(nx_pwm->tclk[ch]);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_nexell_suspend(struct device *dev)
{
	struct nexell_pwm_chip *nx_pwm = dev_get_drvdata(dev);
	struct pwm_chip *chip = &nx_pwm->chip;
	unsigned int ch;

	for (ch = 0; ch < PWM_NUM; ++ch) {
		struct pwm_device *pwm = &chip->pwms[ch];

		if (!(nx_pwm->output_mask & BIT(ch)))
			continue;

		/* save registers */
		nx_pwm->tcfg0[ch] = readl(nx_pwm->base + REG_TCFG0(ch));
		nx_pwm->tcfg1[ch] = readl(nx_pwm->base + REG_TCFG1(ch));
		nx_pwm->tcntb[ch] = readl(nx_pwm->base + REG_TCNTB(ch));
		nx_pwm->tcmpb[ch] = readl(nx_pwm->base + REG_TCMPB(ch));
		nx_pwm->tintr[ch] = readl(nx_pwm->base + REG_TINTR(ch));
		nx_pwm->tcon[ch] = readl(nx_pwm->base + REG_TCON(ch));
		pwm_nexell_disable(chip, pwm);

		/* clock off */
		if (!IS_ERR_OR_NULL(nx_pwm->tclk[ch]))
			clk_disable_unprepare(nx_pwm->tclk[ch]);
	}

	return 0;
}

static int pwm_nexell_resume(struct device *dev)
{
	struct nexell_pwm_chip *nx_pwm = dev_get_drvdata(dev);
	struct pwm_chip *chip = &nx_pwm->chip;
	unsigned int ch;

	for (ch = 0; ch < PWM_NUM; ++ch) {
		struct pwm_device *pwm = &chip->pwms[ch];
		struct nexell_pwm_channel *nx_chan = pwm_get_chip_data(pwm);

		if (!(nx_pwm->output_mask & BIT(ch)))
			continue;

		/* clock on */
		if (!IS_ERR_OR_NULL(nx_pwm->tclk[ch]))
			clk_prepare_enable(nx_pwm->tclk[ch]);

		/* restore registers */
		writel(nx_pwm->tcfg0[ch], nx_pwm->base + REG_TCFG0(ch));
		writel(nx_pwm->tcfg1[ch], nx_pwm->base + REG_TCFG1(ch));
		writel(nx_pwm->tcon[ch], nx_pwm->base + REG_TCON(ch));
		writel(nx_pwm->tcntb[ch], nx_pwm->base + REG_TCNTB(ch));
		writel(nx_pwm->tcmpb[ch], nx_pwm->base + REG_TCMPB(ch));
		writel(nx_pwm->tintr[ch], nx_pwm->base + REG_TINTR(ch));

		if (nx_chan && nx_chan->period_ns) {
			__pwm_nexell_config(chip, pwm, nx_chan->duty_ns,
					    nx_chan->period_ns, true);
			pwm_nexell_manual_update(nx_pwm, pwm);
		}

		if (nx_pwm->disabled_mask & BIT(ch))
			pwm_nexell_disable(chip, pwm);
		else
			pwm_nexell_enable(chip, pwm);
	}

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_nexell_pm_ops, pwm_nexell_suspend,
			 pwm_nexell_resume);

static struct platform_driver pwm_nexell_driver = {
	.driver		= {
		.name	= "nexell-pwm",
		.pm	= &pwm_nexell_pm_ops,
		.of_match_table = of_match_ptr(nexell_pwm_matches),
	},
	.probe		= pwm_nexell_probe,
	.remove		= pwm_nexell_remove,
};
#ifdef CONFIG_DEFERRED_UP_PWM
static int __init pwm_nxp3220_init(void)
{
	return platform_driver_register(&pwm_nexell_driver);
}
early_device_initcall(pwm_nxp3220_init)
#else
module_platform_driver(pwm_nexell_driver);
#endif

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Bon-gyu, KOO <freestyle@nexell.co.kr>");
MODULE_ALIAS("platform:nexell-pwm");
