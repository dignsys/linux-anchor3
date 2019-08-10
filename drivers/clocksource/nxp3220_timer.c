/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 *
 * PDX-License-Identifier: GPL-2.0+
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/clk-provider.h>
#include <linux/reset.h>
#include <linux/percpu.h>
#include <linux/sched_clock.h>

/* timer register */
#define REG_TCFG0 (0x00)
#define REG_TCFG1 (0x04)
#define REG_TCON (0x08)
#define REG_TCNTB0 (0x0C)
#define REG_TCMPB0 (0x10)
#define REG_TCNT0 (0x14)
#define REG_CSTAT (0x18)

#define REG_OFFSET (0x100)

#define TCFG0_PRESCALER_MASK (0xff)
#define TCFG1_MUX_MASK (0x7)

#define TCON_BIT_AUTO (1 << 3)
#define TCON_BIT_INVT (1 << 2)
#define TCON_BIT_UP (1 << 1)
#define TCON_BIT_RUN (1 << 0)
#define CSTAT_IRQSTAT (1 << 5)
#define CSTAT_IRQ_EN_BIT 0

/* timer data structs */
struct timer_info {
	void __iomem *base;
	int interrupt;
	const char *clock_name;
	struct clk *clk;
	unsigned long request;
	unsigned long rate;
	int divmux;
	int prescale;
	unsigned int tcount;
	unsigned int rcount;
};

struct timer_of_dev {
	struct timer_info timer_source;
};

static struct timer_of_dev *timer_dev;
#define get_timer_dev() ((struct timer_of_dev *)timer_dev)

struct nxp3220_clk_evtdev {
	struct clock_event_device evt;
	struct timer_info timer_info;
};

static struct nxp3220_clk_evtdev __percpu *nxp3220_evtdev;

static inline void nxp3220_timer_clock(void __iomem *base, int mux, int scl)
{
	writel((scl - 1) & TCFG0_PRESCALER_MASK, base + REG_TCFG0);
	writel(mux & TCFG1_MUX_MASK, base + REG_TCFG1);
}

static inline void nxp3220_timer_count(void __iomem *base, unsigned int cnt)
{
	writel(cnt, base + REG_TCNTB0);
	writel(cnt, base + REG_TCMPB0);
}

static inline void nxp3220_timer_start(void __iomem *base, int irqon)
{
	int on = irqon ? 1 : 0;
	u32 val;

	writel(on, base + REG_CSTAT);
	writel(TCON_BIT_UP, base + REG_TCON);

	val = (TCON_BIT_AUTO | TCON_BIT_RUN);
	writel(val, base + REG_TCON);
}

static inline void nxp3220_timer_stop(void __iomem *base, int irqon)
{
	int on = irqon ? 1 : 0;

	on |= CSTAT_IRQSTAT;
	writel(on, base + REG_CSTAT);
	writel(0, base + REG_TCON);
}

static inline unsigned int nxp3220_timer_read(void __iomem *base)
{
	return readl(base + REG_TCNT0);
}

static inline u32 nxp3220_timer_read_count(void)
{
	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = &dev->timer_source;

	if (dev == NULL || info->base == NULL)
		return 0;

	info = &dev->timer_source;

	info->rcount = (info->tcount - nxp3220_timer_read(info->base));

	return (u32)info->rcount;
}

static int nxp3220_timer_set_periodic(struct clock_event_device *evt)
{
	struct nxp3220_clk_evtdev *nevt =
		container_of(evt, struct nxp3220_clk_evtdev, evt);
	struct timer_info *info = &nevt->timer_info;
	void __iomem *base = info->base;
	unsigned long cnt = info->tcount;

	nxp3220_timer_stop(base, 0);
	nxp3220_timer_count(base, cnt - 1);
	nxp3220_timer_start(base, 1);

	return 0;
}

static int nxp3220_timer_set_next_event(unsigned long delta,
		struct clock_event_device *evt)
{
	struct nxp3220_clk_evtdev *nevt =
		container_of(evt, struct nxp3220_clk_evtdev, evt);
	struct timer_info *info = &nevt->timer_info;
	void __iomem *base = info->base;

	nxp3220_timer_stop(base, 0);
	nxp3220_timer_count(base, delta - 1);
	nxp3220_timer_start(base, 1);

	return 0;
}

static int nxp3220_timer_set_shutdown(struct clock_event_device *evt)
{
	struct nxp3220_clk_evtdev *nevt =
		container_of(evt, struct nxp3220_clk_evtdev, evt);
	struct timer_info *info = &nevt->timer_info;
	void __iomem *base = info->base;

	nxp3220_timer_stop(base, 0);

	return 0;
}

static void nxp3220_timer_event_resume(struct clock_event_device *evt)
{
	struct nxp3220_clk_evtdev *nevt =
		container_of(evt, struct nxp3220_clk_evtdev, evt);
	struct timer_info *info = &nevt->timer_info;
	void __iomem *base = info->base;

	pr_debug("%s (mux:%d, scale:%d)\n", __func__, info->divmux,
		 info->prescale);

	nxp3220_timer_stop(base, 1);
	nxp3220_timer_clock(base, info->divmux, info->prescale);
}

static int nxp3220_timer_starting_cpu(unsigned int cpu)
{
	struct nxp3220_clk_evtdev *nevt = per_cpu_ptr(nxp3220_evtdev, cpu);
	struct clock_event_device *evt = &nevt->evt;
	struct timer_info *tevt = &nevt->timer_info;

	/* setting clockevent */
	evt->name = "nxp3220_timer";
	evt->set_next_event = nxp3220_timer_set_next_event;
	evt->set_state_shutdown = nxp3220_timer_set_shutdown;
	evt->set_state_periodic = nxp3220_timer_set_periodic;
	evt->tick_resume = nxp3220_timer_set_shutdown;
	evt->features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	evt->resume = nxp3220_timer_event_resume,
	evt->irq = nevt->timer_info.interrupt;
	irq_force_affinity(evt->irq, cpumask_of(cpu));

	clockevents_config_and_register(evt, tevt->rate, 0xf, 0xffffffff);

	return 0;
}

static int nxp3220_timer_dying_cpu(unsigned int cpu)
{
	struct nxp3220_clk_evtdev *nevt = per_cpu_ptr(nxp3220_evtdev, cpu);
	struct clock_event_device *evt = &nevt->evt;

	evt->set_state_shutdown(evt);

	return 0;
}

/*
 * Timer clock source
 */
static void nxp3220_timer_clock_select(struct timer_info *info)
{
	unsigned long rate, tout = 0;
	unsigned long mout, thz, delt = (-1UL);
	unsigned long frequency = info->request;
	int tscl = 0, tmux = 5, smux = 0, pscl = 0;
	int ret;

	if (!info->request) {
		pr_err("request frequency is zero\n");
		return;
	}

	ret = clk_set_rate(info->clk, info->request);
	if (ret < 0) {
		pr_err("failed to set freq %lu (%d)\n", info->request, ret);
		return;
	}

	clk_prepare_enable(info->clk);
	rate = clk_get_rate(info->clk);

	for (smux = 0; smux < 5; smux++) {
		mout = rate / (1 << smux);
		pscl = mout / frequency;
		thz = mout / (pscl ? pscl : 1);
		if (!(mout % frequency) && 256 > pscl) {
			tout = thz, tmux = smux, tscl = pscl;
			break;
		}
		if (pscl > 256)
			continue;
		if (abs(frequency - thz) >= delt)
			continue;
		tout = thz, tmux = smux, tscl = pscl;
		delt = abs(frequency - thz);
	}

	info->divmux = tmux;
	info->prescale = tscl;
	info->tcount = tout / HZ;
	info->rate = tout;

	pr_debug("%s (mux=%d, scl=%d, rate=%ld, tcount = %d)\n",
			__func__, tmux, tscl, tout, info->tcount);
}

static void nxp3220_timer_source_suspend(struct clocksource *cs)
{
	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = &dev->timer_source;
	void __iomem *base = info->base;

	info->rcount = (info->tcount - nxp3220_timer_read(base));
	nxp3220_timer_stop(base, 0);

	if (info->clk)
		clk_disable_unprepare(info->clk);
}

static void nxp3220_timer_source_resume(struct clocksource *cs)
{
	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = &dev->timer_source;
	void __iomem *base = info->base;

	pr_debug("%s (mux:%d, scale:%d cnt:0x%x,0x%x)\n", __func__,
		 info->divmux, info->prescale, info->rcount, info->tcount);

	if (info->clk) {
		clk_set_rate(info->clk, info->rate);
		clk_prepare_enable(info->clk);
	}

	nxp3220_timer_stop(base, 0);
	nxp3220_timer_clock(base, info->divmux, info->prescale);
	nxp3220_timer_count(base, info->rcount); /* restore count */
	nxp3220_timer_start(base, 0);
	nxp3220_timer_count(base, info->tcount); /* next count */
}

static u64 nxp3220_timer_source_read(struct clocksource *cs)
{
	return (u64)nxp3220_timer_read_count();
}

static u64 notrace nxp3220_sched_clock_read(void)
{
	return (u64)nxp3220_timer_read_count();
}

static struct clocksource timer_clocksource = {
	.name = "source timer",
	.rating = 300,
	.read = nxp3220_timer_source_read,
	.mask = CLOCKSOURCE_MASK(32),
	.shift = 20,
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend = nxp3220_timer_source_suspend,
	.resume = nxp3220_timer_source_resume,
};

static struct delay_timer delay_timer = {
	.read_current_timer = (unsigned long (*)(void))nxp3220_timer_read_count,
};

static int __init nxp3220_timer_source_init(void)
{
	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = &dev->timer_source;
	struct clocksource *cs = &timer_clocksource;
	void __iomem *base = info->base;

	nxp3220_timer_clock_select(info);

	/* reset tcount */
	info->tcount = 0xFFFFFFFF;

	clocksource_register_hz(cs, info->rate);

	nxp3220_timer_stop(base, 0);
	nxp3220_timer_clock(base, info->divmux, info->prescale);
	nxp3220_timer_count(base, info->tcount);
	nxp3220_timer_start(base, 0);

	sched_clock_register(nxp3220_sched_clock_read, 32, info->rate);

	delay_timer.freq = info->request;

	pr_debug("timer: source, %9lu(HZ:%d), mult:%u\n",
			info->rate, HZ, cs->mult);

	return 0;
}

/*
 * Timer clock event
 */
static int __init nxp3220_timer_event_init(int cpu)
{
	struct nxp3220_clk_evtdev *nevt = per_cpu_ptr(nxp3220_evtdev, cpu);
	struct timer_info *info = &nevt->timer_info;
	struct clock_event_device *evt = &nevt->evt;
	void __iomem *base = info->base;

	nxp3220_timer_clock_select(info);
	nxp3220_timer_stop(base, 1);
	nxp3220_timer_clock(base, info->divmux, info->prescale);

	evt->irq = info->interrupt;
	evt->cpumask = cpumask_of(cpu);
	evt->rating = 500;

	pr_debug("timer: event , %9lu(HZ:%d), mult:%u\n",
			info->rate, HZ, evt->mult);

	return 0;
}

static int __init nxp3220_timer_local_init(void)
{
	int ret;

	ret = cpuhp_setup_state(CPUHP_AP_NXP3220_TIMER_STARTING,
				"clockevents/nxp3220/timer:starting",
				nxp3220_timer_starting_cpu,
				nxp3220_timer_dying_cpu);
	if (ret) {
		pr_err("Failed to setup hotplug state\n");
		ret = -EINVAL;
		goto out;
	}

	return 0;

out:
	return ret;
}

static irqreturn_t nxp3220_timer_interrupt(int irq, void *dev_id)
{
	struct nxp3220_clk_evtdev *nevt = dev_id;
	struct clock_event_device *evt = &nevt->evt;
	struct timer_info *info = &nevt->timer_info;
	void __iomem *base = info->base;

	/* clear status */
	writel(CSTAT_IRQSTAT | 0x1, base + REG_CSTAT);
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction nxp3220_timer0_irq = {
	.name = "nxp3220_timer0",
	.flags = IRQF_TIMER | IRQF_NOBALANCING,
	.handler = nxp3220_timer_interrupt,
};

static struct irqaction nxp3220_timer1_irq = {
	.name = "nxp3220_timer1",
	.flags = IRQF_TIMER | IRQF_NOBALANCING,
	.handler = nxp3220_timer_interrupt,
};

static int __init
timer_get_device_data(struct device_node *node, struct timer_of_dev *dev)
{
	struct nxp3220_clk_evtdev *nevt0, *nevt1;
	struct timer_info *tsrc = &dev->timer_source;
	struct timer_info *tevt0, *tevt1;
	int src_ch;
	int evt0_ch;
	int evt1_ch;
	void __iomem *base;
	struct clk *apbclk;
	u32 apbclk_rate;
	u32 *evt_rates;
	char *name;
	int vcount;

	base = of_iomap(node, 0);

	if (IS_ERR(base)) {
		pr_err("Can't map registers for timer!");
		return -EINVAL;
	}

	if (of_property_read_u32(node, "clksource", &src_ch)) {
		pr_err("timer node is missing 'clksource'\n");
		return -EINVAL;
	}

	if (of_property_read_u32(node, "clkevent0", &evt0_ch)) {
		pr_err("timer node is missing 'clkevent0'\n");
		return -EINVAL;
	}

	if (of_property_read_u32(node, "clkevent1", &evt1_ch)) {
		pr_err("timer node is missing 'clkevent1'\n");
		return -EINVAL;
	}

	if ((1 << src_ch) & (1 << evt0_ch) & (1 << evt1_ch)) {
		pr_err(" 'clksource' and 'clkevent' is same channel\n");
		return -EINVAL;
	}

	/*
	 * apb(PCLK) clock setting
	 */
	if (of_property_read_u32(node, "clksrc-frequency", &apbclk_rate)) {
		pr_err("failed to get apb clock frequency\n");
		return -EINVAL;
	}

	apbclk = of_clk_get_by_name(node, "apb");
	if (IS_ERR(apbclk)) {
		pr_err("failed to get timer apb clock\n");
		return -EINVAL;
	}

	if (clk_set_rate(apbclk, apbclk_rate) < 0) {
		pr_err("failed to set freq %u\n", apbclk_rate);
		return -EINVAL;
	}
	tsrc->request = apbclk_rate;

	if (clk_prepare_enable(apbclk) < 0) {
		pr_err("failed to enable timer apb clock\n");
		return -EINVAL;
	}

	tsrc->base = base + src_ch * REG_OFFSET;
	tsrc->clk = of_clk_get_by_name(node, "clksrc");
	if (IS_ERR(tsrc->clk)) {
		pr_err("failed timer tsrc clock\n");
		return -EINVAL;
	}

	/*
	 * event device setting
	 */
	nevt0 = per_cpu_ptr(nxp3220_evtdev, 0);
	nevt1 = per_cpu_ptr(nxp3220_evtdev, 1);

	tevt0 = &nevt0->timer_info;
	tevt1 = &nevt1->timer_info;

	tevt0->base = base + evt0_ch * REG_OFFSET;
	tevt1->base = base + evt1_ch * REG_OFFSET;

	/* event device: interrupt */
	tevt0->interrupt = irq_of_parse_and_map(node, 0);
	tevt1->interrupt = irq_of_parse_and_map(node, 1);

	nxp3220_timer0_irq.dev_id = nevt0;
	nxp3220_timer1_irq.dev_id = nevt1;

	setup_irq(tevt0->interrupt, &nxp3220_timer0_irq);
	setup_irq(tevt1->interrupt, &nxp3220_timer1_irq);

	/* event device: clock */
	tevt0->clk = of_clk_get_by_name(node, "clkevt0");
	if (IS_ERR(tevt0->clk)) {
		pr_err("failed to get timer event0 clock\n");
		return -EINVAL;
	}

	tevt1->clk = of_clk_get_by_name(node, "clkevt1");
	if (IS_ERR(tevt1->clk)) {
		pr_err("failed to get timer event1 clock\n");
		return -EINVAL;
	}

	name = "clkevt-frequencies";
	vcount = of_property_count_u32_elems(node, name);
	evt_rates = kmalloc_array(vcount, sizeof(*evt_rates), GFP_KERNEL);
	if (!evt_rates)
		return -ENOMEM;

	if (of_property_read_u32_array(node, name, evt_rates, vcount)) {
		pr_err("%s: error parsing %s\n", __func__, name);
		kfree(evt_rates);
		return -EINVAL;
	}

	if (vcount == 2) {
		tevt0->request = evt_rates[0];
		tevt1->request = evt_rates[1];
	} else {
		tevt0->request = evt_rates[0];
		tevt1->request = evt_rates[0];
	}

	pr_debug("%s : irq %d, %d\n", node->name,
			tevt0->interrupt, tevt1->interrupt);

	return 0;
}

static int __init timer_of_init_dt(struct device_node *node)
{
	struct timer_of_dev *dev;
	u32 i;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	timer_dev = dev;

	nxp3220_evtdev = alloc_percpu(struct nxp3220_clk_evtdev);
	if (!nxp3220_evtdev) {
		pr_err("Failed to alloc memory for local timer\n");
		ret = -ENOMEM;
		goto out;
	}

	if (timer_get_device_data(node, dev))
		panic("unable to map timer cpu !!!\n");

	nxp3220_timer_source_init();
	for_each_possible_cpu(i)
		nxp3220_timer_event_init(i);

	nxp3220_timer_local_init();

	/* timer-based delay loop */
	register_current_timer_delay(&delay_timer);

	return 0;

out:
	return ret;
}
TIMER_OF_DECLARE(nxp3220, "nexell,nxp3220-timer", timer_of_init_dt);
