// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/reset.h>
#include <linux/input.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/driver.h>
#include <linux/iio/machine.h>

#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/of_device.h>


/*
 * ADC definitions
 */
#define	ADC_MAX_SAMPLE_RATE	(1*1000*1000)	/* with 6bit */
#define	ADC_MAX_SAMPLE_BITS	6
#define	ADC_MAX_PRESCALE	256		/* 8bit */
#define	ADC_MIN_PRESCALE	20

#define ADC_TIMEOUT		(msecs_to_jiffies(100))
#define MAX_ADC_V1_CHANNELS	8
#define MAX_ADC_V2_CHANNELS	8

/* Register definitions for ADC_V1 */
#define ADC_V1_CON(x)		((x) + 0x00)
#define ADC_V1_DAT(x)		((x) + 0x04)
#define ADC_V1_INTENB(x)	((x) + 0x08)
#define ADC_V1_INTCLR(x)	((x) + 0x0c)

/* Bit definitions for ADC_V1 */
#define ADC_V1_CON_APEN		(1u << 14)
#define ADC_V1_CON_APSV(x)	(((x) & 0xff) << 6)
#define ADC_V1_CON_ASEL(x)	(((x) & 0x7) << 3)
#define ADC_V1_CON_STBY		(1u << 2)
#define ADC_V1_CON_ADEN		(1u << 0)
#define ADC_V1_INTENB_ENB	(1u << 0)
#define ADC_V1_INTCLR_CLR	(1u << 0)

/* Register definitions for ADC_V2 */
#define ADC_V2_CON(x)		((x) + 0x00)
#define ADC_V2_DAT(x)		((x) + 0x04)
#define ADC_V2_INTENB(x)	((x) + 0x08)
#define ADC_V2_INTCLR(x)	((x) + 0x0c)
#define ADC_V2_PRESCON(x)	((x) + 0x10)

/* Bit definitions for ADC_V2 */
#define ADC_V2_CON_DATA_SEL(x)	(((x) & 0xf) << 10)
#define ADC_V2_CON_CLK_CNT(x)	(((x) & 0xf) << 6)
#define ADC_V2_CON_ASEL(x)	(((x) & 0x7) << 3)
#define ADC_V2_CON_STBY		(1u << 2)
#define ADC_V2_CON_ADEN		(1u << 0)
#define ADC_V2_INTENB_ENB	(1u << 0)
#define ADC_V2_INTCLR_CLR	(1u << 0)
#define ADC_V2_PRESCON_APEN	(1u << 15)
#define ADC_V2_PRESCON_PRES(x)	(((x) & 0x3ff) << 0)

#define ADC_V2_DATA_SEL_VAL	(0)	/* 0:5clk, 1:4clk, 2:3clk, 3:2clk */
					/* 4:1clk: 5:not delayed, else: 4clk */
#define ADC_V2_CLK_CNT_VAL	(6)	/* 28nm ADC */

/* Register definitions for ADC_V3 */
#define ADC_V3_EN(x)		((x) + 0x18)

/* Touchscreen Register */
#define ADC_V3_TOUCHSCREENCON(x)	((x) + 0x14)

/* Bit definitions for ADC_V3 for Touchscreen */
#define ADC_V3_TS_INTCLR		(1u << 1)
#define ADC_V3_TS_INTENB		(1u << 1)

#define ADC_V3_TS_PULLUP_DISABLE	(1u << 4)
#define ADC_V3_TS_XP_OFF		(1u << 3)
#define ADC_V3_TS_XM_ON			(1u << 2)
#define ADC_V3_TS_YP_OFF		(1u << 1)
#define ADC_V3_TS_YM_ON			(1u << 0)

#define ADC_DATX_MASK			0xFFF
#define ADC_DATY_MASK			0xFFF

/* ADC X,Y Plus/Minus Channel */
#define NX_ADC_CH_TS_XP			9
#define NX_ADC_CH_TS_XM			8
#define NX_ADC_CH_TS_YP			7
#define NX_ADC_CH_TS_YM			6

#define X_AXIS_MIN			0
#define X_AXIS_MAX			((1 << 12) - 1)
#define Y_AXIS_MAX			X_AXIS_MAX
#define Y_AXIS_MIN			X_AXIS_MIN

#define TS_PENDOWN			0
#define TS_PENUP			1
#define TS_CORRECTION_VALUE		250
#define TS_COORDINATE_CNT		3

#define ADC_CHANNEL(_index, _addr, _id) {		\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,					\
	.channel = _index,				\
	.address = _addr,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.datasheet_name = _id,				\
}

static struct iio_chan_spec nexell_adc_channels[] = {
	ADC_CHANNEL(0, 0, "adc0"),
	ADC_CHANNEL(1, 1, "adc1"),
	ADC_CHANNEL(2, 2, "adc2"),
	ADC_CHANNEL(3, 3, "adc3"),
	ADC_CHANNEL(4, 4, "adc4"),
	ADC_CHANNEL(5, 5, "adc5"),
	ADC_CHANNEL(6, 6, "adc6"),
	ADC_CHANNEL(7, 7, "adc7"),
};

static struct iio_chan_spec nexell_adc_v3_channels[] = {
	ADC_CHANNEL(0, 0, "adc0"),
	ADC_CHANNEL(1, 1, "adc1"),
	ADC_CHANNEL(2, 2, "adc2"),
	ADC_CHANNEL(3, 3, "adc3"),
	ADC_CHANNEL(6, 4, "adc6"),
	ADC_CHANNEL(7, 5, "adc7"),
	ADC_CHANNEL(8, 6, "adc8"),
	ADC_CHANNEL(9, 7, "adc9"),
};

/* ADC-TS Data */
struct nexell_adc_ts {
	struct device *dev;
	struct input_dev *inp;
	struct iio_dev *iio;

	int pendown;
	int status;

	int pre_xpos;
	int pre_ypos;

	int ts_sample_time;

	int reg_value_backup;

	struct workqueue_struct	*wqueue;
	struct work_struct work;
};

/*
 * ADC data
 */
struct nexell_adc_info {
	struct nexell_adc_data *data;
	void __iomem *adc_base;
	ulong clk_rate;
	ulong sample_rate;
	ulong max_sample_rate;
	ulong min_sample_rate;
	int value;
	int prescale;
	spinlock_t lock;
	struct completion completion;
	int irq;
	struct clk *clk;
	struct iio_map *map;
	struct reset_control *rst;

	struct nexell_adc_ts *tsc;
};

struct nexell_adc_data {
	int version;
	int num_channels;
	struct iio_chan_spec *channels;
	int (*adc_con)(struct nexell_adc_info *adc);
	int (*read_polling)(struct nexell_adc_info *adc, int ch);
	int (*read_val)(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask);
};

static int nexell_adc_remove_devices(struct device *dev, void *c)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static int setup_adc_con(struct nexell_adc_info *adc)
{
	void __iomem *reg = adc->adc_base;

	if (adc->data->version == 3)
		if ((readl(ADC_V3_EN(reg)) & 0x1) == 0)
			writel(0x1, ADC_V3_EN(reg));

	if (adc->data->adc_con)
		adc->data->adc_con(adc);

	return 0;
}

/* you must irq re-enable after sampling */
static void nexell_adc_v1_ch_start(void __iomem *reg, int ch)
{
	unsigned int adcon = 0;

	adcon = readl(ADC_V1_CON(reg)) & ~ADC_V1_CON_ASEL(7);
	adcon &= ~ADC_V1_CON_ADEN;
	adcon |= ADC_V1_CON_ASEL(ch);	/* channel */
	writel(adcon, ADC_V1_CON(reg));
	adcon = readl(ADC_V1_CON(reg));

	adcon |= ADC_V1_CON_ADEN;	/* start */
	local_irq_disable();
	writel(adcon, ADC_V1_CON(reg));
}

static int nexell_adc_v1_read_polling(struct nexell_adc_info *adc, int ch)
{
	void __iomem *reg = adc->adc_base;
	unsigned long wait = loops_per_jiffy * (HZ/10);

	nexell_adc_v1_ch_start(reg, ch);

	while (wait > 0) {
		if (!(readl(ADC_V1_CON(reg)) & ADC_V1_CON_ADEN)) {
			/* get value */
			adc->value = readl(ADC_V1_DAT(reg)); /* get value */
			/* pending clear */
			writel(ADC_V1_INTCLR_CLR, ADC_V1_INTCLR(reg));
			break;
		}
		wait--;
	}
	local_irq_enable();
	if (wait == 0)
		return -ETIMEDOUT;

	return 0;
}

static int nexell_adc_v1_adc_con(struct nexell_adc_info *adc)
{
	unsigned int adcon = 0;
	void __iomem *reg = adc->adc_base;

	adcon = ADC_V1_CON_APSV(adc->prescale);
	adcon &= ~ADC_V1_CON_STBY;
	writel(adcon, ADC_V1_CON(reg));
	adcon |= ADC_V1_CON_APEN;
	writel(adcon, ADC_V1_CON(reg));

	/* *****************************************************
	 * Turn-around invalid value after Power On
	 * *****************************************************/
	nexell_adc_v1_read_polling(adc, 0);
	adc->value = 0;

	writel(ADC_V1_INTCLR_CLR, ADC_V1_INTCLR(reg));
	writel(ADC_V1_INTENB_ENB, ADC_V1_INTENB(reg));
	init_completion(&adc->completion);

	return 0;
}

static int nexell_adc_v1_read_val(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val,
		int *val2,
		long mask)
{
	struct nexell_adc_info *adc = iio_priv(indio_dev);
	int ret = 0;

	reinit_completion(&adc->completion);

	if (adc->data->read_polling)
		ret = adc->data->read_polling(adc, chan->address);
	if (ret < 0) {
		dev_warn(&indio_dev->dev,
				"Conversion timed out! resetting...\n");
		if (adc->rst)
			reset_control_reset(adc->rst);
		setup_adc_con(adc);
		ret = -ETIMEDOUT;
	}

	return ret;
}

static const struct nexell_adc_data nexell_adc_s5p4418_data = {
	.version	= 1,
	.num_channels	= MAX_ADC_V1_CHANNELS,
	.channels	= nexell_adc_channels,
	.adc_con	= nexell_adc_v1_adc_con,
	.read_polling	= nexell_adc_v1_read_polling,
	.read_val	= nexell_adc_v1_read_val,
};

static void nexell_adc_v2_ch_start(void __iomem *reg, int ch)
{
	unsigned int adcon = 0;

	adcon = readl(ADC_V2_CON(reg)) & ~ADC_V2_CON_ASEL(7);
	adcon &= ~ADC_V2_CON_ADEN;
	adcon |= ADC_V2_CON_ASEL(ch);	/* channel */
	writel(adcon, ADC_V2_CON(reg));
	adcon = readl(ADC_V2_CON(reg));

	adcon |= ADC_V2_CON_ADEN;	/* start */
	writel(adcon, ADC_V2_CON(reg));
}

static int nexell_adc_v2_read_polling(struct nexell_adc_info *adc, int ch)
{
	void __iomem *reg = adc->adc_base;
	unsigned long wait = loops_per_jiffy * (HZ/10);

	nexell_adc_v2_ch_start(reg, ch);

	while (wait > 0) {
		if (readl(ADC_V2_INTCLR(reg)) & ADC_V2_INTCLR_CLR) {
			/* pending clear */
			writel(ADC_V2_INTCLR_CLR, ADC_V2_INTCLR(reg));
			/* get value */
			adc->value = readl(ADC_V2_DAT(reg)); /* get value */
			break;
		}
		wait--;
	}
	if (wait == 0)
		return -ETIMEDOUT;

	return 0;
}

static int nexell_adc_v2_adc_con(struct nexell_adc_info *adc)
{
	unsigned int adcon = 0;
	unsigned int pres = 0;
	void __iomem *reg = adc->adc_base;

	adcon = ADC_V2_CON_DATA_SEL(ADC_V2_DATA_SEL_VAL) |
		ADC_V2_CON_CLK_CNT(ADC_V2_CLK_CNT_VAL);
	adcon &= ~ADC_V2_CON_STBY;
	writel(adcon, ADC_V2_CON(reg));

	pres = ADC_V2_PRESCON_PRES(adc->prescale);
	writel(pres, ADC_V2_PRESCON(reg));
	pres |= ADC_V2_PRESCON_APEN;
	writel(pres, ADC_V2_PRESCON(reg));

	/* *****************************************************
	 * Turn-around invalid value after Power On
	 * *****************************************************/
	nexell_adc_v2_read_polling(adc, 0);
	adc->value = 0;

	writel(ADC_V2_INTCLR_CLR, ADC_V2_INTCLR(reg));
	writel(ADC_V2_INTENB_ENB, ADC_V2_INTENB(reg));
	init_completion(&adc->completion);

	return 0;
}

static int nexell_adc_v2_read_val(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val,
		int *val2,
		long mask)
{
	struct nexell_adc_info *adc = iio_priv(indio_dev);
	void __iomem *reg = adc->adc_base;
	unsigned long timeout;
	int ret = 0;

	reinit_completion(&adc->completion);

	nexell_adc_v2_ch_start(reg, chan->address);

	timeout = wait_for_completion_timeout(&adc->completion, ADC_TIMEOUT);
	if (timeout == 0) {
		dev_warn(&indio_dev->dev,
				"Conversion timed out! resetting...\n");
		if (adc->rst)
			reset_control_reset(adc->rst);
		setup_adc_con(adc);
		ret = -ETIMEDOUT;
	}

	return ret;
}

static const struct nexell_adc_data nexell_adc_s5p6818_data = {
	.version	= 2,
	.num_channels	= MAX_ADC_V2_CHANNELS,
	.channels	= nexell_adc_channels,
	.adc_con	= nexell_adc_v2_adc_con,
	.read_polling	= nexell_adc_v2_read_polling,
	.read_val	= nexell_adc_v2_read_val,
};

static const struct nexell_adc_data nexell_adc_nxp3220_data = {
	.version	= 3,
	.num_channels	= MAX_ADC_V2_CHANNELS,
	.channels	= nexell_adc_v3_channels,
	.adc_con	= nexell_adc_v2_adc_con,
	.read_polling	= nexell_adc_v2_read_polling,
	.read_val	= nexell_adc_v2_read_val,
};

#ifdef CONFIG_OF
static const struct of_device_id nexell_adc_match[] = {
	{
		.compatible = "nexell,nxp3220-adc",
		.data = &nexell_adc_nxp3220_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nexell_adc_match);

static struct nexell_adc_data *nexell_adc_get_data(struct platform_device *pdev)
{
	const struct of_device_id *match;

	match = of_match_node(nexell_adc_match, pdev->dev.of_node);
	return (struct nexell_adc_data *)match->data;
}
#endif

/*
 * ADC functions
 */
static int nexell_adc_setup(struct nexell_adc_info *adc,
		struct platform_device *pdev)
{
	ulong min_rate;
	uint32_t sample_rate;
	int prescale = 0;

	of_property_read_u32(pdev->dev.of_node, "sample_rate", &sample_rate);

	prescale = (adc->clk_rate) / (sample_rate * ADC_MAX_SAMPLE_BITS);
	min_rate = (adc->clk_rate) / (ADC_MAX_PRESCALE * ADC_MAX_SAMPLE_BITS);

	if (sample_rate > ADC_MAX_SAMPLE_RATE ||
			min_rate > sample_rate) {
		dev_err(&pdev->dev, "not support %u(%d ~ %lu) sample rate\n",
			sample_rate, ADC_MAX_SAMPLE_RATE, min_rate);
		return -EINVAL;
	}

	adc->sample_rate = sample_rate;
	adc->max_sample_rate = ADC_MAX_SAMPLE_RATE;
	adc->min_sample_rate = min_rate;
	adc->prescale = prescale;

	setup_adc_con(adc);

	dev_info(&pdev->dev, "CHs %d, %ld(%ld ~ %ld) sample rate, scale=%d(bit %d)\n",
		adc->data->num_channels,
		adc->sample_rate,
		adc->max_sample_rate, adc->min_sample_rate,
		prescale, ADC_MAX_SAMPLE_BITS);

	return 0;
}

static int nexell_adc_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val,
		int *val2,
		long mask)
{
	struct nexell_adc_info *adc = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);

	if (adc->data->read_val) {
		ret = adc->data->read_val(indio_dev, chan, val, val2, mask);
		if (ret < 0)
			goto out;
	}

	*val = adc->value;
	*val2 = 0;
	ret = IIO_VAL_INT;

	dev_dbg(&indio_dev->dev, "ch=%d, val=0x%x\n", chan->channel, *val);

out:
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_info nexell_adc_iio_info = {
	.read_raw = &nexell_adc_read_raw,
	.driver_module = THIS_MODULE,
};

static int nexell_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int nexell_adc_resume(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct nexell_adc_info *adc = iio_priv(indio_dev);

	if (adc->rst)
		reset_control_reset(adc->rst);

	setup_adc_con(adc);

	return 0;
}

static inline bool nexell_ts_get_detect(struct nexell_adc_info *adc)
{
	return ((readl(ADC_V3_TOUCHSCREENCON(adc->adc_base)) >> 5) & 0x1);
}

static void nexell_adc_ts_v3_detect_mode(struct nexell_adc_info *adc, bool on)
{
	unsigned int reg_value = 0;

	if (on)
		reg_value &= ~ADC_V3_TS_PULLUP_DISABLE;
	else
		reg_value |= ADC_V3_TS_PULLUP_DISABLE;

	reg_value &= ~ADC_V3_TS_XM_ON;
	reg_value |= (ADC_V3_TS_XP_OFF | ADC_V3_TS_YP_OFF | ADC_V3_TS_YM_ON);

	writel(reg_value, ADC_V3_TOUCHSCREENCON(adc->adc_base));

	mdelay(2);
}

static int nexell_adc_ts_v3_read_xy(struct nexell_adc_info *adc, int *x, int *y)
{
	struct nexell_adc_ts *tsc;
	struct iio_dev *indio_dev;
	struct iio_chan_spec *chan;
	unsigned int tx[TS_COORDINATE_CNT], ty[TS_COORDINATE_CNT];
	unsigned int reg_value, tval;
	int i, diff_x, diff_y;

	*x  = 0, *y = 0;

	tsc = ((struct nexell_adc_ts *)adc->tsc);
	indio_dev = ((struct iio_dev *)tsc->iio);

	reg_value = ADC_V3_TS_PULLUP_DISABLE;
	reg_value &= ~(ADC_V3_TS_XP_OFF | ADC_V3_TS_YM_ON);
	reg_value |= (ADC_V3_TS_YP_OFF | ADC_V3_TS_XM_ON);
	writel(reg_value, ADC_V3_TOUCHSCREENCON(adc->adc_base));

	/* It takes time to turn on/off the pull-up and switch. */
	mdelay(1);

	for (i = 0; i < TS_COORDINATE_CNT; i++) {
		chan = ((struct iio_chan_spec *)
				&nexell_adc_v3_channels[NX_ADC_CH_TS_YP - 2]);
		nexell_adc_read_raw(indio_dev, chan, &tx[i], &tval, 0xFFFF);
		*x += tx[i];
		if (tx[i] < 0)
			pr_warn("fail, read the adc-ch:%d value!!\n",
				NX_ADC_CH_TS_YP);
	}
	*x /= TS_COORDINATE_CNT;

	reg_value = ADC_V3_TS_PULLUP_DISABLE;
	reg_value &= ~(ADC_V3_TS_YP_OFF | ADC_V3_TS_XM_ON);
	reg_value |= (ADC_V3_TS_XP_OFF | ADC_V3_TS_YM_ON);
	writel(reg_value, ADC_V3_TOUCHSCREENCON(adc->adc_base));

	/* It takes time to turn on/off the pull-up and switch. */
	mdelay(1);

	for (i = 0; i < TS_COORDINATE_CNT; i++) {
		chan = ((struct iio_chan_spec *)
				&nexell_adc_v3_channels[NX_ADC_CH_TS_XP - 2]);
		nexell_adc_read_raw(indio_dev, chan, &ty[i], &tval, 0xFFFF);
		*y += ty[i];
		if (ty[i] < 0)
			pr_warn("fail, read the adc-ch:%d value!!\n",
				NX_ADC_CH_TS_XP);
	}
	*y /= TS_COORDINATE_CNT;

	/* Discard values with large deviations. */
	if ((tsc->pre_xpos != 0) && (tsc->pre_ypos != 0)) {
		diff_x = ((tsc->pre_xpos > *x)
			? (tsc->pre_xpos - *x) : (*x - tsc->pre_xpos));
		diff_y = ((tsc->pre_ypos > *y)
			? (tsc->pre_ypos - *y) : (*y - tsc->pre_ypos));

		if (!((diff_x >= TS_CORRECTION_VALUE)
			|| (diff_y >= TS_CORRECTION_VALUE))) {
			tsc->pre_xpos = *x;
			tsc->pre_ypos = *y;
		} else {
			*x = tsc->pre_xpos;
			*y = tsc->pre_ypos;
		}
	} else {
		tsc->pre_xpos = *x;
		tsc->pre_ypos = *y;
	}

	if ((*x < 0) || (*y < 0))
		return -1;

	return 0;
}

static void nexell_adc_ts_v3_thread(struct work_struct *data)
{
	struct nexell_adc_ts *tsc
		= container_of(data, struct nexell_adc_ts, work);
	struct nexell_adc_info *adc = iio_priv(tsc->iio);
	struct input_dev *inp = adc->tsc->inp;
	unsigned int reg_value;
	int x, y;

	while (1) {
		nexell_adc_ts_v3_detect_mode(adc, true);

		if (nexell_ts_get_detect(adc)) {
			tsc->pendown = TS_PENUP;
			break;
		}

		if (nexell_adc_ts_v3_read_xy(adc, &x, &y) < 0)
			continue;

		input_report_abs(inp, ABS_X, (x & ADC_DATX_MASK));
		input_report_abs(inp, ABS_Y, (y & ADC_DATY_MASK));
		input_report_abs(inp, ABS_PRESSURE, 1);
		input_report_key(inp, BTN_TOUCH, 1);
		input_sync(inp);

		msleep(tsc->ts_sample_time);
	}

	tsc->pre_xpos = 0;
	tsc->pre_ypos = 0;

	if (tsc->pendown == TS_PENUP) {
		input_report_key(inp, BTN_TOUCH, 0);
		input_report_abs(inp, ABS_PRESSURE, 0);
		input_sync(inp);
	}

	reg_value = readl(ADC_V2_INTENB(adc->adc_base));
	reg_value |= (ADC_V3_TS_INTENB | tsc->reg_value_backup);
	writel(reg_value, ADC_V2_INTENB(adc->adc_base));
}

static irqreturn_t nexell_adc_ts_v3_isr(int irq, void *dev_id)
{
	struct nexell_adc_info *adc
		= ((struct nexell_adc_info *)dev_id);
	unsigned int reg_value;

	/* Check the Touchscreen Interrupt Pending */
	reg_value = ((readl(ADC_V2_INTCLR(adc->adc_base)) >> 1) & 0x1);
	if (reg_value) {
		writel(ADC_V3_TS_INTCLR, ADC_V2_INTCLR(adc->adc_base));

		reg_value = readl(ADC_V2_INTENB(adc->adc_base));
		adc->tsc->reg_value_backup = reg_value;
		reg_value &= ~ADC_V3_TS_INTENB;
		writel(reg_value, ADC_V2_INTENB(adc->adc_base));

		adc->tsc->pendown = TS_PENDOWN;

		schedule_work(&adc->tsc->work);
	} else {
		writel(ADC_V2_INTCLR_CLR, ADC_V2_INTCLR(adc->adc_base));

		adc->value = readl(ADC_V2_DAT(adc->adc_base));

		complete(&adc->completion);
	}

	return IRQ_HANDLED;
}

static int nexell_adc_ts_open(struct input_dev *inp)
{
	struct nexell_adc_info *adc = input_get_drvdata(inp);

	nexell_adc_ts_v3_detect_mode(adc, true);

	return 0;
}

static void nexell_adc_ts_close(struct input_dev *inp)
{
	struct nexell_adc_info *adc = input_get_drvdata(inp);

	nexell_adc_ts_v3_detect_mode(adc, false);
}

static void nexell_adc_ts_init(struct platform_device *pdev)
{
	struct iio_dev *iio = platform_get_drvdata(pdev);
	struct nexell_adc_info *adc = iio_priv(iio);
	unsigned int reg_value;

	nexell_adc_ts_v3_detect_mode(adc, true);

	reg_value = readl(ADC_V2_INTENB(adc->adc_base));
	reg_value |= (ADC_V3_TS_INTENB | ADC_V2_INTENB_ENB);
	writel(reg_value, ADC_V2_INTENB(adc->adc_base));
}

static int nexell_adc_ts_probe(struct platform_device *pdev)
{
	struct iio_dev *iio = platform_get_drvdata(pdev);
	struct nexell_adc_info *adc = iio_priv(iio);
	struct nexell_adc_ts *tsc;
	struct input_dev *inp;
	int ret = 0;

	tsc = devm_kzalloc(&pdev->dev, sizeof(*tsc), GFP_KERNEL);
	if (!tsc)
		return -ENOMEM;

	inp = devm_input_allocate_device(&pdev->dev);
	if (!inp)
		return -ENOMEM;

	inp->name	= "Nexell Touchscreen";
	inp->phys	= "nexell/event0";
	inp->open	= nexell_adc_ts_open;
	inp->close	= nexell_adc_ts_close;
	inp->dev.parent = &pdev->dev;

	inp->id.bustype = BUS_HOST;
	inp->id.vendor  = 0x0001;
	inp->id.product = 0x0001;
	inp->id.version = 0x0100;

	inp->absbit[0] = BIT(ABS_X) | BIT(ABS_Y);
	inp->evbit[0]  = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	inp->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(inp, ABS_X, X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(inp, ABS_Y, Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
	input_set_abs_params(inp, ABS_PRESSURE, 0, 1, 0, 0);
	input_set_abs_params(inp, ABS_TOOL_WIDTH, 0, 1, 0, 0);

	input_set_drvdata(inp, tsc);
	ret = input_register_device(inp);
	if (ret) {
		dev_err(&pdev->dev, "fail, %s register for input device ...\n",
			pdev->name);
		goto err_dev;
	}

	tsc->dev = &pdev->dev;
	tsc->inp = inp;
	tsc->iio = iio;
	adc->tsc = tsc;

	nexell_adc_ts_init(pdev);

	ret = of_property_read_u32(pdev->dev.of_node, "ts-sample-time",
				   &tsc->ts_sample_time);
	if (ret)
		tsc->ts_sample_time = 4;

	INIT_WORK(&tsc->work, nexell_adc_ts_v3_thread);

	return 0;

err_dev:
	input_free_device(inp);

	return ret;
}

static int nexell_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *iio = NULL;
	struct nexell_adc_info *adc = NULL;
	struct resource	*mem;
	struct device_node *np = pdev->dev.of_node;
	int irq, ret = -ENODEV;

	if (!np)
		return ret;

	iio = devm_iio_device_alloc(&pdev->dev, sizeof(struct nexell_adc_info));
	if (!iio) {
		dev_err(&pdev->dev, "failed allocating iio ADC device\n");
		return -ENOMEM;
	}

	adc = iio_priv(iio);

	adc->data = nexell_adc_get_data(pdev);
	if (!adc->data) {
		dev_err(&pdev->dev, "failed getting nexell ADC data\n");
		return -EINVAL;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	adc->adc_base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(adc->adc_base))
		return PTR_ERR(adc->adc_base);

	/* setup: clock */
	adc->clk = devm_clk_get(&pdev->dev, "adc");
	if (IS_ERR(adc->clk)) {
		dev_err(&pdev->dev, "failed getting clock for ADC\n");
		return PTR_ERR(adc->clk);
	}
	adc->clk_rate = clk_get_rate(adc->clk);
	clk_prepare_enable(adc->clk);

	/* setup: reset */
	if (adc->data->version != 3) {
		adc->rst = devm_reset_control_get(&pdev->dev, "adc-reset");
		if (IS_ERR(adc->rst)) {
			dev_err(&pdev->dev, "failed to get reset\n");
			return PTR_ERR(adc->rst);
		}

		reset_control_reset(adc->rst);
	}

	/* setup: adc */
	ret = nexell_adc_setup(adc, pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed setup iio ADC device\n");
		goto err_unprepare_clk;
	}

	/* setup: irq */
	if (adc->data->version >= 2) {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0) {
			dev_err(&pdev->dev, "failed get irq resource\n");
			goto err_unprepare_clk;
		}

		ret = devm_request_irq(&pdev->dev, irq,
			nexell_adc_ts_v3_isr, 0, dev_name(&pdev->dev), adc);

		if (ret < 0) {
			dev_err(&pdev->dev, "failed get irq (%d)\n", irq);
			goto err_unprepare_clk;
		}

		adc->irq = irq;
	}

	platform_set_drvdata(pdev, iio);

	iio->name = dev_name(&pdev->dev);
	iio->dev.parent = &pdev->dev;
	iio->info = &nexell_adc_iio_info;
	iio->modes = INDIO_DIRECT_MODE;
	iio->channels = adc->data->channels;
	iio->num_channels = adc->data->num_channels;
	iio->dev.of_node = pdev->dev.of_node;

	/*
	 * sys interface : user interface
	 */
	ret = devm_iio_device_register(&pdev->dev, iio);
	if (ret)
		goto err_unprepare_clk;

	ret = of_platform_populate(np, nexell_adc_match, NULL, &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed adding child nodes\n");
		goto err_of_populate;
	}

	dev_dbg(&pdev->dev, "ADC init success\n");

	/* parsing the touchscreen option */
	if (of_property_read_bool(np, "touchscreen-enable")) {
		ret = nexell_adc_ts_probe(pdev);
		if (ret < 0)
			goto err_of_populate;
	}

	return ret;

err_of_populate:
	device_for_each_child(&pdev->dev, NULL,
			nexell_adc_remove_devices);
err_unprepare_clk:
	clk_disable_unprepare(adc->clk);

	return ret;
}

static int nexell_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *iio = platform_get_drvdata(pdev);
	struct nexell_adc_info *adc = iio_priv(iio);

	device_for_each_child(&pdev->dev, NULL,
			nexell_adc_remove_devices);
	clk_disable_unprepare(adc->clk);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver nexell_adc_driver = {
	.probe		= nexell_adc_probe,
	.remove		= nexell_adc_remove,
	.suspend	= nexell_adc_suspend,
	.resume		= nexell_adc_resume,
	.driver		= {
		.name	= "nexell-adc",
		.owner	= THIS_MODULE,
		.of_match_table = nexell_adc_match,
	},
};

#if defined(CONFIG_DEFERRED_ADC) || defined(CONFIG_DEFERRED_UP_ADC)
static int __init nexell_adc_driver_init(void)
{
	return platform_driver_register(&nexell_adc_driver);
}
#if CONFIG_DEFERRED_LEVEL == 1
deferred_module_init(nexell_adc_driver_init)
#elif CONFIG_DEFERRED_LEVEL == 2
subsys_initcall(nexell_adc_driver_init);
#endif
#else
module_platform_driver(nexell_adc_driver);
#endif

MODULE_AUTHOR("Bon-gyu, KOO <freestyle@nexell.co.kr>");
MODULE_DESCRIPTION("ADC driver for the Nexell SoC");
MODULE_LICENSE("GPL v2");
