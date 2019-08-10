// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/soc/nexell/sec_io.h>

#define CHIPNAME_LEN	48
#define EFUSE_SECURE	(0x20070000)
#define HPM_IDS0	0x530
#define HPM_IDS1	0x534
#define HPM_IDS2	0x538
#define HPM_IDS3	0x53C

struct nx_ecid_regs {
	u8 chipname[CHIPNAME_LEN]; /* 0x00 */
	u32 __reserved_0x30;
	u32 guid0; /* 0x34 */
	u16 guid1; /* 0x38 */
	u16 guid2; /* 0x3a */
	u8 guid3[8]; /* 0x3c */
	u32 ec0; /* 0x44 */
	u32 __reserved_0x48;
	u32 ec2; /* 0x4c */
	u32 __reserved_0x50[(0x100 - 0x50) / 4];
	u32 ecid[4]; /* 0x100 */
};

struct nx_guid {
	u32 guid0;
	u16 guid1;
	u16 guid2;
	u8 guid3[8];
};

struct nx_ecid_mod {
	struct nx_ecid_regs *base;
};

static struct nx_ecid_mod *ecid_mod = &(struct nx_ecid_mod) {
	.base = NULL,
};

static unsigned int convertmsblsb(uint32_t data, int bits)
{
	uint32_t result = 0;
	uint32_t mask = 1;
	int i = 0;

	for (i = 0; i < bits ; i++) {
		if (data & (1 << i))
			result |= mask << (bits - i - 1);
	}
	return result;
}

static const char gst36Strtable[36] = {
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
	'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
	'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S',
	'T', 'U', 'V', 'W', 'X', 'Y', 'Z'
};

static void lotid_num2string(uint32_t lot_id, char str[6])
{
	uint32_t value[3];
	uint32_t mad[3];

	value[0] = lot_id / 36;
	mad[0] = lot_id % 36;

	value[1] = value[0] / 36;
	mad[1] = value[0] % 36;

	value[2] = value[1] / 36;
	mad[2] = value[1]  % 36;

	str[0] = 'S';
	str[1] = gst36Strtable[value[2]];
	str[2] = gst36Strtable[mad[2]];
	str[3] = gst36Strtable[mad[1]];
	str[4] = gst36Strtable[mad[0]];
	str[5] = '\0';
}

static int nx_ecid_get_key_ready(void)
{
	const u32 ready_pos = 16; /* sense done */
	const u32 ready_mask = 1ul << ready_pos;

	u32 regval = ecid_mod->base->ec2;

	return (int)((regval & ready_mask) >> ready_pos);
}

static void nx_ecid_get_chip_name(u8 *chip_name)
{
	int i;
	u8 *c = chip_name;

	for (i = 0; i < CHIPNAME_LEN; i++)
		c[i] = ecid_mod->base->chipname[i];

	for (i = CHIPNAME_LEN - 1; i >= 0; i--) {
		if (c[i] != '-')
			break;
		c[i] = 0;
	}
}

static void nx_ecid_get_ecid(u32 ecid[4])
{
	ecid[0] = ecid_mod->base->ecid[0];
	ecid[1] = ecid_mod->base->ecid[1];
	ecid[2] = ecid_mod->base->ecid[2];
	ecid[3] = ecid_mod->base->ecid[3];
}

static void nx_ecid_get_guid(struct nx_guid *guid)
{
	guid->guid0 = ecid_mod->base->guid0;
	guid->guid1 = ecid_mod->base->guid1;
	guid->guid2 = ecid_mod->base->guid2;
	guid->guid3[0] = ecid_mod->base->guid3[0];
	guid->guid3[1] = ecid_mod->base->guid3[1];
	guid->guid3[2] = ecid_mod->base->guid3[2];
	guid->guid3[3] = ecid_mod->base->guid3[3];
	guid->guid3[4] = ecid_mod->base->guid3[4];
	guid->guid3[5] = ecid_mod->base->guid3[5];
	guid->guid3[6] = ecid_mod->base->guid3[6];
	guid->guid3[7] = ecid_mod->base->guid3[7];
}

static void nx_efuse_get_hpm_ro(u32 hpm[4])
{
	void __iomem *reg = (void __iomem *)EFUSE_SECURE;

	hpm[0] = sec_readl(reg + HPM_IDS0);
	hpm[1] = sec_readl(reg + HPM_IDS1);
	hpm[2] = sec_readl(reg + HPM_IDS2);
	hpm[3] = sec_readl(reg + HPM_IDS2);
}

static int wait_key_ready(void)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(20);

	while (!nx_ecid_get_key_ready()) {
		if (time_after(jiffies, timeout)) {
			if (nx_ecid_get_key_ready())
				break;

			pr_err("Error: key not ready\n");

			return -EINVAL;
		}
		cpu_relax();
	}
	return 0;
}

static int nx_cpu_id_guid(u32 guid[4])
{
	if (wait_key_ready() < 0)
		return -EBUSY;

	nx_ecid_get_guid((struct nx_guid *)guid);

	return 0;
}

static int nx_cpu_id_ecid(u32 ecid[4])
{
	if (wait_key_ready() < 0)
		return -EBUSY;

	nx_ecid_get_ecid(ecid);

	return 0;
}

static int nx_cpu_hpm_ro(u16 hpm[8])
{
	u32 _hpm[4];

	if (wait_key_ready() < 0)
		return -EBUSY;

	nx_efuse_get_hpm_ro(_hpm);

	hpm[0] = (_hpm[0] >> 0) & 0x3ff;
	hpm[1] = (_hpm[0] >> 10) & 0x3ff;
	hpm[2] = (_hpm[0] >> 20) & 0x3ff;
	hpm[3] = ((_hpm[0] >> 30) & 0x3) | ((_hpm[1] & 0xff) << 2);
	hpm[4] = (_hpm[1] >> 8) & 0x3ff;
	hpm[5] = (_hpm[1] >> 18) & 0x3ff;
	hpm[6] = ((_hpm[1] >> 28) & 0xf) | ((_hpm[2] & 0x3f) << 4);
	hpm[7] = (_hpm[2] >> 6) & 0x3ff;

	return 0;
}

static int nx_cpu_id_string(u8 *chipname)
{
	if (wait_key_ready() < 0)
		return -EBUSY;

	nx_ecid_get_chip_name(chipname);

	return 0;
}

/* Notify cpu GUID: /sys/devices/platform/cpu,  guid, uuid,  name  */
static ssize_t sys_id_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct attribute *at = &attr->attr;
	char *s = buf;
	u32 uid[4] = {0, };
	u8 chipname[CHIPNAME_LEN + 1] = {0, };
	int string = 0;
	int ret = 0;

	pr_debug("[%s : name =%s ]\n", __func__, at->name);

	if (!strncmp(at->name, "uuid", 4)) {
		ret = nx_cpu_id_ecid(uid);
	} else if (!strncmp(at->name, "guid", 4)) {
		ret = nx_cpu_id_guid(uid);
	} else if (!strncmp(at->name, "name", 4)) {
		ret = nx_cpu_id_string(chipname);
		string = 1;
	} else {
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	if (string) {
		if (isprint(chipname[0])) {
			s += snprintf(s, sizeof(chipname), "%s\n", chipname);
		} else {
			#define _W	(12)	/* width */
			int i;

			for (i = 0; i < CHIPNAME_LEN; i++) {
				s += snprintf(s, 2, "%02x", chipname[i]);
				if ((i + 1) % _W == 0)
					s += snprintf(s, 1, " ");
			}
			s += snprintf(s, 1, "\n");
		}
	} else {
		s += snprintf(s, 36, "%08x:%08x:%08x:%08x\n",
				uid[0], uid[1], uid[2], uid[3]);
	}

	if (s != buf)
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t sys_ids_show(struct device *pdev, struct device_attribute *attr,
			    char *buf)
{
	struct attribute *at = &attr->attr;
	char *s = buf;
	u32 uid[4] = {0, };
	int ret = 0;

	u32 cpu_ids;
	u32 core_ids;
	int cpu_frac, cpu_inte;
	int core_frac, core_inte;
	int len;

	pr_debug("[%s : name =%s ]\n", __func__, at->name);

	ret = nx_cpu_id_ecid(uid);
	if (ret < 0)
		return ret;

	cpu_ids = (uid[1] >> 16) & 0xff;
	core_ids = (uid[1] >> 24) & 0xff;

	cpu_frac = ((uid[1] >> 16) & 0x3) * 25;
	cpu_inte = ((uid[1] >> 18) & 0x3f);

	core_frac = ((uid[1] >> 24) & 0x3) * 25;
	core_inte = ((uid[1] >> 26) & 0x3f);

	s += snprintf(s, 7, "%02x:%02x ", cpu_ids, core_ids);
	len = snprintf(NULL, 0, "cpu: %2d.%02d mA, ", cpu_inte, cpu_frac);
	s += snprintf(s, len + 1, "cpu: %2d.%02d mA, ", cpu_inte, cpu_frac);
	len = snprintf(NULL, 0, "core: %2d.%02d mA\n", core_inte, core_frac);
	s += snprintf(s, len + 1, "core: %2d.%02d mA\n", core_inte, core_frac);

	if (s != buf)
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t sys_ro_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct attribute *at = &attr->attr;
	char *s = buf;
	u16 hpm[8] = {0, };
	int ret = 0;
	int len = 0;

	pr_debug("[%s : name =%s ]\n", __func__, at->name);

	ret = nx_cpu_hpm_ro(hpm);
	if (ret < 0)
		return ret;

	len = snprintf(NULL, 0, "%03x:%03x:%03x:%03x:%03x:%03x:%03x:%03x\n",
			hpm[0], hpm[1], hpm[2], hpm[3],
			hpm[4], hpm[5], hpm[6], hpm[7]);
	s += snprintf(s, len + 1, "%03x:%03x:%03x:%03x:%03x:%03x:%03x:%03x\n",
			hpm[0], hpm[1], hpm[2], hpm[3],
			hpm[4], hpm[5], hpm[6], hpm[7]);

	if (s != buf)
		*(s-1) = '\n';

	return (s - buf);
}


static struct device_attribute __guid__ =
			__ATTR(guid, 0444, sys_id_show, NULL);
static struct device_attribute __uuid__ =
			__ATTR(uuid, 0444, sys_id_show, NULL);
static struct device_attribute __name__ =
			__ATTR(name, 0444, sys_id_show, NULL);
static struct device_attribute __ids__ =
			__ATTR(ids, 0444, sys_ids_show, NULL);
static struct device_attribute __ro__ =
			__ATTR(ro, 0444, sys_ro_show, NULL);

static struct attribute *sys_attrs[] = {
	&__guid__.attr,
	&__uuid__.attr,
	&__name__.attr,
	&__ids__.attr,
	&__ro__.attr,
	NULL,
};

static struct attribute_group sys_attr_group = {
	.attrs = (struct attribute **)sys_attrs,
};

static const struct of_device_id nxp3220_ecid_match[] = {
	{ .compatible = "nexell,nxp3220-ecid" },
	{ /* sentinel */ }
};

static int __init cpu_sys_id_setup(void)
{
	const struct of_device_id *match;
	struct device_node *np;
	struct resource regs;

	struct kobject *kobj;
	u32 uid[4] = { 0, };
	int ret = 0;
	u32 lotid;
	char strlotid[6];

	np = of_find_matching_node_and_match(NULL, nxp3220_ecid_match, &match);
	if (!np) {
		pr_err("DT node not found. Failed to access ecid.\n");
		return -ENXIO;
	}

	if (of_address_to_resource(np, 0, &regs) < 0) {
		pr_err("failed to get ecid registers\n");
		of_node_put(np);
		return -ENXIO;
	}

	of_node_put(np);

	ecid_mod->base = (struct nx_ecid_regs *)
		ioremap_nocache(regs.start, resource_size(&regs));
	if (!ecid_mod->base) {
		pr_err("failed to map ecid module registers\n");
		return -ENXIO;
	}

	/*
	 * create interfaces
	 */
	kobj = kobject_create_and_add("cpu", &platform_bus.kobj);
	if (!kobj) {
		pr_err("Failed create cpu kernel object ....\n");
		return -ret;
	}

	ret = sysfs_create_group(kobj, &sys_attr_group);
	if (ret) {
		pr_err("Failed create cpu sysfs group ...\n");
		kobject_del(kobj);
		return -ret;
	}

	if (nx_cpu_id_ecid(uid) < 0)
		pr_err("FAIL: cannot get ecid !!!\n");

	lotid = convertmsblsb(uid[0] & 0x1FFFFF, 21);
	lotid_num2string(lotid, strlotid);

	pr_info("ECID: %08x:%08x:%08x:%08x\n", uid[0], uid[1], uid[2], uid[3]);
	pr_info("LOT ID : %s\n", strlotid);

	return ret;
}

static int __init cpu_sys_init_setup(void)
{
	cpu_sys_id_setup();

	return 0;
}
core_initcall(cpu_sys_init_setup);
