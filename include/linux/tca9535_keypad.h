// SPDX-License-Identifier: GPL-2.0+
/*
 * tca9535 keypad platform support
 *
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 */

#ifndef _TCA9535_KEYS_H
#define _TCA9535_KEYS_H

#include <linux/types.h>

struct tca9535_button {
	int code;		/* input event code (KEY_*, SW_*) */
	int active_low;
	int type;		/* input event type (EV_KEY, EV_SW) */
	const char *desc;
	unsigned int irq;
};

struct tca9535_keys_platform_data {
	struct tca9535_button *buttons;
	int nbuttons;
	unsigned int rep:1;	/* enable input subsystem auto repeat */
	uint16_t pinmask;
	uint16_t invert;
	int irq_is_gpio;
	int use_polling;	/* polling if irq is not connected */
	const char *name;
};
#endif /* _TCA9535_KEYS_H */
