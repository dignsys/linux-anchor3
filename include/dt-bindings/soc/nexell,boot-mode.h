/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __NEXELL_BOOT_MODE_H
#define __NEXELL_BOOT_MODE_H

/* high 24 bits(0x6e7870) is tag, low 8 bits is type of reboot */
#define REBOOT_MAGIC		0x6e787000

/* normal boot */
#define BOOT_NORMAL		(REBOOT_MAGIC + 0)

/* enter fastboot mode */
#define BOOT_FASTBOOT		(REBOOT_MAGIC + 1)

/* enter recovery mode */
#define BOOT_RECOVERY		(REBOOT_MAGIC + 2)

/* enter dfu downloader mode */
#define BOOT_DFU		(REBOOT_MAGIC + 3)

#endif
