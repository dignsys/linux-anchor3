// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Jongkeun, Choi <jkchoi@nexell.co.kr>
 */

#ifndef __DT_BINDINGS_MEDIA_NXP3220_VIP_H__
#define __DT_BINDINGS_MEDIA_NXP3220_VIP_H__

/* capture hw interface type */
#define NX_CAPTURE_INTERFACE_PARALLEL		0
#define NX_CAPTURE_INTERFACE_MIPI_CSI		1

/* camera sensor <--> vip data order(yuv422) */
#define NX_VIN_CBY0CRY1				0
#define NX_VIN_CRY1CBY0				1
#define NX_VIN_Y0CBY1CR				2
#define NX_VIN_Y1CRY0CB				3

/* camera sensor configuration interface */
#define NX_CAPTURE_SENSOR_I2C			0
#define NX_CAPTURE_SENSOR_SPI			1
#define NX_CAPTURE_SENSOR_LOOPBACK		2

/* camera enable sequence marker */
#define NX_ACTION_START				0x12345678
#define NX_ACTION_END				0x87654321
#define NX_ACTION_TYPE_GPIO			0xffff0001
#define NX_ACTION_TYPE_PMIC			0xffff0002
#define NX_ACTION_TYPE_CLOCK			0xffff0003

#endif /* __DT_BINDINGS_MEDIA_NXP3220_V4L2_H__ */
