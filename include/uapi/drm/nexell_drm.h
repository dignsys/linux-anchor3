// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#ifndef _UAPI_NEXELL_DRM_H_
#define _UAPI_NEXELL_DRM_H_

#include <drm/drm.h>

/**
 * User-desired buffer creation information structure.
 *
 * @size: user-desired memory allocation size.
 *	- this size value would be page-aligned internally.
 * @flags: user request for setting memory type or cache attributes.
 * @handle: returned a handle to created gem object.
 *	- this handle will be set by gem module of kernel side.
 */
struct nx_gem_create {
	uint64_t size;
	unsigned int flags;
	unsigned int handle;
	void *priv_data;
};

/**
 * A structure to gem information.
 *
 * @handle: a handle to gem object created.
 * @flags: flag value including memory type and cache attribute and
 *	this value would be set by driver.
 * @size: size to memory region allocated by gem and this size would
 *	be set by driver.
 */
struct nx_gem_info {
	unsigned int handle;
	unsigned int flags;
	uint64_t size;
};

/*
 * nexell gem memory type
 */
enum nx_gem_type {
	/*
	 * DMA continuous memory
	 * user   : non-cacheable
	 * kernel : non-cacheable
	 */
	NEXELL_BO_DMA,

	/*
	 * DMA continuous memory, allocate from DMA,
	 * user   : cacheable
	 * kernel : non-cacheable
	 */
	NEXELL_BO_DMA_CACHEABLE,

	/*
	 * System continuous memory, allocate from system
	 * user   : non-cacheable
	 * kernel : non-cacheable
	 */
	NEXELL_BO_SYSTEM,

	/*
	 * System continuous memory, allocate from system
	 * user   : cacheable
	 * kernel : cacheable
	 */
	NEXELL_BO_SYSTEM_CACHEABLE,

	/*
	 * System non-continuous memory, allocate from system
	 * user   : non-cacheable
	 * kernel : non-cacheable
	 */
	NEXELL_BO_SYSTEM_NONCONTIG,

	/*
	 * System non-continuous memory, allocate from system
	 * user   : cacheable
	 * kernel : cacheable
	 */
	NEXELL_BO_SYSTEM_NONCONTIG_CACHEABLE,

	NEXELL_BO_MAX,
};

/**
 * G2D command type
 */
enum nx_g2d_cmd_type {
	NX_G2D_CMD_SRC_CTRL,
	NX_G2D_CMD_SRC_ADDR,
	NX_G2D_CMD_SRC_STRIDE,
	NX_G2D_CMD_SRC_BLKSIZE,
	NX_G2D_CMD_SOLID_COLOR,
	NX_G2D_CMD_SIZE,
	NX_G2D_CMD_DST_CTRL,
	NX_G2D_CMD_DST_ADDR,
	NX_G2D_CMD_DST_STRIDE,
	NX_G2D_CMD_DST_BLKSIZE,
	NX_G2D_CMD_BLEND_COLOR,
	NX_G2D_CMD_BLEND_EQUAT_ALPHA,
	NX_G2D_CMD_RUN,
	NX_G2D_CMD_NR,
};

/**
 * G2D buffer type
 */
enum nx_g2d_buf_type {
	NX_G2D_BUF_TYPE_NONE = 0,
	NX_G2D_BUF_TYPE_GEM = (1 << 0),
	NX_G2D_BUF_TYPE_CPU = (1 << 1),
};

/**
 * A structure to g2d buffer
 *
 * @handle: a handle to gem object created.
 * @offset: memory start offset from gem object handle.
 * @flags: flag value including memory type.
 */
struct nx_g2d_buf {
	__u32 handle;
	__u32 offset;
	enum nx_g2d_buf_type type;
};

/**
 * A structure to g2d ioctl command argument
 *
 * @cmd: g2d command array
 * @cmd_nr: g2d command count
 * @cmd_mask: mask to indicate valid command.
 * @src: source buffer info.
 * @dst: destination buffer info.
 */
#define	NX_G2D_CMD_MAX_SIZE	16

struct nx_g2d_cmd {
	__u32 cmd[NX_G2D_CMD_MAX_SIZE];
	__u32 cmd_nr;
	__u64 cmd_mask;
	struct nx_g2d_buf src;
	struct nx_g2d_buf dst;
	__u32 flags;
	/* returned a handle to g2d command */
	__u32 id;
};

struct nx_g2d_ver {
	int major;
	int minor;
};

/*
 * Nexell GEM ioctl
 */
#define DRM_NX_GEM_CREATE		0x00
/* Reserved 0x03 ~ 0x05 for nx specific gem ioctl */
#define DRM_NX_GEM_GET			0x04
#define DRM_NX_GEM_SYNC			0x05

/*
 * Nexell G2D ioctl
 */
#define DRM_NX_G2D_GET_VER		0x10
#define DRM_NX_G2D_DMA_EXEC		0x11
#define DRM_NX_G2D_DMA_SYNC		0x12

#define NX_IOWR(c, t)	DRM_IOWR(DRM_COMMAND_BASE + c, t)

#define DRM_IOCTL_NX_GEM_CREATE \
		NX_IOWR(DRM_NX_GEM_CREATE, struct nx_gem_create)
#define DRM_IOCTL_NX_GEM_SYNC \
		NX_IOWR(DRM_NX_GEM_SYNC, struct nx_gem_create)
#define DRM_IOCTL_NX_GEM_GET \
		NX_IOWR(DRM_NX_GEM_GET, struct nx_gem_info)

#define DRM_IOCTL_NX_G2D_GET_VER \
		NX_IOWR(DRM_NX_G2D_GET_VER, struct nx_g2d_ver)
#define DRM_IOCTL_NX_G2D_DMA_EXEC \
		NX_IOWR(DRM_NX_G2D_DMA_EXEC, struct nx_g2d_cmd)
#define DRM_IOCTL_NX_G2D_DMA_SYNC \
		NX_IOWR(DRM_NX_G2D_DMA_SYNC, struct nx_g2d_cmd)

#endif
