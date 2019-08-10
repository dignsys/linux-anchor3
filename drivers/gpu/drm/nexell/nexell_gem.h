// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell DRM driver
 * Copyright (c) 2018 JungHyun Kim <jhkim@nexell.co.kr>
 */

#ifndef _NEXELL_DRM_GEM_H_
#define _NEXELL_DRM_GEM_H_

#include <drm/drm_gem.h>

/*
 * nexell drm gem object
 */
struct nx_gem_object {
	struct drm_gem_object base;
	dma_addr_t dma_addr;
	struct sg_table *sgt;		 /* for system memory */
	void *cpu_addr;
	size_t size;
	uint32_t flags;
	struct sg_table *import_sgt; /* for prime import */
	struct mutex lock;
	struct page **pages;
	struct list_head vmas;
};

static inline struct nx_gem_object *to_nx_gem_obj(struct drm_gem_object *obj)
{
	return container_of(obj, struct nx_gem_object, base);
}

static inline struct drm_gem_object *to_gem_obj(struct nx_gem_object *nx_obj)
{
	return &nx_obj->base;
}

/*
 * framebuffer with gem
 */
struct nx_gem_object *nx_drm_fb_gem(struct drm_framebuffer *fb,
				    unsigned int plane);

/*
 * struct nx_gem_object elements
 */
struct nx_gem_object *nx_drm_gem_create(struct drm_device *drm,
					size_t size, unsigned int flags);
void nx_drm_gem_destroy(struct nx_gem_object *nx_obj);

/*
 * struct drm_driver elements
 */
int nx_drm_gem_dumb_create(struct drm_file *file_priv,
			   struct drm_device *dev,
			   struct drm_mode_create_dumb *args);
int nx_drm_gem_dumb_map_offset(struct drm_file *file_priv,
			       struct drm_device *dev, uint32_t handle,
			       uint64_t *offset);
void nx_drm_gem_free_object(struct drm_gem_object *obj);

struct dma_buf *nx_drm_gem_prime_export(struct drm_device *drm,
					struct drm_gem_object *obj,
					int flags);
struct drm_gem_object *nx_drm_gem_prime_import(struct drm_device *drm,
					       struct dma_buf *dma_buf);
int nx_drm_gem_prime_mmap(struct drm_gem_object *obj,
			  struct vm_area_struct *vma);
void *nx_drm_gem_prime_vmap(struct drm_gem_object *obj);
void nx_drm_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr);

struct sg_table *nx_drm_gem_prime_get_sg_table(struct drm_gem_object *obj);
struct drm_gem_object *nx_drm_gem_prime_import_sg_table(
				struct drm_device *dev,
				struct dma_buf_attachment *attach,
				struct sg_table *sgt);

dma_addr_t nx_drm_gem_get_dma_addr(struct drm_device *drm, unsigned int handle,
				    struct drm_file *file_priv);

/* struct file_operations elements */
int nx_drm_gem_fops_mmap(struct file *filp, struct vm_area_struct *vma);

/*
 * struct drm_ioctl_desc
 */
int nx_drm_gem_create_ioctl(struct drm_device *drm, void *data,
			    struct drm_file *file_priv);
int nx_drm_gem_sync_ioctl(struct drm_device *drm, void *data,
			  struct drm_file *file_priv);
int nx_drm_gem_get_ioctl(struct drm_device *drm, void *data,
			 struct drm_file *file_priv);

/*
 * gem fence
 */
int nx_drm_gem_wait_fence(struct drm_gem_object *obj);

#endif
