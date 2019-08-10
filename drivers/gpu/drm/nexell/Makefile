# SPDX-License-Identifier: GPL-2.0
# Nexell Platform Support

nexell_drm-y := nexell_crtc.o nexell_plane.o nexell_connector.o \
		nexell_encoder.o nexell_fb.o nexell_gem.o nexell_drv.o \
		nexell_subdrv.o
nexell_drm-y += soc/drm.o soc/display.o soc/low.o

nexell_drm-$(CONFIG_DRM_NEXELL_RGB)	+= soc/rgb.o
nexell_drm-$(CONFIG_DRM_NEXELL_LVDS) 	+= soc/lvds.o

nexell_drm-$(CONFIG_DRM_NEXELL_RGB)	+= nexell_lcd.o
nexell_drm-$(CONFIG_DRM_NEXELL_LVDS)	+= nexell_lcd.o
nexell_drm-$(CONFIG_DRM_NEXELL_G2D) 	+= nxp3220_g2d.o
obj-$(CONFIG_DRM_NEXELL)		+= nexell_drm.o
