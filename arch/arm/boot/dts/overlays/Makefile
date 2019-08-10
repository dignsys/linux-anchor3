ifeq ($(CONFIG_OF_OVERLAY),y)

dtbo-$(CONFIG_ARCH_NEXELL) += \
	sip-s31nx-artik310s-explorer-spi.dtbo \
	sip-s31nx-artik310s-explorer-rgb.dtbo \
	sip-s31nx-artik310s-explorer-tw9900.dtbo \
	sip-s31nx-artik310s-trike-rgb.dtbo \
	sip-s31nx-artik310s-trike-uart4.dtbo \
	sip-s31nx-artik310s-trike-uart5.dtbo \
	sip-s31nx-artik310s-trike-spi0.dtbo \
	sip-s31nx-artik310s-trike-rs4851.dtbo \
	sip-s31nx-artik310s-trike-eth1.dtbo \
	sip-s31nx-artik310s-trike-i2s2.dtbo \
	sip-s31nx-artik310s-trike-pdm.dtbo \
	sip-s31nx-artik310s-trike-pdm1.dtbo \
	sip-s31nx-artik310s-trike-otg.dtbo

targets	+= dtbs dtbs_install
targets	+= $(dtb-y)

endif

always		:= $(dtbo-y)
clean-files	:= *.dtbo
