# LIB_MAPLE_HOME needs to be defined in your environment 

#export SENSORS := 48 # 3 sensor units aka Chimaera Mini
#export SENSORS := 96 # 6 sensor units aka Chimaera Midi
export SENSORS := 144 # 9 sensor units aka Chimaera Maxi

export CCACHE := ccache

export BOARD := F303CB
export MEMORY_TARGET := jtag
export USER_MODULES := $(shell pwd)

export DFU_VENDOR := 0x0483
export DFU_PRODUCT := 0xdf11

.DEFAULT_GOAL := sketch

all: sketch

%:
	$(MAKE) -f $(LIB_MAPLE_HOME)/Makefile $@

download:	build/$(BOARD).bin
	oscsend osc.udp://255.255.255.255:4444 /chimaera/reset/flash i 13
	sleep 1
	dfu-util -a 0 -d $(DFU_VENDOR):$(DFU_PRODUCT) -s 0x08000000:leave -D $<
