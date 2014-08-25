# Space Whistle - Wind Controller - Musical IoT

## A highly responsive wind controller with expressive continuous push buttons based on linear hall-effect sensors - A musical IoT device -

Wind controllers are fancy but expensive and their output signals have a too coarse resolution in time and their scope of application is too confined and they are not open, so we will build our own with some special twists. 

Instead of simple on/off buttons, we will use continuous push buttons (e-valves) instead with which we hope to be able to add a lot of expressiveness to our play. We will thus get a quasi continuous range of values per e-valve instead of a binary one. The e-valves will be based on linear hall-effect sensors and 8 of them with an accompanying respiratory air pressure sensor are interfaced to an ARM MCU. 

The device is fit for the future as all communication is network based. Events are serialized to Open Sound Control and sent via UDP/TCP to some host running a software synth. 

Finally the device could look like a networked electronic trumpet on stereoids and be the more most expressive wind controller out there, ready to record some fancy sci-fi movie themes.

<http://hackaday.io/project/2011>

## Build instructions

### ARM embedded toolchain
<https://launchpad.net/gcc-arm-embedded>

	cd $HOME
	tar xjf gcc-arm-none-eabi-4_8-2014q2-20140609-linux.tar.bz2
	export PATH="$PATH:$HOME/gcc-arm-none-eabi-4_8-2014q2/bin"

### LeafLabs libmaple F3 port
<https://github.com/ventosus/libmaple/tree/F3>

	cd $HOME
	git clone https://github.com/ventosus/libmaple.git
	cd libmaple
	git checkout -b F3 origin/F3
	export LIB_MAPLE_HOME=$HOME/libmaple</code></pre>

### Space Whistle firmware
<https://github.com/OpenMusicKontrollers/space_whistle_firmware>

	git clone https://github.com/OpenMusicKontrollers/space_whistle_firmware.git
	cd space_whistle_firmware
	make
