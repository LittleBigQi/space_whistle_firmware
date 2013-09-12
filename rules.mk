# Template rules.mk file.

# First, include the standard libmaple rules.mk header. Leave this
# line alone.
include $(MAKEDIR)/header.mk

###############################################################################

### Change this middle section for your project.

### Source subdirectories

# If any subdirectories contain source files, we have to add them to
# the variable BUILDDIRS, like this. $(BUILD_PATH) is the directory
# where compilation output (like object files) goes. The variable $(d)
# gets expanded to the directory containing this rules.mk file.
BUILDDIRS += $(BUILD_PATH)/$(d)/cmc
BUILDDIRS += $(BUILD_PATH)/$(d)/wiz
BUILDDIRS += $(BUILD_PATH)/$(d)/nosc
BUILDDIRS += $(BUILD_PATH)/$(d)/tuio2
BUILDDIRS += $(BUILD_PATH)/$(d)/dump
BUILDDIRS += $(BUILD_PATH)/$(d)/config
BUILDDIRS += $(BUILD_PATH)/$(d)/sntp
BUILDDIRS += $(BUILD_PATH)/$(d)/tube
BUILDDIRS += $(BUILD_PATH)/$(d)/eeprom
BUILDDIRS += $(BUILD_PATH)/$(d)/chimutil
BUILDDIRS += $(BUILD_PATH)/$(d)/chimaera
BUILDDIRS += $(BUILD_PATH)/$(d)/ipv4ll
BUILDDIRS += $(BUILD_PATH)/$(d)/mdns
BUILDDIRS += $(BUILD_PATH)/$(d)/dns_sd
BUILDDIRS += $(BUILD_PATH)/$(d)/dhcpc
BUILDDIRS += $(BUILD_PATH)/$(d)/rtpmidi
BUILDDIRS += $(BUILD_PATH)/$(d)/oscmidi
BUILDDIRS += $(BUILD_PATH)/$(d)/midi
BUILDDIRS += $(BUILD_PATH)/$(d)/dummy
BUILDDIRS += $(BUILD_PATH)/$(d)/scsynth
BUILDDIRS += $(BUILD_PATH)/$(d)/arp
BUILDDIRS += $(BUILD_PATH)/$(d)/calibration

### Local flags: these control how the compiler gets called.

# Here we set a variable for our project's include directory.  Note
# that the project top-level directory (i.e., the one containing this
# rules.mk file) is automatically used for include files, so you don't
# need to add it here.
EXAMPLE_INCLUDE_DIR := $(d)/include

# CFLAGS_$(d) are additional flags you want to give the C (not C++!)
# compiler. You at least want to have this, which provide the
# appropriate GCC -I switches to let you include libmaple headers.
CFLAGS_$(d) := $(WIRISH_INCLUDES) $(LIBMAPLE_INCLUDES)
# We'll also want our local include directory
CFLAGS_$(d) += -I$(EXAMPLE_INCLUDE_DIR)

# custom preprocessor flags
#CFLAGS_$(d) += -DBENCHMARK
CFLAGS_$(d) += -DSENSORS_$(SENSORS)

# CXXFLAGS_$(d) are extra flags passed to the C++ compiler. We'll need
# our include directory, and we'll also add an extra definition as a
# demo (look in getter.cpp for how it's used).
CXXFLAGS_$(d) := -I$(EXAMPLE_INCLUDE_DIR)

# ASFLAGS_$(d) are extra flags passed to the assembler. We don't
# have any assembly language files in this example, so we'll just
# leave it empty.
ASFLAGS_$(d) :=

### Local rules

# You can add any additional rules you want here. We don't have
# any extra rules to add.

$(USER_MODULES)/lookup.c: lookup.py
	python $< > $@

### Source files

# cSRCS_$(d) are the C source files we want compiled.
cSRCS_$(d) := cmc/cmc.c
cSRCS_$(d) += wiz/wiz.c
cSRCS_$(d) += nosc/nosc.c
cSRCS_$(d) += tuio2/tuio2.c
cSRCS_$(d) += dump/dump.c
cSRCS_$(d) += config/config.c
cSRCS_$(d) += sntp/sntp.c
cSRCS_$(d) += tube/tube.c
cSRCS_$(d) += eeprom/eeprom.c
cSRCS_$(d) += chimutil/chimutil.c
cSRCS_$(d) += chimaera/chimaera.c
cSRCS_$(d) += mdns/mdns.c
cSRCS_$(d) += dns_sd/dns_sd.c
cSRCS_$(d) += ipv4ll/ipv4ll.c
cSRCS_$(d) += dhcpc/dhcpc.c
cSRCS_$(d) += rtpmidi/rtpmidi.c
cSRCS_$(d) += oscmidi/oscmidi.c
cSRCS_$(d) += midi/midi.c
cSRCS_$(d) += dummy/dummy.c
cSRCS_$(d) += scsynth/scsynth.c
cSRCS_$(d) += arp/arp.c
cSRCS_$(d) += calibration/calibration.c
cSRCS_$(d) += lookup.c
cSRCS_$(d) += firmware.c

# cppSRCS_$(d) are the C++ sources we want compiled.  We have our own
# main.cpp, and one additional file.
#
# We can't call our main file main.cpp, or libmaple's build system
# will get confused and try to build it without our CXXFLAGS. So call
# it something else. Annoying! Hopefully LeafLabs will fix it soon.
cppSRCS_$(d) := chimaerish.cpp

# sSRCS_$(d) are the assembly sources. We don't have any.
sSRCS_$(d) :=

###############################################################################

# Include the libmaple rules.mk footer. Leave this line alone.
include $(MAKEDIR)/footer.mk
