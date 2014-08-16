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
BUILDDIRS += $(BUILD_PATH)/$(d)/wiz
BUILDDIRS += $(BUILD_PATH)/$(d)/osc
BUILDDIRS += $(BUILD_PATH)/$(d)/oscquery
BUILDDIRS += $(BUILD_PATH)/$(d)/config
BUILDDIRS += $(BUILD_PATH)/$(d)/sntp
BUILDDIRS += $(BUILD_PATH)/$(d)/ptp
BUILDDIRS += $(BUILD_PATH)/$(d)/tube
BUILDDIRS += $(BUILD_PATH)/$(d)/eeprom
BUILDDIRS += $(BUILD_PATH)/$(d)/utility
BUILDDIRS += $(BUILD_PATH)/$(d)/debug
BUILDDIRS += $(BUILD_PATH)/$(d)/oscpod
BUILDDIRS += $(BUILD_PATH)/$(d)/ipv4ll
BUILDDIRS += $(BUILD_PATH)/$(d)/mdns-sd
BUILDDIRS += $(BUILD_PATH)/$(d)/dhcpc
BUILDDIRS += $(BUILD_PATH)/$(d)/arp

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

# set WIZnet chip version based on board revision: 5200, 5500
ifeq ($(REVISION), 3)
export WIZ_CHIP := 5200
endif
ifeq ($(REVISION), 4)
export WIZ_CHIP := 5500
endif

# custom preprocessor flags
#CFLAGS_$(d) += -DBENCHMARK
CFLAGS_$(d) += -DWIZ_CHIP=$(WIZ_CHIP)
CFLAGS_$(d) += -DREVISION=$(REVISION)
CFLAGS_$(d) += -DVERSION_MAJOR=$(VERSION_MAJOR)
CFLAGS_$(d) += -DVERSION_MINOR=$(VERSION_MINOR)
CFLAGS_$(d) += -DVERSION_PATCH=$(VERSION_PATCH)

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

### Source files

# cSRCS_$(d) are the C source files we want compiled.
cSRCS_$(d) := wiz/wiz.c
ifeq ($(WIZ_CHIP), 5200)
cSRCS_$(d) += wiz/W5200.c
endif
ifeq ($(WIZ_CHIP), 5500)
cSRCS_$(d) += wiz/W5500.c
endif
cSRCS_$(d) += osc/osc.c
cSRCS_$(d) += oscquery/oscquery.c
cSRCS_$(d) += config/config.c
cSRCS_$(d) += sntp/sntp.c
cSRCS_$(d) += ptp/ptp.c
cSRCS_$(d) += tube/tube.c
cSRCS_$(d) += eeprom/eeprom.c
cSRCS_$(d) += utility/utility.c
cSRCS_$(d) += debug/debug.c
cSRCS_$(d) += oscpod/oscpod.c
cSRCS_$(d) += mdns-sd/mdns-sd.c
cSRCS_$(d) += ipv4ll/ipv4ll.c
cSRCS_$(d) += dhcpc/dhcpc.c
cSRCS_$(d) += arp/arp.c
cSRCS_$(d) += firmware.c

# cppSRCS_$(d) are the C++ sources we want compiled.  We have our own
# main.cpp, and one additional file.
#
# We can't call our main file main.cpp, or libmaple's build system
# will get confused and try to build it without our CXXFLAGS. So call
# it something else. Annoying! Hopefully LeafLabs will fix it soon.
cppSRCS_$(d) := firmwareish.cpp

# sSRCS_$(d) are the assembly sources. We don't have any.
sSRCS_$(d) :=

###############################################################################

# Include the libmaple rules.mk footer. Leave this line alone.
include $(MAKEDIR)/footer.mk
