#
# This file is the makefile for building Ethernet PHY library.
#
ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

MODULE_NAME = enetphy

SRCDIR = .
INCDIR = . include/core include/phy

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk
INCLUDE_INTERNAL_INTERFACES =

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON += enetphy.c generic_phy.c
SRCS_COMMON += dp83867.c dp83869.c dp83822.c vsc8514.c

PACKAGE_SRCS_COMMON = $(SRCDIR)
CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(ENET_CFLAGS)

# Core/SoC/platform specific source files and CFLAGS
# Example:
#   SRCS_<core/SoC/platform-name> =
#   CFLAGS_LOCAL_<core/SoC/platform-name> =

# Include common make files
ifeq ($(MAKERULEDIR), )
  #Makerule path not defined, define this and assume relative path from ROOTDIR
  MAKERULEDIR := $(ROOTDIR)/ti/build/makerules
  export MAKERULEDIR
endif
include $(MAKERULEDIR)/common.mk

# OBJs and libraries are built by using rule defined in rules_<target>.mk
#     and need not be explicitly specified here

# Nothing beyond this point
