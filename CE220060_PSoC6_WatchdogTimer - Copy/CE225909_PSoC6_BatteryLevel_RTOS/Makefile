#
# This makefile is just here so that typing 'make' in this directory
# works.  This makefile just includes the modus.mk.  The modus.mk name
# is special in IDE environments and is a trigger so that the IDE knows
# this is a modus compliant example that supports the IDE related targets
#

PLATFORM ?= PSOC6_DUAL_CORE
ifeq ($(PLATFORM),PSOC6_DUAL_CORE)
    DEVICE=CY8C6347BZI-BLD53
else
    DEVICE=CY8C6337BZI-BLF13
endif

include modus.mk
