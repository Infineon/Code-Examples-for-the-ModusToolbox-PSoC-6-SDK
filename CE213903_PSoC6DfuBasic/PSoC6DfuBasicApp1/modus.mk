################################################################################
# \file modus.mk
# \version 1.0
#
# \brief
# This file has two purposes.
# - The first purpose is to provide information to the makefile build system.
# - The second purpose is to provide information to any IDE about the example.
#   Therefore, this file should not contain anything other than make variables.
#
################################################################################
# \copyright
# Copyright 2018, Cypress Semiconductor Corporation.  All rights reserved.
# You may use this file only in accordance with the license, terms, conditions,
# disclaimers, and limitations in the end user license agreement accompanying
# the software package with which this file was provided.
################################################################################

#
# Do not include the default CM0+ main()
#
CY_MAINAPP_CM0P_OVERRIDE_MAIN = 1

#
# Toolchain, its optimization level and the configuration (Debug/Release) type
#
TOOLCHAIN=GCC
OPTIMIZATION = Og
CONFIG = Debug

# Define custom linker script location (<ABSOLUTE PATH>/customScript.ld)
CY_MAINAPP_CM0P_LINKER_SCRIPT=$(CY_LOCAL_INCLUDE_CM0P)/dfu_cm0p.ld
CY_MAINAPP_CM4_LINKER_SCRIPT=$(CY_LOCAL_INCLUDE_CM4)/dfu_cm4.ld

#
# Vector Floating-point flag (soft/hard) selection
# 
VFP_FLAG = soft

#
# The target platform for the example
#
PLATFORMS_VERSION = 1.0
PLATFORM=PSOC6_DUAL_CORE

#
# The default name of this example
#
CY_EXAMPLE_NAME = PSoC6DfuApp1

#
# Description of the example project to display
#
CY_EXAMPLE_DESCRIPTION = These examples demonstrate basic device firmware update (DFU), also known as bootloading, with PSoC 6 MCU. This includes downloading an application from a host and installing it in device flash, and then transferring control to that application.

#
# New project dialog inclusion
#
CY_SHOW_NEW_PROJECT = true

#
# Valid platforms for this example
#
CY_VALID_PLATFORMS = PSOC6_DUAL_CORE PSOC6_SINGLE_CORE

#
# This is the required SDK for this example
#
CY_REQUIRED_SDK = Cypress SDK[1.0]

#
# Valid devices for this example. If empty, this example works for all devices
#
CY_VALID_DEVICES = CY8C6347BZI-BLD53

#
# The source code for the CM0+ application
#
CY_APP_CM0P_SOURCE = \
    Source_cm0p/main.c \
    dfu_cm0p.ld

#
# The source code for the CM4 application
#
CY_APP_CM4_SOURCE = \
	Source/main.c \
    dfu_cm4.ld \
    dfu_postbuild.bash \
    readme.txt

#
# Paths to use for ModusToolbox IDE 
#
CY_LOCAL_INCLUDE_CM0P = $(CY_GENERATED_DIR)/$(CYMAINAPP_CM0P_NAME)
CY_LOCAL_INCLUDE_CM4 = $(CY_GENERATED_DIR)/$(CYMAINAPP_CM4_NAME)

#
# Includes specific to the CM0+ application
#
APP_MAINAPP_CM0P_INCLUDES = \
	-IGeneratedSource\
    -I$(CY_LOCAL_INCLUDE_CM0P)/Source_cm0p

#
# Includes specific to the CM4 application
#
APP_MAINAPP_CM4_INCLUDES = \
	-IGeneratedSource\
    -I$(CY_LOCAL_INCLUDE_CM4)/Source

#
# Compiler flags specific to the CM0+ application
#
APP_MAINAPP_CM0P_FLAGS =

#
# Compiler flags specific to the CM4 application
#
APP_MAINAPP_CM4_FLAGS =

#
# Defines specific to the CM0+ application
#
APP_MAINAPP_CM0P_DEFINES = \
    -DAPP_NAME='"$(CY_EXAMPLE_NAME)_cm0p"'

#
# Defines specific to the CM4 application
#
APP_MAINAPP_CM4_DEFINES = \
    -DAPP_NAME='"$(CY_EXAMPLE_NAME)_cm4"'

#
# Software components needed by CM0+
#
CY_MAINAPP_CM0P_SWCOMP_USED= \
    psoc6sw-1.0/components/psoc6mw/dfu/make/param \
    psoc6sw-1.0/components/psoc6mw/dfu

CY_MAINAPP_CM0P_SWCOMP_EXT= \
    psoc6sw-1.0/components/psoc6mw/dfu/linker_scripts \

#
# Software components needed by CM4
#
CY_MAINAPP_SWCOMP_USED= \
    psoc6sw-1.0/components/psoc6mw/dfu/make/param \
    psoc6sw-1.0/components/psoc6mw/dfu

CY_MAINAPP_SWCOMP_EXT= \
    psoc6sw-1.0/components/psoc6mw/dfu/linker_scripts

#
# Other libraries (.a) needed by the CM0+ application
#
APP_MAINAPP_CM0P_LIBS = \

#
# Other libraries (.a) needed by the CM4 application
#
APP_MAINAPP_CM4_LIBS = \

#
# The path to the design.modus file
#
CYCONFIG_DESIGN_MODUS = design.modus

#
# Additional (non-core) set of generated source files
#
CYCONFIG_GENERATED_SOURCES = \

#
# Check that the CYSDK environment variable exists
#
ifndef CYSDK
$(error The SDK must be defined via the CYSDK environment variable)
endif

#
# Include the main makefile for building this type of example
#
include $(CYSDK)/libraries/platforms-$(PLATFORMS_VERSION)/common/find_platform.mk
