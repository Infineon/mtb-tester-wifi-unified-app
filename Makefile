################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Top-level application make file.
#
################################################################################
# \copyright
# Copyright 2020-2023 Cypress Semiconductor Corporation (an Infineon company)
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################


################################################################################
# Basic Configuration
################################################################################

#Type of MTB Makefile Options include:
#
#COMBINED    -- Top Level Makefile usually for single standalone application
#APPLICATION -- Top Level Makefile usually for multi project application
#PROJECT     -- Project Makefile under Application
#
MTB_TYPE=COMBINED

# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager
# ('make library-manager' from command line), which will also update Eclipse IDE launch
# configurations.
TARGET=APP_CY8CEVAL-062S2-LAI-4373M2

# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=wifi-unified-tester

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC 11.3.1, provided with ModusToolbox IDE
# ARM     -- ARM Compiler (must be installed separately)
# IAR     -- IAR Compiler (must be installed separately)
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

# Default build configuration. Options include:
#
# Debug   -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
CONFIG=Release

# If set to "true" or "1", display full command-lines when building.
VERBOSE=
ifeq ($(TARGET), APP_HATCHET1-CP-FLASH)
# Standard Library Config
CY_CORE_EXTRA_LD_FLAGS+=-L$(CY_TOOLS_DIR)/gcc/lib/gcc/arm-none-eabi/11.3.1/thumb/v8-m.main+fp/hard
CY_RECIPE_EXTRA_LIBS+=-lgcc -lc -lnosys
DEFINES+=CY_STORAGE_WIFI_DATA=\".cy_xip\"
HEAP_SIZE=0x11800
DEFINES += TX_PACKET_POOL_SIZE=10
DEFINES += RX_PACKET_POOL_SIZE=20
DEFINES += WCM_WORKER_THREAD_STACK_SIZE=5120
DEFINES += SECURE_SOCKETS_THREAD_STACKSIZE=1024
DEFINES += DEFAULT_TCP_WINDOW_SIZE=12288
LIFE_CYCLE_STATE=CM
else ifeq ($(TARGET), APP_HATCHET1-CP-XIP)
CY_CORE_EXTRA_LD_FLAGS+=-L$(CY_TOOLS_DIR)/gcc/lib/gcc/arm-none-eabi/11.3.1/thumb/v8-m.main+fp/hard
CY_RECIPE_EXTRA_LIBS+=-lgcc -lc -lnosys
HEAP_SIZE=0x20000
XIP=1
LIFE_CYCLE_STATE=CM
endif

#SN PRINTF is enabled
DEFINES += HAVE_SNPRINTF

################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
ifeq ($(TARGET), APP_HATCHET1-CP-FLASH)
COMPONENTS+=THREADX NETXDUO 55900 CYW955913WLIPA WCM WIFI_INTERFACE_OCI
else ifeq ($(TARGET), APP_HATCHET1-CP-XIP)
COMPONENTS+=THREADX NETXDUO NETXSECURE 55900 CYW955513WLIPA PSOC6HAL SECURE_SOCKETS WCM WIFI_INTERFACE_OCI
else
COMPONENTS=FREERTOS PSOC6HAL LWIP MBEDTLS WCM SECURE_SOCKETS
endif

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=./configs

MBEDTLSFLAGS = MBEDTLS_USER_CONFIG_FILE='"configs/mbedtls_user_config.h"'

ifeq ($(TARGET), APP_HATCHET1-CP-FLASH)
# As Current LPA supports lwip only, so disabling LPA source for H1-CP
CY_IGNORE+=./lpa
else ifeq ($(TARGET), APP_HATCHET1-CP-XIP)
CY_IGNORE+=./lpa
endif

# Add additional defines to the build process (without a leading -D).
ifeq ($(TARGET), APP_HATCHET1-CP-FLASH)
DEFINES+=OS_THREADX CYBSP_WIFI_CAPABLE HAVE_SNPRINTF CY_RTOS_AWARE H1CP_SUPPORT CY_RETARGET_IO_CONVERT_LF_TO_CRLF
else ifeq ($(TARGET), APP_HATCHET1-CP-XIP)
DEFINES+=OS_THREADX CYBSP_WIFI_CAPABLE ENABLE_UNITTESTS HAVE_SNPRINTF CY_RTOS_AWARE H1CP_SUPPORT
else
DEFINES+=$(MBEDTLSFLAGS) CYBSP_WIFI_CAPABLE CY_RTOS_AWARE CY_RETARGET_IO_CONVERT_LF_TO_CRLF TARGET_NAME=$(TARGET)
endif

# The definitions to enable Wi-Fi Low Power Assistant (LPA)
# The default configuraion is disabled
#DEFINES+=LPA_ENABLE

# Build timestamp
BUILD_TIME := "Timestamp_$(shell date +%Y-%m-%dT%H:%M:%S%z)"
DEFINES+=BUILD_TIME_STAMP=$(BUILD_TIME)

# WiFi Unified Application Version
file := version.xml
WIFI_UNIFIED_VERSION_STRING :=  $(shell sed -n '1 p' ${file} | sed -e 's/<version>//g' -e 's/<\/version>//')
DEFINES+=WIFI_UNIFIED_VER=$(WIFI_UNIFIED_VERSION_STRING)
# WiFi Bluetooth Tester Version
file := sub_version.xml
WIFI_BT_VERSION_STRING :=  $(shell sed -n '1 p' ${file} | sed -e 's/<con_app_version>//g' -e 's/<\/con_app_version>//')
DEFINES+=WIFI_BT_VER=$(WIFI_BT_VERSION_STRING)
# WiFi MFG Tester Version
WIFI_MFG_VERSION_STRING :=  $(shell sed -n '2 p' ${file} | sed -e 's/<mfg_app_version>//g' -e 's/<\/mfg_app_version>//')
DEFINES+=WIFI_MFG_VER=$(WIFI_MFG_VERSION_STRING)
# WiFi MFG Middleware Version
WIFI_MFG_MW_VERSION_STRING :=  $(shell sed -n '3 p' ${file} | sed -e 's/<mfg_mw_version>//g' -e 's/<\/mfg_mw_version>//')
DEFINES+=WIFI_MFG_MW_VER=$(WIFI_MFG_MW_VERSION_STRING)
# WiFi Cert Tester Version
WIFI_CERT_VERSION_STRING :=  $(shell sed -n '4 p' ${file} | sed -e 's/<cert_app_version>//g' -e 's/<\/cert_app_version>//')
DEFINES+=WIFI_CERT_VER=$(WIFI_CERT_VERSION_STRING)
# WiFi LPA(Low Power Asset) Middleware Version
WIFI_LPA_VERSION_STRING :=  $(shell sed -n '5 p' ${file} | sed -e 's/<lpa_mw_version>//g' -e 's/<\/lpa_mw_version>//')
DEFINES+=WIFI_LPA_VER=$(WIFI_LPA_VERSION_STRING)

# Disable WHD logging
DEFINES+=WHD_PRINT_DISABLE

# Control CY_WIFI_HOST_WAKE_SW_FORCE
ifeq ($(HOST_WAKE),0)
DEFINES+=CY_WIFI_HOST_WAKE_SW_FORCE=0
endif

ifneq ($(CY_APP_TEST_NUM),)
COMPONENTS += CUSTOM_DESIGN_MODUS TKO_CONFIG$(CY_APP_TEST_NUM)
endif

DEFINES+=CY_SD_HOST_CLK_RAMP_UP_TIME_MS_WAKEUP=0

ifeq ($(TARGET), APP_HATCHET1-CP-FLASH)
else ifeq ($(TARGET), APP_HATCHET1-CP-XIP)
else ifeq ($(TARGET), APP_CY8CEVAL-062S2-CYW43022CUB)
else ifeq ($(TARGET), APP_CY8CEVAL-062S2-LAI-4373M2)
else ifeq ($(TARGET), APP_CY8CEVAL-062S2-MUR-4373M2)
else
DEFINES+=WIFI_6G_CAPABLE
endif

DEFINES+=MBEDTLS_MD4_C

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT=

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CFLAGS=

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
LDFLAGS=

# Additional / custom libraries to link in to the application.
LDLIBS=

# Path to the linker script to use (if empty, use the default linker script).
LINKER_SCRIPT=

# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.
POSTBUILD=

CY_LIBS_SEARCH_DEPTH=7

# Disables iperf application
# DEFINES+=DISABLE_COMMAND_CONSOLE_IPERF

# Disables WL command support
# DEFINES+=DISABLE_WL_SUPPORT

#
# This is setup to build with either the IAR compiler or the
# GCC compiler.  There is currently no support for the ARM
# compiler.
#
ifeq ($(TOOLCHAIN),IAR)
CY_IGNORE+=./libs/freertos/Source/portable/TOOLCHAIN_GCC_ARM
else
CY_IGNORE+=./libs/freertos/Source/portable/TOOLCHAIN_IAR
endif

ifeq ($(TOOLCHAIN), ARM)
LDFLAGS+=--diag_suppress=L6314W
endif

ifeq ($(TARGET), APP_HATCHET1-CP-FLASH)
CY_IGNORE+=$(SEARCH_mw-command-console)/TESTS $(SEARCH_mw-command-console)/doxygen $(SEARCH_mw-command-console)/docs
CY_IGNORE+=$(SEARCH_btstack) $(SEARCH_btstack-integration) $(SEARCH_freertos)
CY_IGNORE+=$(SEARCH_lwip-freertos-integration) $(SEARCH_lwip) $(SEARCH_lwip-network-interface-integration)
CY_IGNORE+=$(SEARCH_mbedtls) $(SEARCH_enterprise-security) $(SEARCH_wpa3-external-supplicant)
CY_IGNORE+=$(SEARCH_topic)/TESTS $(SEARCH_topic)/doxygen $(SEARCH_topic)/docs $(SEARCH_wifi-cert)
else ifeq ($(TARGET), APP_HATCHET1-CP-XIP)
CY_IGNORE+=$(SEARCH_mw-command-console)/TESTS $(SEARCH_mw-command-console)/doxygen $(SEARCH_mw-command-console)/docs
CY_IGNORE+=$(SEARCH_btstack) $(SEARCH_btstack-integration) $(SEARCH_freertos)
CY_IGNORE+=$(SEARCH_lwip-freertos-integration) $(SEARCH_lwip) $(SEARCH_lwip-network-interface-integration)
CY_IGNORE+=$(SEARCH_mbedtls) $(SEARCH_enterprise-security) $(SEARCH_wpa3-external-supplicant)
CY_IGNORE+=$(SEARCH_topic)/TESTS $(SEARCH_topic)/doxygen $(SEARCH_topic)/docs $(SEARCH_wifi-cert)
endif
################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler's "bin" directory.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# IDE provided compiler by default).
CY_COMPILER_PATH=


# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder).
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS))
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

include $(CY_TOOLS_DIR)/make/start.mk
