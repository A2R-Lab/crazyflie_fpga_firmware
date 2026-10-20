# FPGA Controller Application for Crazyflie
#
# This application implements an out-of-tree controller that communicates
# with an external FPGA over SPI to compute control commands.
#
# Build with: make
# Flash with: make cload
#
# The firmware uses the Kbuild build system. There are 'Kbuild' files
# that outline what needs to be built. (check src/Kbuild).

CRAZYFLIE_BASE := $(PWD)/crazyflie-firmware

# Use app-config for build configuration
OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
