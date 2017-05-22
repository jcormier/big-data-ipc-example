#
#  ======== readme.txt ========
#

BigdataIPC examples

These examples are created to show case how to share big data buffers
between cores in a multicore system

host_bios: Examples here assume host running TI RTOS

simple_buffer_example: This example demonstrates simple buffer exchange
	between cores


Build procedure
==============
The following commandline can be used to build the example
with the appropriate paths set from the top directory.

PLATFORM=<platform_name> \
XDC_INSTALL_DIR="<xdc_install_dir>" \
BIOS_INSTALL_DIR="<bios_install_dir>" \
IPC_INSTALL_DIR="<ipc_install_dir>" \
PDK_INSTALL_DIR="<pdk_install_dir>" \
env gnu.targets.arm.A15F="<A15_tools_dir>" \
env ti.targets.elf.C66="<c66_tools_dir>"\
BOARD_NAME=<board_name> \
make host_bios

or
The corresponding example products.mak can be updated with the paths
for the various components and the individual examples can be built
separately.
