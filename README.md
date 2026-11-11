# crazyflie_fpga_firmware

This repository contains an out-of-tree Crazyflie firmware application that interfaces with an external FPGA over SPI to run the flight controller off-board from the STM32 control loop.

The project targets the Bitcraze Crazyflie platform and provides the firmware-side integration needed to exchange state data with the FPGA, receive motor commands asynchronously, and deploy the controller on the quadrotor.

## Build and Flash

The application builds against the upstream Bitcraze firmware tree, which must be present at `crazyflie-firmware/` (it is a git submodule in the public release and is not included in this anonymized snapshot):

```bash
git clone https://github.com/bitcraze/crazyflie-firmware.git crazyflie-firmware
git -C crazyflie-firmware checkout daf6820b1dfe1aa92c9758de9056c54648283f64
git -C crazyflie-firmware submodule update --init --recursive
make          # build
make cload    # flash over the radio bootloader
```

Refer to the official Crazyflie documentation for the standard out-of-tree firmware build and flashing workflow used by this project:

- https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/

## Repository Contents

- `src/`: firmware sources for the FPGA controller, SPI communication layer, and FPGA deck support
- `app-config`: out-of-tree application configuration used during Crazyflie firmware builds
- `crazyflie-firmware/`: location of the Crazyflie firmware tree used as the build base (see above)
- `Kbuild`: top-level build integration for the out-of-tree application
- `Makefile`: entry point for building and flashing the firmware

## Related Components

- FPGA controller: `ADMM_FPGA`
- FPGA deck hardware: `Crazyflie_FPGA_Deck`
- Base Crazyflie firmware: https://github.com/bitcraze/crazyflie-firmware
