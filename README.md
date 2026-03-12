# crazyflie_fpga_firmware

This repository contains an out-of-tree Crazyflie firmware application that interfaces with an external FPGA over SPI to run the flight controller off-board from the STM32 control loop.

The project targets the Bitcraze Crazyflie platform and provides the firmware-side integration needed to exchange state data with the FPGA, receive motor commands asynchronously, and deploy the controller on the quadrotor.

## Project Information

This work was developed within the master's thesis:

*Hardware-Algorithm Co-Design for Real-Time Linear Model Predictive Control. FPGA Implementation and Deployment on a Resource-Constrained Quadrotor*

Author: Andrea Grillo  
Period: 2025-2026

## Build and Flash

Refer to the official Crazyflie documentation for the standard out-of-tree firmware build and flashing workflow used by this project:

- https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/

## Repository Contents

- `src/`: firmware sources for the FPGA controller, SPI communication layer, and FPGA deck support
- `app-config`: out-of-tree application configuration used during Crazyflie firmware builds
- `crazyflie-firmware/`: bundled Crazyflie firmware tree used as the build base
- `Kbuild`: top-level build integration for the out-of-tree application
- `Makefile`: entry point for building and flashing the firmware

## Related Repositories

- FPGA controller repository: [ADMM_FPGA](https://github.com/A2R-Lab/ADMM_FPGA)
- FPGA deck hardware repository: [Crazyflie_FPGA_Deck](https://github.com/A2R-Lab/Crazyflie_FPGA_Deck)
- Base Crazyflie firmware repository: [crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware)
