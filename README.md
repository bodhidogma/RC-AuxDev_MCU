# Overview

Simple STM32F3 interface to support the following functionality:

* Decode 2+ Radio Control Receiver PWM channels via timer CMP mode
* Output 2 WS2812 RGB led chains via DMA + Timers
* GPIO trigger MOSFET high current switch (rocket igniter)
* IO support via USB CDC (serial) interface @ 115200

# Building

* setup: `cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -S . -B build/Debug -G Ninja`

* build: `cmake --build build/Debug/ --clean-first -j 8`
* build: `cmake --build --preset Debug -j 8 --verbose`

* flash: `STM32_Programmer_CLI --connect port=swd --download build/Debug/RC-AuxDev_MCU.elf -hardRst`

* flash: `st-flash --debug --format ihex write build/WS2812B_STM32F103.hex`

# App