
# building

* setup: `cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -S . -B build/Debug -G Ninja`

* build: `cmake --build build/Debug/ --clean-first -j 8`

* flash: `STM32_Programmer_CLI --connect port=swd --download build/Debug/rcauxdev_mcu.elf -hardRst`

* flash: `st-flash --debug --format ihex write build/WS2812B_STM32F103.hex`

```