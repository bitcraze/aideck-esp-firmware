# AI-Deck ESP32 firmware [![CI](https://github.com/bitcraze/aideck-esp-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/aideck-esp-firmware/actions?query=workflow%3ACI)

This firmware is running on the ESP32 Nina W102 module of the AI-Deck. It
implements communication with the GAP8 chip, the Crazyflie's STM32 main MCU as
well as implementing Wifi communication.

**NOTE**: The default configuration uses the NINA_SYSBOOT (IO1) as log UART TX from the ESP32.

## Compile and flash

The firmware is currently designed to be flashed and run from the serial port.

It has been developed using the version 4.3.1 of the [Esp32 IDF].

To flash and run:
```
$ source ~/path/to/idf/export.sh
$ idf.py build
$ idf.py flash
$ idf.py monitor
```

[Esp32 IDF]: https://github.com/espressif/esp-idf.git

### Flash using the builder image

To build the firmware in Docker, use:
```
docker run --rm -it -v $PWD:/module/ --device /dev/ttyUSB0 --privileged -P bitcraze/builder /bin/bash -c "source /new_home/.espressif/python_env/idf4.3_py3.10_env/bin/activate && make"
```

Then, to flash the ESP using a JTAG, use:

```
docker run --rm -it -v $PWD:/module/ --device /dev/ttyUSB0 --privileged -P bitcraze/builder /bin/bash -c "openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f board/esp-wroom-32.cfg -c 'adapter_khz 20000' -c 'program_esp build/bootloader/bootloader.bin 0x1000 verify' -c 'program_esp build/aideck_esp.bin 0x10000 verify reset exit'"
```

### Flash using the cfloader

A binary can be flashed to the ESP via a Crazyradio using the [cfloader](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/functional-areas/cfloader/)

Note: replace the radio address with the appropriate value for your Crazyflie

```
cfloader flash build/aideck_esp.bin deck-bcAI:esp-fw -w radio://0/30/2M
```

### Build with the [toolbelt](https://github.com/bitcraze/toolbelt)

When using the [toolbelt](https://github.com/bitcraze/toolbelt) the required toolchain is running in a docker container
and no tools have to be installed on the machine.
Just pre-pend make with tb, for instance

`tb make`

`tb make clean`


### Debugging

Conntect an Olimex Programmer (Arm-usb-Tiny-h) to the esp debug port on the deck. Make sure its powered.

To start an openocd server 
```
openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f board/esp-wroom-32.cfg -c 'adapter_khz 20000'
```

Then add a debug config in your launch.json (or create one) 

    {
      "name": "Attach to ESP32",
      "type": "cppdbg",
      "request": "launch",
      "program": "${workspaceFolder}/build/aideck_esp.elf",
      "cwd": "${workspaceFolder}",
      "miDebuggerPath": "your_espressif_path/.espressif/tools/xtensa-esp-elf-gdb/11.2_20220823/xtensa-esp-elf-gdb/bin/xtensa-esp32-elf-gdb",
      "miDebuggerServerAddress": "localhost:3333",
      "externalConsole": false,
      "stopAtEntry": false
  }

