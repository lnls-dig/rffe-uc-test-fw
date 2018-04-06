# RFFEuC Test Firmware

Firmware for testing RFFEuC boards, mounted on the [RFFE-uC Test Board](https://github.com/lnls-dig/rffe-uc-hw-test-board) using a Cortex M3 LPC1768 processor.

## Pre-requisites

The following packages must be installed on your system in order to compile the firmware:
- **gcc-arm-none-eabi**

**gcc-arm-none-eabi** can be installed from the pre-compiled files found at: https://launchpad.net/gcc-arm-embedded/+download
or you can run the following command under Ubuntu:

	sudo apt-get install gcc-arm-none-eabi

Next step is to clone this repository into your workspace.

	git clone https://github.com/lnls-dig/rffe-uc-test-fw

## Compilation

Go to the repository folder

	cd /path/to/repo/

Run `make` (you can add the `-j4` flag to speed up the proccess) :

    make -j4

A few flags can be set in order to match your hardware setup, which are:

    ETH_INTERFACE=<FIX_IP|DHCP>

If not set, the Makefile will output a warning and use a default value for each.

Example using fixed IP addressing:

	make -j4 ETH_INTERFACE=FIX_IP IP=10.2.119.203 GATEWAY=10.2.119.1

*NOTE: The compiler will print a few warnings, most of them are regarding the mbed libraries, but since they have a stable version on github, we'll just ignore those warnings.*

Both a `.elf` file and a `.bin` file will be generated in the source folder. You can use any one you prefer to program your processor.

To clean the compilation files (binaries, objects and dependence files), just run

	make clean

## Programming

You'll need a programming board to the LPC1768 uC in order to flash the firmware to the controller.

To program using the LPCLink v1 or v2 you'll need to install the latest [LPCXpresso](https://www.nxp.com/products/processors-and-microcontrollers/arm-based-processors-and-mcus/lpc-cortex-m-mcus/lpc1100-cortex-m0-plus-m0/lpcxpresso-ide-v8.2.2:LPCXPRESSO) software (we need the programmer firmware that they provide) and dfu-util.

    sudo apt-get install dfu-util

### LPCLink v1

After installing LPCXpresso, follow these steps:

    dfu-util -d 0x0471:0xDF55 -c 0 -t 2048 -R -D <LPCXpresso_install_path>/bin/LPCXpressoWIN.enc
    <LPCXpresso_install_path>/bin/crt_emu_cm3_nxp -pLPC1768 -g -wire=winusb -load-base=0 -flash-load-exec=BUILD/rffe-uc-test-fw

Dfu-util will only run successfully the first time it's run. If at first the NXP program software doesn't run, try to run it again.

### LPCLink v2

After installing LPCXpresso, follow these steps:

    dfu-util -d 0x1FC9:0x000C -c 0 -t 2048 -R -D <LPCXpresso_install_path>/bin/LPC432x_CMSIS_DAP_V5_173.bin.hdr
    <LPCXpresso_install_path>/bin/crt_emu_cm_redlink -pLPC1768 -g -load-base=0 -flash-load-exec=BUILD/rffe-uc-test-fw

Dfu-util will only run successfully the first time it's run. If at first the NXP program software doesn't run, try to run it again.

## Testing

The firmware will perform 5 tests in sequence upon initialization: LEDs brightness, GPIO loopback, Power Supply, FeRAM and Ethernet tests.

In the LEDs test, a single 20mm LDR is directed at all 4 LEDs and connected to the Header on the test board. Each LED will light up once and the firmware will check if all of them are lighting up properly.

In the GPIO loopback, all the GPIO signals that are connected in loopback pairs in the test board will toggle its logic levels and assert its connection.

Power Supply test will check the voltage level on the 5V and 3.3V lines on the board.

FeRAM will write a random pattern in all pages and read them back to check if the memory access is successfull.

The Ethernet test will configure the PLL to generate the 50MHz to the PHY chip and setup a TCP server. It'll wait for a client to connect and send the following string `Test msg!\0` (`\0` here is the terminator character on the string).

A python script was developed to continuously try to connect to the TCP server and send the test string. It's placed on the `scripts` folder and its usage is as follows:

    python eth_server_test.py

If all tests are successfull, the board will flash its 4 LEDs at a 2Hz rate. All tests information are available through the USB-SERIAL port (8N1 115200bps).
