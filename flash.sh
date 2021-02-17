#!/bin/sh

firmware_elf=./BUILD/rffe-uc-test-fw.elf

openocd -f lpc17-cmsis.cfg -c init -c "reset halt" -c "program ${firmware_elf} verify" -c "reset run" -c "shutdown"
