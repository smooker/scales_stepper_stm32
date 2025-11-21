#!/bin/bash
#gdb-multiarch -x ./script2.gdb ./build/modbus.elf
arm-none-eabi-gdb -x ./script2.gdb ./build/scales_stepper_malinovski.elf
