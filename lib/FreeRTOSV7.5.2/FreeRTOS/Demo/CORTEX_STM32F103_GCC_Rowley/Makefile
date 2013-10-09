#This is the Makefile for Lab 1 Blinky for ELEC424 Fall 2013
# Authors: Steven Arroyo and Lin Zhong, Rice University


# use the arm compiler 
CC = arm-none-eabi-gcc

# Define paths for all included files.  This assumes STM standard library was placed unmodified 
# in a folder called lib/ as outlined in the lab description.
current_dir = $(shell pwd)
Common = ../Common/include
FreeRTOS_Core = ../../Source
FreeRTOS_Core_Include = $(FreeRTOS_Core)/include
FreeRTOS_ARM_CM3 = $(FreeRTOS_Core)/portable/GCC/ARM_CM3
STM = "ST Library"/inc

# Define the compiler flags
CFLAGS = -O0 -g3 -mcpu=cortex-m3 -mthumb -I$(Common) -I$(current_dir)/Drivers -I$(current_dir) -I$(STM) -I$(FreeRTOS_Core) -I$(FreeRTOS_Core_Include) -I$(FreeRTOS_ARM_CM3) $(FreeRTOS_Core_Include)/FreeRTOS.h  -Wl,--gc-sections -T

# build all relevant files and create .elf
all:
	$(CC) $(CFLAGS) STM32F10x_Startup.s main.c $(current_dir)/ParTest/ParTest_MCBSTM32.c $(FreeRTOS_Core)/list.c $(FreeRTOS_Core)/queue.c $(FreeRTOS_Core)/tasks.c $(FreeRTOS_Core)/timers.c $(FreeRTOS_ARM_CM3)/port.c -o blinky.o
