#Geiger counter Makefile

MCU			 = atmega8u2
ARCH		 = AVR8
BOARD 		 = NONE
F_CPU 		 = 8000000
F_USB		 = $(F_CPU)
OPTIMIZATION = s
TARGET		 = reflow
SRC			 = $(TARGET).c Descriptors.c $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
LUFA_PATH 	 = ./LUFA
CC_FLAGS	 = -DUSE_LUFA_CONFIG_HEADER -IConfig/
LD_FLAGS	 =
SERIAL		 =	/dev/ttyACM0

all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk

upload: all
	avrdude -p atmega8u2 -b 19200 -P $(SERIAL) -c avrisp -v -e -U flash:w:$(TARGET).hex -U eeprom:w:$(TARGET).eep

