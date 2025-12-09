Black Pill F4 pinout blackmagic probe
https://github.com/blackmagic-debug/blackmagic/tree/main/src/platforms/common/blackpill-f4

PB8 - SWDCLK - white
PB9 - SWDDIO - gray
GND - purple



Pylnata opitna postanovka - logic analizer + blackmagic probe + stm32f103 blue pill

[Mon Dec  8 10:38:55 2025] usb 3-3: new high-speed USB device number 18 using xhci_hcd
[Mon Dec  8 10:38:56 2025] usb 3-3: New USB device found, idVendor=0403, idProduct=6010, bcdDevice= 7.00
[Mon Dec  8 10:38:56 2025] usb 3-3: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[Mon Dec  8 10:38:56 2025] usb 3-3: Product: USB <-> Serial Converter
[Mon Dec  8 10:38:56 2025] usb 3-3: Manufacturer: FTDI
[Mon Dec  8 10:38:56 2025] usb 3-3: SerialNumber: FT2232HL
[Mon Dec  8 10:38:56 2025] ftdi_sio 3-3:1.0: FTDI USB Serial Device converter detected
[Mon Dec  8 10:38:56 2025] usb 3-3: Detected FT2232H
[Mon Dec  8 10:38:56 2025] usb 3-3: FTDI USB Serial Device converter now attached to ttyUSB0
[Mon Dec  8 10:38:56 2025] ftdi_sio 3-3:1.1: FTDI USB Serial Device converter detected
[Mon Dec  8 10:38:56 2025] usb 3-3: Detected FT2232H
[Mon Dec  8 10:38:56 2025] usb 3-3: FTDI USB Serial Device converter now attached to ttyUSB1
[Mon Dec  8 10:40:39 2025] usb 3-5: new full-speed USB device number 19 using xhci_hcd
[Mon Dec  8 10:40:39 2025] usb 3-5: New USB device found, idVendor=0483, idProduct=5740, bcdDevice= 2.00
[Mon Dec  8 10:40:39 2025] usb 3-5: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[Mon Dec  8 10:40:39 2025] usb 3-5: Product: STM32 Virtual ComPort
[Mon Dec  8 10:40:39 2025] usb 3-5: Manufacturer: STMicroelectronics
[Mon Dec  8 10:40:39 2025] usb 3-5: SerialNumber: 6D8131955348
[Mon Dec  8 10:40:39 2025] cdc_acm 3-5:1.0: ttyACM0: USB ACM device
[Mon Dec  8 10:40:48 2025] usb 3-2: new full-speed USB device number 20 using xhci_hcd
[Mon Dec  8 10:40:48 2025] usb 3-2: New USB device found, idVendor=1d50, idProduct=6018, bcdDevice= 2.00
[Mon Dec  8 10:40:48 2025] usb 3-2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[Mon Dec  8 10:40:48 2025] usb 3-2: Product: Black Magic Probe (BlackPill-F411CE) v2.0.0-rc1-38-gf4e79b65
[Mon Dec  8 10:40:48 2025] usb 3-2: Manufacturer: Black Magic Debug
[Mon Dec  8 10:40:48 2025] usb 3-2: SerialNumber: 3368334B3134
[Mon Dec  8 10:40:48 2025] cdc_acm 3-2:1.0: ttyACM1: USB ACM device
[Mon Dec  8 10:40:48 2025] cdc_acm 3-2:1.2: ttyACM2: USB ACM device


104849 smooker@sw1 ~/src/stm32/scales_stepper_malinovski $ cat ./USB_DEVICE/App/usbd_desc.c | grep -e USBD_PID -e USBD_VID
#define USBD_VID     1155
#define USBD_PID_FS     22336

104935 smooker@sw1 ~/src/stm32/scales_stepper_malinovski $ hex2dec.pl 1155
HEX:0483
DEC:1155
BIN:     10010000011
104941 smooker@sw1 ~/src/stm32/scales_stepper_malinovski $ hex2dec.pl 22336
HEX:5740
DEC:22336
BIN: 101011101000000
104947 smooker@sw1 ~/src/stm32/scales_stepper_malinovski $




