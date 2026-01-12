#!/bin/bash
echo '[General]' > neshto.creator
find . -name "*.c" > neshto.files
find . -name "*.h" >> neshto.files
echo "Makefile" >> neshto.files
echo "./Core/Inc/" > neshto.includes
echo "./Drivers/STM32F0xx_HAL_Driver/Inc/" >> neshto.includes
echo "./Drivers/CMSIS/Device/ST/STM32F0xx/Include/" >> neshto.includes
echo "./Drivers/STM32F1xx_HAL_Driver/Inc/" >> neshto.includes
echo "./Drivers/STM32F1xx_HAL_Driver/Inc/Legacy/" >> neshto.includes
echo "./Core/Inc" >> neshto.includes 
echo "./Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" >> neshto.includes
echo "./Drivers/STM32F1xx_HAL_Driver/Inc" >> neshto.includes
echo "./Drivers/CMSIS/Device/ST/STM32F1xx/Include" >> neshto.includes
echo "./Drivers/CMSIS/Include" >> neshto.includes
echo "./USB_DEVICE/App" >> neshto.includes
echo "./USB_DEVICE/Target" >> neshto.includes
echo "./Middlewares/ST/STM32_USB_Device_Library/Core/Inc" >> neshto.includes
echo "./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" >> neshto.includes
echo "./src" >> neshto.includes

