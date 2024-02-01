# STM32-SD-Logging-DMA
Non-blocking FATFS microSD datalogging at 2.5MBps using an STM32 SDIO in DMA mode

This board was designed as a test bench for ultra fast blackbox data recording for my own quadcopter FPV flight controller and firmware.

The goal was to validate the SDIO+DMA peripherals and customer FATFS firmware for non-blocking writes of +256 BYTES every 313 micr-seconds (3.2kHz PID loops)

![SDIO_Dev_Board_ISOMET](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/8266ebc9-88af-463e-90df-691020a96654)

![Screenshot 2023-08-27 224710](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/fb199d7d-89ec-41c2-9682-daa8ceea8deb)

![Screenshot 2023-08-27 224647](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/517ffbcf-851b-4d67-b90e-8ff8f3fcacf9)
