# STM32-SD-Logging-DMA
Non-blocking FATFS microSD datalogging at 2.5MBps using an STM32 SDIO in DMA mode

This board was designed as a test bench for ultra fast non-blocking blackbox data recording for my own fully self built quadcopter FPV flight hardware and firmware.

The goal was to validate the SDIO+DMA peripherals and custom FATFS firmware for non-blocking writes of +256 BYTES every 313 micro-seconds (3.2kHz PID loops)

Using the built in "blackboxSDBenchmark()" function I was able to achieve up to 2.5MBps writes speeds using a cheap microSDHC class-6 card.

![SDIO_Dev_Board_ISOMET](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/8266ebc9-88af-463e-90df-691020a96654)

![Screenshot 2023-08-27 224710](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/fb199d7d-89ec-41c2-9682-daa8ceea8deb)

![Screenshot 2023-08-27 224647](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/517ffbcf-851b-4d67-b90e-8ff8f3fcacf9)

![SDIO_Dev_Board_PCB](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/ca3d4cf8-f235-4ad0-9c22-dc99864ec901)
