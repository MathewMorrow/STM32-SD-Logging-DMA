# STM32-SD-Logging-DMA
Non-blocking FATFS microSD datalogging at 2.5MBps using an STM32 SDIO in DMA mode

This board was designed as a test bench for ultra fast non-blocking blackbox data recording for my own fully self built quadcopter FPV flight hardware and firmware.

The goal was to validate the SDIO+DMA peripherals and custom FATFS firmware for non-blocking writes of +256 BYTES every 313 micro-seconds (3.2kHz PID loops)

Using the built in "blackboxSDBenchmark()" function I was able to achieve up to 2.5MBps writes speeds using a cheap microSDHC class-6 card.

# Hardware Design
![SDIO_Dev_Board_ISOMET](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/8266ebc9-88af-463e-90df-691020a96654)

![Screenshot 2023-08-27 224710](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/fb199d7d-89ec-41c2-9682-daa8ceea8deb)

![Screenshot 2023-08-27 224647](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/517ffbcf-851b-4d67-b90e-8ff8f3fcacf9)

![SDIO_Dev_Board_PCB](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/ca3d4cf8-f235-4ad0-9c22-dc99864ec901)

![IMG_0043](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/4522d70e-8b15-4562-a890-a9d9e68322ce)

# End Application in use
Snapshot of raw binary data decoded from my flight controller. I further wrapped this blackbox logging code with raw binary data writes to the SD card and designed a MATLAB script that decodes the raw data. This significantly decreases execution cycles and memory size over writing ASCII-CSV data to the card but adds complexity of post-decoding instead of simply opening the log file in excell. It ended up being easier than I thought and only took an evening of work to update the blackbox wrapper code and MATLAB decode script.  
![matlab_decode_example](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/df21cc90-e7eb-4ac5-b20c-87a9f2295d46)

# FFT Application
The most important application for this data logging was FFT analysis to see where to place my LPF and Notch filters on my quad.  
Here is a plot of the raw gyro data coming out of the chip and my filtered data going into the PIDs.  
**Anything above ~100Hz needs filtering out and below 80Hz is what the drone needds to control**
![FFT_Example](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/391683b6-83b7-4e12-879b-0fdffa7c5188)

