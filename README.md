# STM32-SD-Logging-DMA
Non-blocking FATFS microSD datalogging at 2.5MBps using an STM32 SDIO in DMA mode then

This board was designed as a test bench for ultra fast non-blocking blackbox data recording for my own fully self built quadcopter FPV flight hardware and firmware.

The goal was to validate the SDIO+DMA peripherals and custom FATFS firmware for non-blocking writes of +256 BYTES every 313 micro-seconds (3.2kHz PID loops).

Using the built in "blackboxSDBenchmark()" function I was able to achieve up to 2.5MBps writes speeds using a cheap microSDHC class-6 card.

**Link to my FPV drone hardware and Firmware project - THIS REPO IS PRIVATE AT THIS TIME**  
[**RubyFlight Project**](https://github.com/MathewMorrow/rubyflight.git/)  
This hardware+firmware project represent 500+ hours of my personal time outside of my day job as a Sr. Electrical Eng.  
Once I have more thoroughly documented and cleaned up the project I will make it public.

# How the code works  
## Background preamble
It's important to first know that all microSD cards can and will block new data read/writes for up to **200mS** whenever they need to perform internal housekeeping such as wear leveling or cluster boundary writes. If you are performing "fast" writes to a card, these blocking calls will occure every few seconds depending on the SD card. For time critical applications, such as a flight controller running 6 PID loops every 300 micro-seconds, this is a non-starter. This also means that simply adding a DMA channel to handle the SDIO peripheral writes to the card is not going to avoid these delays. In fact the standard FATFS library always performs a blocking call, even with DMA enabled, to ensure data is properly written to the card and the MCUs FATFS file-pointers etc. are updated correctly. The FATFS standard library is designed for RTOS systems creating a non-blocking non-RTOS file system is non-trivial.  
## Keys to success  
* Start with good hardware. Garbage in = garbage out. Amazon is filled with fake SD cards. Use a microSDHC of class 6 or higher.
* Reformat the card with a cluster size of 4096. This is the buffer size used in the code. More on this later.
    * I also played around with reading the cluster size from the card at startup and dynamically changing the buffer size.
* Use the SDIO hardware peripheral. While SPI is 'supported' by the microSD standard it's an afterthought for manufacturers and a fraction of the speed of SDIO.
* For maximum write speeds it is critical to write a full cluster at a time as well as on cluster boundaries. If data is partially written across a cluster boundary, the card will need to perform a cluster erase and rewrite the next time the cluster is written to.
* Ensure the file has a contiguous amount of space pre-allocated. Otherwise the card will have to halt and find a new location on the disk to write data.
## The code  
* Create a new file in the root directory. Read through existing file indexes and increment number in file name.
* Pre-allocate a contiguous amount of space in the file using the FATFS standard function f_expand.
   * Optional check if the file is contiguous - this is slow and is always true.
* Continuously buffer data into two pingpong buffers that are the size of a full cluster (4096 bytes).
* Once a buffer is full, set a blackboxDataPending flag and pingpong to the other buffer.
    * First check that the DMA is not actively writing the other buffer, otherwise drop the packet
    * If the other buffer is full and there is not enough space to buffer new data on the current buffer drop the packet.
    * These packet drops never get close to occurring with 4096 buffer sizes but good edge case handling.
* Every loop check if the blackboxDataPending is set. If so, trigger the DMA to send the cluster sized buffer.
* So far everything is easy. The hard part is modifying the standard FATFS library with a "fastWrite" flag.
    * Removing blocking calls in the FATFS library altogether will cause hard faults for other standard functions like card initialization, fopen, fclose etc.
    * I added a fastWrite flag that avoids wait states in the FATFS library only for the full cluster data writes.
    * The fastWrite flag is set just before and reset just after a DMA writeSector() is triggered.
* When the file is closed, it is likely the current buffer is only partially filled.
    * At this point fast writes are not critical and the partially filled buffer is written to the file before closing and housekeeping with the standard FATFS fclose() function.

# Hardware Design
![SDIO_Dev_Board_ISOMET](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/8266ebc9-88af-463e-90df-691020a96654)

![Screenshot 2023-08-27 224710](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/fb199d7d-89ec-41c2-9682-daa8ceea8deb)

![Screenshot 2023-08-27 224647](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/517ffbcf-851b-4d67-b90e-8ff8f3fcacf9)

![SDIO_Dev_Board_PCB](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/ca3d4cf8-f235-4ad0-9c22-dc99864ec901)

![IMG_0043](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/4522d70e-8b15-4562-a890-a9d9e68322ce)

# End Application in use
Snapshot of raw binary data decoded from my flight controller. I further wrapped this blackbox logging code with raw binary data writes to the SD card and designed a MATLAB script that decodes the raw data. This significantly decreases execution cycles and memory size over writing ASCII-CSV data to the card but adds complexity of post-decoding instead of simply opening the log file in excell. It ended up being easier than I thought and only took an evening of work to update the blackbox wrapper code and MATLAB decode script.  

[**Link To MATLAB Blackbox Decode Repo**](https://github.com/MathewMorrow/Ruby-Blackbox-Decode.git)

![matlab_decode_example](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/5cdc683e-28ab-40dc-8775-f0ccb034343a)

# FFT Application
The most important application for this data logging was FFT analysis to see where to place my LPF and Notch filters on my quad.  
Here is a plot of the raw gyro data coming out of the chip and my filtered data going into the PIDs.  
**Anything above ~100Hz needs filtering out and below 80Hz is what the drone needs to control**
![FFT_Example](https://github.com/MathewMorrow/STM32-SD-Logging-DMA/assets/50677844/391683b6-83b7-4e12-879b-0fdffa7c5188)

