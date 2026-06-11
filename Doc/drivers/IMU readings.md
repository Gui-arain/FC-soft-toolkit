
So for your case, the right architecture is:

1. **Board code / kernel side**
    - initialize the SPI bus
    - configure CS, DRDY/INT GPIO, EXTI interrupt
    - register an IMU driver
2. **Kernel IMU driver**
    - owns the SPI peripheral
    - configures the IMU ODR/FIFO/interrupts
    - reads samples from the sensor
    - pushes data to a buffer/device interface    
3. **User-space task**
    - opens `/dev/...`
    - reads buffered samples or waits on `poll()`
    - does filtering / estimator work

IMU data-ready interrupt + hardware FIFO + burst SPI reads + kernel ring buffer