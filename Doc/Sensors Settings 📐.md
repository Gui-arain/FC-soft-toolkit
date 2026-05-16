# Sensor Settings

## IMU - ICM-40609-D

SPI5

Datasize: 8bits

Frame format: Motorola

First bit: MSB first

Mode 0:
```c
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
```


## MAG - MMC5983MA

SPI4

Datasize: 8bits

Frame format: Motorola

First bit: MSB first

Mode 3:
```c
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
```
