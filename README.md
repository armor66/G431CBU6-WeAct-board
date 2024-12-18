Reading the HX711 24-bit ADC for load cells / weight scales / tensometric beams.
In hx711_spi_g431cb MOSI is used as CLK to produce the required number of cycles/pulses for HX711.
It enables non-blocking SPI DMA by calling HAL_SPI_TransmitReceive_DMA() and applying moving average
there after. MOSI as CLK for HX711 original idea copied from https://github.com/ufnalski/hx711_non_blocking_spi_g431kb
