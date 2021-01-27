# LIS3DHH
Arduino library to support the LIS3DHH 3D accelerometer

## API

This sensor uses SPI to communicate.

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LIS3DHHSensor Accelero(&dev_spi, CS_PIN);
    Accelero.begin();
    Accelero.Enable_X();

The access to the sensor values is done as explained below:  

  Read accelerometer.  

    int32_t accelerometer[3];
    Accelero.Get_X_Axes(&accelerometer);  

## Documentation

You can find the source files at  
https://github.com/stm32duino/LIS3DHH

The LIS3DHH datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/accelerometers/lis3dhh.html
