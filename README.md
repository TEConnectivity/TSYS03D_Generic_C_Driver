# TSYS03 Generic C Driver
This is a generic C driver for [TSYS03 sensor] (https://www.te.com/usa-en/product-CAT-DTS0001.html).

![TSYS03](https://www.te.com/content/dam/te-com/catalog/part/CAT/DTS/000/CAT-DTS0001-t1.jpg/jcr:content/renditions/product-details.png)

The TSYS3 is a miniature digital temperature sensor that provides factory calibrated highly accurate temperature data. The device contains a durable temperature sensor element, A/D converter, and microcontroller to manage data communications via an I2C interface. TSYS3 is available in a TDFN8 or a XDFN6 package to easily adapt to the space available on a PC board. These packages are very small and have low thermal mass which provides a quick response to temperature changes.

### Specifications
*	Measures temperature from -40°C to 125°C
*	Temperature resolution of +-0.01°C
*	Typical accuracy +-0.5°C, from 0°C to 60°C
*	5 µA supply current
*	< 0.4µA standby current at 25°C
*	I2C interface up to 1MHz
*	Programmable I2C address

### Driver features
* TSYS03 Reset request
* Launch and read temperature measurement
* Read TSYS03 Serial Number
* Update programmable I2C address

**NB:** This driver is intended to provide an implementation example of the sensor communication protocol, in order to be usable you have to implement a proper I2C layer for your target platform.