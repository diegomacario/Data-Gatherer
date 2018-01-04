# Data-Gatherer

A real-time temperature and tilt measurement device.

<p align="center">
  <img src="https://github.com/diegomacario/Data-Gatherer/blob/master/readme_images/early_prototype.png"/>
</p>

## Summary

The goal of this project was to design a multicomponent system to explore the interactions between embedded peripherals and a smartphone device. This involved developing an interface to support the communication between an [STM32F4 Discovery board](http://www.st.com/en/evaluation-tools/stm32f4discovery.html), an [STM32F401RE Nucleo board](http://www.st.com/en/evaluation-tools/nucleo-f401re.html) and an Android smartphone.

Two-way communication between the Nucleo board and the smartphone was achieved over Bluetooth Low Energy (BLE) by fitting the Nucleo board with an [IDB04A1 BLE shield](http://www.st.com/en/ecosystems/x-nucleo-idb04a1.html), while communication between the Discovery board and the Nucleo board was achieved using the Serial Peripheral Interface (SPI) protocol and a system of binary codes.

The final system allows for temperature and tilt measurements to be sent from the Discovery board to the smartphone, which displays these values on its screen. The Discovery board also supports a double tap function which raises a notification on the smartphone when a double tap is detected, and the smartphone has controls for the LEDs included on the Discovery board.

The final system, including all of its components and connections, is illustrated below. For a detailed description, see [this](https://github.com/diegomacario/Data-Gatherer/blob/master/documentation/data_gatherer.pdf) document.

<p align="center">
  <img src="https://github.com/diegomacario/Data-Gatherer/blob/master/readme_images/complete_system.png"/>
</p>
