# HTS221 Temperature & Humidity Measurement Project

This project measures **temperature** and **humidity** using an STM32 microcontroller and the **X-NUCLEO-IKS01A2** expansion board.  
It is based on the **STM32L053R8** MCU and communicates with the **HTS221** sensor over I²C.  
Sensor data is printed to a serial terminal via UART.

## Requirements

- STM32L053R8 development board  
- X-NUCLEO-IKS01A2 sensor expansion board  
- PuTTY or any other serial terminal  
- 1× pin connector wire (male–female)  
- USB cable  

## How It Works

The MCU continuously polls the **HTS221** sensor on the expansion board.  
When new data becomes available, the STM32 reads temperature and humidity values over I²C and sends them to a serial terminal through UART.
