# STM32 Temperature and Humidity Projects

These two projects measure **temperature** and **humidity** using an STM32 microcontroller and the **X-NUCLEO-IKS01A2** expansion board.  

Both projects are based on the **STM32L053R8** MCU and communicate with the **HTS221** sensor over I²C. Sensor data is printed to a serial terminal via UART.  

The main difference between the projects is the type of communication:

- `stm32_peripheral_project` uses **polling** communication.  
- `stm32_peripheral_irq_project` uses **interrupt** communication.

---

## Requirements

- STM32L053R8 development board  
- X-NUCLEO-IKS01A2 sensor expansion board  
- PuTTY or any other serial terminal  
- 1× pin connector wire (male–female)  
- USB cable  

---

## How It Works

### Polling Project (`stm32_peripheral_project`)

1. The MCU continuously polls the **HTS221** sensor on the expansion board.  
2. When new data becomes available, the STM32 reads temperature and humidity values over I²C.  
3. The data is then sent to a serial terminal via UART.  

### Interrupt Project (`stm32_peripheral_irq_project`)

1. The MCU initiates a read request to the **HTS221** sensor via its I²C module.  
2. The I²C module starts receiving data from the sensor.  
3. When the reception is complete, the I²C module triggers an **interrupt** and calls a callback function.  
4. The received data is available in the MCU buffer.  
5. The data is then transmitted to the serial terminal using the same interrupt logic via UART.
