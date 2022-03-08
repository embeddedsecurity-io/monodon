# Monodon

This project is part of [embeddedsecurity.io](https://embeddedsecurity.io). It provides a firmware for the famous STM32 Bluepills. This firmware  has different features implemented with the goal to provide easy examples for **embedded protocols analysis**.

This example is used in :

* -----
* -----
* -----


## Firmware
The firmware implements:
* UART1 on pins PA9 and PA9 is only TX and does print out logs from the other interfaces. It also prints "Hello World!" every 5 seconds. This is based on TIM2 interrupt, a prescale divisor of 10000 and the internal clock.
* I2C1 on PB6 and PB7 ...
* UART2 on PA2 and PA3 ...