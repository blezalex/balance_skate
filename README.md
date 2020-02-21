# Wheelie mode for a skate

## Parts:
* [STM32F401CCU6](https://www.amazon.com/gp/product/B07XBWGF9M/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)
* HC-05 bluetooth module
* MPU 6050
* 2x VESC4 or VESC6 (FocBOX might work too)
* PPM remote with receiver

## Operation
* Remote should opeareate in throttle / brakes mode, no reverse.
* When throttle is above 25% or when breakes are more than 25%, tilt the board into wheelie position, the board should start balancing

## Wiring:

#### MPU 6050
* VCC -> VCC
* GND -> GND
* SDA -> PB7
* SCL -> PB6

#### Bluetooth

Configure module for 115200 baud, 8bit, no parity, 1 stop bit

* VCC -> VCC
* GND -> GND
* BT RX -> PA11
* BT TX -> PA12

#### VESC

Configure VESC in FOC mode, ensure motor runs smoothly.

APP: USART

#### VESC1: 
* 5v -> STM32F401CCU6 5v
* GND -> GND
* RX -> PA9
* TX -> PA10

#### VESC2: 
* 5v not connected
* GND -> GND
* RX -> PA2
* TX -> PA3


#### Remote receiver:
* Throttle channel PPM signal -> PB0
* GND->GND
* VCC->VCC

## Flashing
Use [STLINK V2](https://www.ebay.com/sch/i.html?_from=R40&_trksid=p2380057.m570.l1311.R1.TR11.TRC1.A0.H0.Xstlink.TRS0&_nkw=st-link+v2&_sacat=0)
Use the file attached in releases [hex](https://github.com/blezalex/balance_skate/releases/download/alpha/skate.hex) and upload it via [STM32CubeProg](https://www.st.com/en/development-tools/stm32cubeprog.html)

#### Connection
* GND
* SWDIO
* SWCLK

## Configuing
Use attached (releases) APP [apk](https://github.com/blezalex/balance_skate/releases/download/alpha/SkateConfig.apk). Here is my setting [file](https://github.com/blezalex/balance_skate/releases/download/alpha/skate.settings)



