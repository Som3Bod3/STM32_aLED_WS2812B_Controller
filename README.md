# STM32_aLED_WS2812B_Controller

STM32_aLED_WS2812B_Controller is a project that uses a STM32F446RE ARM Cortex M4 microcontroller to drive a 16x16 LED matrix consisting of WS2812B LEDs. The controller uses PWM and DMA channels for control using a NZR protocol that was implemented. The driver remembers its state after power loss thanks to FLASH memory implementation. The LED matrix is controlled via a 6-button keyboard that the user can use to change colors, brightness, animation modes, animation speeds, and upload icons.

The controller features several animations, including one solid color, rainbow, pulse, wave, and others. The communication between the STM32 controller and other devices is established via a UART interface protocol, which uses a circular buffer and check sum to detect errors. The communication protocol was designed and implemented from scratch.

The project was developed in STM32CubeIDE.

## Features

- 16x16 LED matrix consisting of WS2812B LEDs
- PWM and DMA channels for control
- FLASH memory implementation for state memory
- 6-button keyboard for control
- Ability to change colors, brightness, animation modes, animation speeds, and upload icons
- Several animation modes, including one solid color, rainbow, pulse, wave, and others
- UART interface protocol with circular buffer and check sum error detection
- NZR protocol implementation for LED control with PWM and DMA

## Technology Stack

- STM32F446RE ARM Cortex M4 microcontroller
- STM32CubeIDE
- 16x16 LED matrix consisting of WS2812B LEDs
- 6x tact switch keyboard connected with pulldowns
- Power source, one for STM and one for the LEDs(12v 15A for full brightness)

## Installation

To use the STM32_aLED_WS2812B_Controller, follow these steps:

1. Clone the repository to your local machine.
2. Open the STM32CubeIDE and import the project.
3. Connect the STM32F446RE ARM Cortex M4 microcontroller to the 16x16 LED matrix consisting of WS2812B LEDs and the 6-button keyboard.
4. Compile the project and upload it to the microcontroller.

## Usage

Once the STM32_aLED_WS2812B_Controller is installed, the user can control the LED matrix using the 6-button keyboard. The user can change colors, brightness, animation modes, animation speeds, and upload icons. The animations include solid color, rainbow, pulse, wave, and others.

The communication protocol with other devices can be used to send commands to the controller and receive status updates. The circular buffer and checksum are used to detect errors in the communication. Message me to get the list of commands.


