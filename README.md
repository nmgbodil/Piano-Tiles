# Piano Tiles - STM32 Project

## Overview
Piano Tiles is a rhythm-based game implemented on an STM32F0 microcontroller. The game features falling tiles that the player must press in time to earn points. Additionally, it plays the melody of "Twinkle Twinkle Little Star" as background music.

## Features
- **Real-time Tile Movement:** Tiles fall at different speeds based on the selected difficulty mode.
- **Button Press Interactions:** Players press corresponding buttons to match the falling tiles.
- **Scoring System:** Players earn points by hitting tiles correctly, with multipliers for streaks.
- **Lives System:** Players start with a limited number of lives; missing tiles reduces them.
- **Multiple Difficulty Levels:** Easy, Medium, and Hard modes control tile speed and available lives.
- **Sound Integration:** Uses a PWM-based buzzer system to play notes of "Twinkle Twinkle Little Star."
- **OLED Display:** Game information such as score, lives, and mode selection is shown on an OLED screen.

## Hardware and Software Requirements
### Hardware
- **Microcontroller:** STM32F0 series (tested on STM32F0xx)
- **Peripherals:**
  - SPI for OLED display
  - USART for debugging
  - GPIO for button inputs
  - Timer-based PWM for sound generation
  - SD card support
- **Other Components:**
  - OLED LCD Display
  - Physical buttons for input
  - Buzzer for audio feedback

### Software
- **Development Environment:**
  - ARM-GCC toolchain
  - OpenOCD for debugging
  - STM32CubeIDE or other IDEs supporting CMSIS
- **Dependencies:**
  - CMSIS framework for STM32 development

## Installation & Setup
1. Clone the repository containing the project files.
2. Connect the STM32 board to your development environment.
3. Compile the firmware using the ARM-GCC toolchain or STM32CubeIDE.
4. Flash the compiled binary onto the STM32 board using OpenOCD or ST-Link Utility.
5. Ensure all peripherals (OLED, buttons, buzzer) are properly connected.
6. Start the game by powering on the board.

## Usage Instructions
### Starting the Game
- The OLED screen will display mode selection:
  - **A:** Easy Mode (3 Lives, Slow Tiles)
  - **B:** Medium Mode (2 Lives, Medium Speed)
  - **C:** Hard Mode (1 Life, Fast Tiles)
- Press the corresponding button to choose a mode.
- The game will begin immediately after selection.

### Gameplay
- Tiles fall from the top of the screen.
- Press the correct button to match the falling tile in its column.
- Correct presses increase the score.
- Missing tiles results in losing lives.
- The game ends when all lives are lost.
- A game over message is displayed on the OLED.

## Contributors
- Noddie Mgbodille
- Gabriel Ogbalor
- Amartya Singh
- Aleksandar Ilijevski

## Acknowledgments
Special thanks to the STM32 community and CMSIS developers for providing low-level support and frameworks for embedded development.
