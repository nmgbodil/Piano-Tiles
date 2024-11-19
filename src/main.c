#include "stm32f0xx.h"
#include "notes.h"
#include "lcd.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

void set_char_msg(int, char);
void nano_wait(unsigned int);
void game(void);
void internal_clock();
void enable_ports();
void init_tim7();
void update_screen(void);
void main_game();
void init_spi2(void);
void spi2_init_oled();
void spi2_setup_dma();
void spi2_enable_dma();
void spi2_dma_display1(const char* str);
void spi2_dma_display2(const char* str);

//===========================================================================
// Main function
//===========================================================================

extern uint8_t col;
extern int msg_index;
extern uint16_t msg[8];
// const char font[];


void init_usart5() {
    // Enable RCC clocks to GPIOC and GPIOD
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;

    // Configure PC12 for USART5_TX
    GPIOC->MODER &= ~GPIO_MODER_MODER12;  // Clear mode bits for PC12
    GPIOC->MODER |= GPIO_MODER_MODER12_1;   // Set alternate function mode for PC12
    //GPIOC->AFR[1] |= 0X2 << 4;
   GPIOC->AFR[1] |= (2 << ((12 - 8) * 4)); // Set AF2 (USART5_TX) for PC12

    // Configure PD2 for USART5_RX
    GPIOD->MODER &= ~GPIO_MODER_MODER2;   // Clear mode bits for PD2
    GPIOD->MODER |= GPIO_MODER_MODER2_1;    // Set alternate function mode for PD2
    //GPIOD->AFR[0] |= 0X2 << 2;
    GPIOD->AFR[0] |= (2 << (2 * 4));   // Set AF2 (USART5_RX) for PD2

    // Enable RCC clock to USART5 peripheral
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

    // Disable USART5 by clearing the UE bit
    USART5->CR1 &= ~USART_CR1_UE;

    // Set word size to 8 bits (default)
    USART5->CR1 &= ~USART_CR1_M;

    // Set one stop bit
    USART5->CR2 &= ~USART_CR2_STOP;

    // Set no parity control
    USART5->CR1 &= ~(USART_CR1_PCE | USART_CR1_OVER8);

    // Set baud rate to 115200
    USART5->BRR = 0X1A1;

    // Enable transmitter and receiver
    USART5->CR1 |= USART_CR1_TE | USART_CR1_RE;

    // Enable USART5
    USART5->CR1 |= USART_CR1_UE;

    // Wait for TEACK and REACK
    while (!(USART5->ISR & USART_ISR_TEACK) & !(USART5->ISR & USART_ISR_REACK));
}

#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

#include"fifo.h"
#include"tty.h"

void enable_tty_interrupt(void) {
    USART5->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART3_8_IRQn);
    USART5->CR3 |= USART_CR3_DMAR;
    RCC->AHBENR |= RCC_AHBENR_DMA2EN;

    DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;

    DMA2_Channel2->CMAR = (uint32_t)serfifo;
    DMA2_Channel2->CPAR = (uint32_t)&USART5->RDR;
    DMA2_Channel2->CNDTR = FIFOSIZE;
    DMA2_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL | DMA_CCR_EN;
}

char interrupt_getchar() {
    while (!fifo_newline(&input_fifo)) {
        asm volatile ("wfi");
    }
    return fifo_remove(&input_fifo);
}

int __io_putchar(int c) {
    if (c == '\n') {
        while (!(USART5->ISR & USART_ISR_TXE));
        USART5->TDR = '\r';
    }
    while (!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    return interrupt_getchar();
}

void USART3_8_IRQHandler(void) {
    while (DMA2_Channel2->CNDTR != (FIFOSIZE - seroffset)) {
        if (!fifo_full(&input_fifo))
            insert_echo_char(serfifo[seroffset]);
        seroffset = (seroffset + 1) % FIFOSIZE;
    }
}

void init_spi1_slow(void) {
    // Enable the clock for GPIOB and SPI1
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;   // Enable GPIOB clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable SPI1 clock

    // Set PB3, PB4, PB5 to Alternate Function mode (AF0 for SPI1)
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);  // Clear mode bits
    GPIOB->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);  // Set to AF mode

    GPIOB->AFR[0] &= ~((0xF << (3 * 4)) | (0xF << (4 * 4)) | (0xF << (5 * 4)));  // Clear alternate function
    GPIOB->AFR[0] |= (0x0 << (3 * 4)) | (0x0 << (4 * 4)) | (0x0 << (5 * 4));    // Set AF0 for PB3, PB4, PB5

    // Configure SPI1 settings in CR1 register
    SPI1->CR1 = SPI_CR1_MSTR        // Set to Master mode
              | SPI_CR1_BR          // Set baud rate divisor to maximum for slowest speed
              | SPI_CR1_SSM         // Enable Software Slave Management
              | SPI_CR1_SSI;        // Set Internal Slave Select

    // Set data size to 8 bits in CR2 register and configure FIFO reception threshold
    SPI1->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;

    // Enable the SPI1 peripheral
    SPI1->CR1 |= SPI_CR1_SPE;
}


void enable_sdcard(void) {
    // Enable the GPIOB clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure PB2 as an output
    GPIOB->MODER &= ~GPIO_MODER_MODER2;   // Clear mode bits for PB2
    GPIOB->MODER |= GPIO_MODER_MODER2_0;  // Set PB2 to output mode

    // Set PB2 low to enable the SD card
    GPIOB->BSRR = GPIO_BSRR_BR_2;  // Reset (clear) PB2, setting it low
}

void disable_sdcard(void) {
    // Set PB2 high to disable the SD card
    GPIOB->BSRR = GPIO_BSRR_BS_2;  // Set PB2, driving it high
}

void init_sdcard_io(void) {
    // Initialize SPI1 in slow mode for SD card communication
    init_spi1_slow();

    // Enable the GPIOB clock (if not already enabled)
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure PB2 as an output
    GPIOB->MODER &= ~GPIO_MODER_MODER2;   // Clear mode bits for PB2
    GPIOB->MODER |= GPIO_MODER_MODER2_0;  // Set PB2 to output mode

    // Initially disable the SD card by setting PB2 high
    disable_sdcard();
}

void sdcard_io_high_speed(void) {
    // Disable the SPI1 channel
    SPI1->CR1 &= ~SPI_CR1_SPE;

    // Set the baud rate divisor to get a 12 MHz SPI clock (assuming 48 MHz system clock)
    // f_PCLK / 4 -> 48 MHz / 4 = 12 MHz
    SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR) | (0x1 << SPI_CR1_BR_Pos);

    // Re-enable the SPI1 channel
    SPI1->CR1 |= SPI_CR1_SPE;
}

void init_lcd_spi(void) {
    // Enable the GPIOB clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure PB8, PB11, and PB14 as outputs
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14);  // Clear mode bits
    GPIOB->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0);  // Set as output mode

    // Initialize SPI1 at a slow speed
    init_spi1_slow();

    // Configure SPI1 to run at high speed for the LCD
    sdcard_io_high_speed();
}

#define SCREEN_HEIGHT 320
#define TILE_WIDTH 53
#define TILE_HEIGHT 30
#define NUM_COLUMNS 4
#define NUM_NOTES 26

int current_time = 0;       // Current song time (in ms), updated by Timer 1
int tile_duration = 2000;   // Time for a tile to move from top to bottom (in ms)
int game_speed = 1;             // Game speed multiplier (1 = easy, 2 = med, 3 = hard)
int song_length = 16500;
bool screen_update_flag = false;

extern Note song_notes[NUM_NOTES];

void init_tim1(void) {
    // Enable clock for Timer 1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Set the timer period
    // Assuming the timer clock is 48 MHz and we want a period of 10 ms:
    TIM1->PSC = 48000 - 1;  // Prescaler to divide the clock down to 1 kHz (1 ms per count)
    TIM1->ARR = 10 - 1;     // Auto-reload value for 10 ms period

    // Enable update interrupt
    TIM1->DIER |= TIM_DIER_UIE;

    // Disable the timer
    TIM1->CR1 &= ~TIM_CR1_CEN;

    // Enable Timer 1 interrupt in NVIC
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1); // Set priority
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
    if (TIM1->SR & TIM_SR_UIF) {  // Check if the update interrupt flag is set
        TIM1->SR &= ~TIM_SR_UIF;  // Clear the update interrupt flag

        // Perform any periodic task here (e.g., updating the current time)
        current_time += 10; // Increment the timer (10 ms per interrupt)
        if (current_time >= song_length) {
            current_time = 0; // Wrap around at the end of the song
        }
    }
}


void init_tim2(void) {
    // Enable clock for Timer 2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set the timer period
    // Assuming the timer clock is 48 MHz and we want a frame rate of 30 Hz:
    TIM2->PSC = 48000 - 1;  // Prescaler to divide the clock down to 1 kHz (1 ms per count)
    TIM2->ARR = 33 - 1;     // Auto-reload value for ~30 Hz (1000 ms / 30 frames)

    // Clear any pending interrupt flags
    TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag

    // Enable update interrupt
    TIM2->DIER |= TIM_DIER_UIE;

    // Disable the timer
    TIM2->CR1 &= ~TIM_CR1_CEN;

    // Enable Timer 2 interrupt in NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 2); // Set priority (lower than Timer 1 if necessary)
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) { // Check if the interrupt flag is set
        TIM2->SR &= ~TIM_SR_UIF; // Clear the interrupt flag

        // Turn on flag to update screen
        screen_update_flag = true;
    }
}

void set_song_speed(int speed) {
    if (speed == 1) { // Easy
        TIM1->ARR = 10 - 1; // 10 ms increments
    } else if (speed == 2) { // Medium
        TIM1->ARR = 7 - 1; // 7 ms increments (faster song)
    } else if (speed == 3) { // Hard
        TIM1->ARR = 5 - 1; // 5 ms increments (even faster song)
    }
}

void set_screen_update_speed(int speed) {
    if (speed == 1) { // Easy
        TIM2->ARR = 33 - 1; // ~30 Hz frame rate
    } else if (speed == 2) { // Medium
        TIM2->ARR = 25 - 1; // ~40 Hz frame rate
    } else if (speed == 3) { // Hard
        TIM2->ARR = 16 - 1; // ~60 Hz frame rate
    }
}

void update_screen(void) {
    for (int i = 0; i < NUM_NOTES; i++) {
        // Check if the note is active
        if (song_notes[i].start_time >= current_time - tile_duration &&
            song_notes[i].start_time <= current_time) {

            // Calculate the y-position based on time
            int elapsed_time = current_time - song_notes[i].start_time;
            int y_position = (elapsed_time * SCREEN_HEIGHT) / tile_duration;

            // Clear the previous tile position (if it exists)
            if (y_positions[i] != -1) { // -1 means no tile previously
                LCD_DrawFillRectangle(
                    x_positions[i], 
                    y_positions[i], 
                    x_positions[i] + TILE_WIDTH, 
                    y_positions[i] + TILE_HEIGHT, 
                    0x0000 // Black to clear the tile
                );
            }

            // Update the new tile position
            y_positions[i] = y_position;

            // Assign a random column (x-position) if not already assigned
            if (x_positions[i] == -1) { // Use -1 as a marker for uninitialized tiles
                x_positions[i] = (rand() % NUM_COLUMNS) * (TILE_WIDTH + 10);
            }

            // Draw the tile
            LCD_DrawFillRectangle(
                x_positions[i],
                y_positions[i],
                x_positions[i] + TILE_WIDTH,
                y_positions[i] + TILE_HEIGHT,
                song_notes[i].color
            );
        } else {
            // Reset the tile's position when it goes off-screen
            y_positions[i] = -1;
            x_positions[i] = -1;
        }
    }
}


void main_game() {
    for (int i = 0; i < NUM_NOTES; i++) {
        y_positions[i] = -1;
    }
    
    LCD_Setup();
    LCD_Clear(0x0000);
    set_song_speed(game_speed);
    set_screen_update_speed(game_speed);

    TIM1->CR1 |= TIM_CR1_CEN; // Enable timer 1
    TIM2->CR1 |= TIM_CR1_CEN; // Enable timer 2

    init_spi2();
    spi2_init_oled();
    spi2_setup_dma();
    spi2_enable_dma();
    spi2_dma_display1("Hello!");
    spi2_dma_display2("Pick Mode: A B C");

    
    for (;;) {
        if (screen_update_flag) {
            update_screen();
            screen_update_flag = false;
        }
    }
}

#define MAIN
#ifdef MAIN
#include "stdio.h"

int main() {
    internal_clock();

    enable_ports();
    init_tim7(); // setup keyboard
    init_tim1();
    init_tim2();

    init_usart5();
    enable_tty_interrupt();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);

    main_game();
}
#endif

// #define ORIG
#ifdef ORIG
#include <stdio.h>
int main(void) {
    internal_clock();

    // msg[0] |= font['E'];
    // msg[1] |= font['C'];
    // msg[2] |= font['E'];
    // msg[3] |= font[' '];
    // msg[4] |= font['3'];
    // msg[5] |= font['6'];
    // msg[6] |= font['2'];
    // msg[7] |= font[' '];

    // GPIO enable
    enable_ports();
    // setup keyboard
    init_tim7();

    // Game on!  The goal is to score 100 points.
    game();
}
#endif