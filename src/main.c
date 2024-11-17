#include "stm32f0xx.h"
#include <stdio.h>

void set_char_msg(int, char);
void nano_wait(unsigned int);
void game(void);
void internal_clock();

//===========================================================================
// Configure GPIOC
//===========================================================================
void enable_ports(void) {
    // Only enable port C for the keypad
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~0xffff;
    GPIOC->MODER |= 0x55 << (4*2);
    GPIOC->OTYPER &= ~0xff;
    GPIOC->OTYPER |= 0xf0;
    GPIOC->PUPDR &= ~0xff;
    GPIOC->PUPDR |= 0x55;
}


uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//===========================================================================
// Bit Bang SPI LED Array
//===========================================================================
int msg_index = 0;
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];




//===========================================================================
// Configure timer 7 to invoke the update interrupt at 1kHz
// Copy from lab 4 or 5.
//===========================================================================
void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 4799;
    TIM7->ARR = 9;
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] = 1 << TIM7_IRQn;
    TIM7->CR1 |= TIM_CR1_CEN;
}


//===========================================================================
// Copy the Timer 7 ISR from lab 5
//===========================================================================
// TODO To be copied
void TIM7_IRQHandler() {
    TIM7->SR &= ~TIM_SR_UIF;
    int rows = read_rows();
    update_history(col, rows);
    col = (col + 1) & 3;
    drive_column(col);
}

// ===========================================================================
// Initialize the SPI2 peripheral.
// ===========================================================================
void init_spi2(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    GPIOB->MODER &= 0xBAFFFFFF;
    GPIOB->MODER |= 0x8A000000;
    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 |= SPI_CR1_BR;
    SPI2->CR2 |= SPI_CR2_DS;
    SPI2->CR1 |= SPI_CR1_MSTR;
    SPI2->CR2 |= SPI_CR2_SSOE;
    SPI2->CR2 |= SPI_CR2_NSSP;
    SPI2->CR2 |= SPI_CR2_TXDMAEN;
    SPI2->CR1 |= SPI_CR1_SPE;
}

// ===========================================================================
// Configure the SPI2 peripheral to trigger the DMA channel when the
// transmitter is empty.  Use the code from setup_dma from lab 5.
// ===========================================================================
void spi2_setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CMAR = (uint32_t*) msg;
    DMA1_Channel5->CPAR = &SPI2->DR;
    DMA1_Channel5->CNDTR = 8;
    DMA1_Channel5->CCR |= DMA_CCR_DIR;
    DMA1_Channel5->CCR |= DMA_CCR_MINC;
    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel5->CCR |= DMA_CCR_CIRC;
    SPI2->CR2 |= SPI_CR2_TXDMAEN;
}

//===========================================================================
// Enable the DMA channel.
//===========================================================================
void spi2_enable_dma(void) {
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

//===========================================================================
// 4.4 SPI OLED Display
//===========================================================================
void init_spi1() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    GPIOA->MODER &= ~0xC000CC00;
    GPIOA->MODER |= 0x80008800;
    GPIOA->AFR[0] &= ~0xF0F00000;
    GPIOA->AFR[1] &= ~0xF0000000;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_BR;
    SPI1->CR2 |= SPI_CR2_DS_3 | SPI_CR2_DS_0;
    SPI1->CR2 &= ~SPI_CR2_DS_2;
    SPI1->CR2 &= ~SPI_CR2_DS_1;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR2 |= SPI_CR2_SSOE;
    SPI1->CR2 |= SPI_CR2_NSSP;
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    SPI1->CR1 |= SPI_CR1_SPE;
}
void spi_cmd(unsigned int data) {
    while (!(SPI1->SR & SPI_SR_TXE)) {
        SPI1->DR = data;
    }
}

void spi1_init_oled() {
    nano_wait(1000000);
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0C);
}

//===========================================================================
// This is the 34-entry buffer to be copied into SPI1.
// Each element is a 16-bit value that is either character data or a command.
// Element 0 is the command to set the cursor to the first position of line 1.
// The next 16 elements are 16 characters.
// Element 17 is the command to set the cursor to the first position of line 2.
//===========================================================================
uint16_t display[34] = {
        0x002, // Command to set the cursor at the first position line 1
        0x200+'E', 0x200+'C', 0x200+'E', 0x200+'3', 0x200+'6', + 0x200+'2', 0x200+' ', 0x200+'i',
        0x200+'s', 0x200+' ', 0x200+'t', 0x200+'h', + 0x200+'e', 0x200+' ', 0x200+' ', 0x200+' ',
        0x0c0, // Command to set the cursor at the first position line 2
        0x200+'c', 0x200+'l', 0x200+'a', 0x200+'s', 0x200+'s', + 0x200+' ', 0x200+'f', 0x200+'o',
        0x200+'r', 0x200+' ', 0x200+'y', 0x200+'o', + 0x200+'u', 0x200+'!', 0x200+' ', 0x200+' ',
};

//===========================================================================
// Configure the proper DMA channel to be triggered by SPI1_TX.
// Set the SPI1 peripheral to trigger a DMA when the transmitter is empty.
//===========================================================================
void spi1_setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CMAR = (uint32_t*) display;
    DMA1_Channel3->CPAR = &SPI1->DR;
    DMA1_Channel3->CNDTR = 34;
    DMA1_Channel3->CCR |= DMA_CCR_DIR;
    DMA1_Channel3->CCR |= DMA_CCR_MINC;
    DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel3->CCR |= DMA_CCR_CIRC;
}

//===========================================================================
// Enable the DMA channel triggered by SPI1_TX.
//===========================================================================
void spi1_enable_dma(void) {
    DMA1_Channel3->CCR |= DMA_CCR_EN;
}

//===========================================================================
// Main function
//===========================================================================

// #define ORIG
#ifdef ORIG
int main(void) {
    internal_clock();

    msg[0] |= font['E'];
    msg[1] |= font['C'];
    msg[2] |= font['E'];
    msg[3] |= font[' '];
    msg[4] |= font['3'];
    msg[5] |= font['6'];
    msg[6] |= font['2'];
    msg[7] |= font[' '];

    // GPIO enable
    enable_ports();
    // setup keyboard
    init_tim7();

    // Game on!  The goal is to score 100 points.
    game();
}
#endif

#include "stm32f0xx.h"
#include "notes.h"
#include "lcd.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

void internal_clock();
void update_screen(void);
void main_game();

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

    // Enable the timer
    TIM1->CR1 |= TIM_CR1_CEN;

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

    // Enable the timer
    TIM2->CR1 |= TIM_CR1_CEN;

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
    init_tim1();
    init_tim2();
    for (;;) {
        if (screen_update_flag) {
            update_screen();
            screen_update_flag = false;
        }
    }
}

#define TEST

#ifdef TEST
#include "stdio.h"

int main() {
    internal_clock();
    init_usart5();
    enable_tty_interrupt();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    main_game();
}

#endif