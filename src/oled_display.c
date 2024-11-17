#include "stm32f0xx.h"
#include <stdio.h>

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
// void init_spi2(void) {
//     RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
//     RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
//     GPIOB->MODER &= 0xBAFFFFFF;
//     GPIOB->MODER |= 0x8A000000;
//     SPI2->CR1 &= ~SPI_CR1_SPE;
//     SPI2->CR1 |= SPI_CR1_BR;
//     SPI2->CR2 |= SPI_CR2_DS;
//     SPI2->CR1 |= SPI_CR1_MSTR;
//     SPI2->CR2 |= SPI_CR2_SSOE;
//     SPI2->CR2 |= SPI_CR2_NSSP;
//     SPI2->CR2 |= SPI_CR2_TXDMAEN;
//     SPI2->CR1 |= SPI_CR1_SPE;
// }

// // ===========================================================================
// // Configure the SPI2 peripheral to trigger the DMA channel when the
// // transmitter is empty.  Use the code from setup_dma from lab 5.
// // ===========================================================================
// void spi2_setup_dma(void) {
//     RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//     DMA1_Channel5->CCR &= ~DMA_CCR_EN;
//     DMA1_Channel5->CMAR = (uint32_t*) msg;
//     DMA1_Channel5->CPAR = &SPI2->DR;
//     DMA1_Channel5->CNDTR = 8;
//     DMA1_Channel5->CCR |= DMA_CCR_DIR;
//     DMA1_Channel5->CCR |= DMA_CCR_MINC;
//     DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;
//     DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;
//     DMA1_Channel5->CCR |= DMA_CCR_CIRC;
//     SPI2->CR2 |= SPI_CR2_TXDMAEN;
// }

// //===========================================================================
// // Enable the DMA channel.
// //===========================================================================
// void spi2_enable_dma(void) {
//     DMA1_Channel5->CCR |= DMA_CCR_EN;
// }

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