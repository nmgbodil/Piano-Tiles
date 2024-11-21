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
char get_keypress();
void check_button_press(int index);
char get_key_event();
void update_score(int val);
void mistake(const char* mode);
void set_tile_color(int color);
void end_game();


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
#define NUM_TILES 20  // Number of active tiles we'll track

typedef struct {
    int x;
    int y;
    bool active;
    int column;
} Tile;

// Global variables
Tile tiles[NUM_TILES];
int score = 0;
bool screen_update_flag = false;
int lives = 3;  // Start with 3 lives
bool game_over = false;
uint16_t current_tile_color = GREEN;

// Add these globals at the top
#define SPEED_EASY 3      // Slower tile movement
#define SPEED_MEDIUM 5    // Medium tile movement
#define SPEED_HARD 7      // Faster tile movement
int current_speed = SPEED_EASY;
bool game_started = false;
int multiplier = 10;  // Start with 10x multiplier
uint16_t ORANGE = 0xFD20;  // Define orange color

// Initialize a new tile at the top of a random column
void spawn_tile() {
    for (int i = 0; i < NUM_TILES; i++) {
        if (!tiles[i].active) {
            int column = rand() % NUM_COLUMNS;
            tiles[i].column = column;
            tiles[i].x = column * (TILE_WIDTH + 10);
            tiles[i].y = 0;
            tiles[i].active = true;
            break;
        }
    }
}

void update_screen() {
    if (game_over) {
        // Keep tiles current color when game is over
        for (int i = 0; i < NUM_TILES; i++) {
            if (tiles[i].active) {
                LCD_DrawFillRectangle(
                    tiles[i].x,
                    tiles[i].y,
                    tiles[i].x + TILE_WIDTH,
                    tiles[i].y + TILE_HEIGHT,
                    current_tile_color
                );
            }
        }
        return;
    }

    for (int i = 0; i < NUM_TILES; i++) {
        if (tiles[i].active) {
            LCD_DrawFillRectangle(
                tiles[i].x,
                tiles[i].y,
                tiles[i].x + TILE_WIDTH,
                tiles[i].y + TILE_HEIGHT,
                BLACK
            );

            tiles[i].y += current_speed;

            if (tiles[i].y >= SCREEN_HEIGHT - TILE_HEIGHT) {
                tiles[i].active = false;
                lives--;
                
                char lives_str[17];
                snprintf(lives_str, 17, "Lives: %d x%d", lives, multiplier);
                spi2_dma_display1(lives_str);

                // Different behavior for each mode
                switch(current_speed) {
                    case SPEED_EASY:
                        if (lives == 2) {
                            current_tile_color = YELLOW;
                            multiplier = 5;
                        }
                        else if (lives == 1) {
                            current_tile_color = RED;
                            multiplier = 2;
                        }
                        break;
                        
                    case SPEED_MEDIUM:
                        if (lives == 1) {
                            current_tile_color = RED;
                            multiplier = 5;
                        }
                        break;
                }

                if (lives <= 0) {
                    game_over = true;
                    spi2_dma_display1("Game Over!");
                    char final_score[17];
                    snprintf(final_score, 17, "Final Score: %d", score);
                    spi2_dma_display2(final_score);
                    return;
                }
                continue;
            }

            LCD_DrawFillRectangle(
                tiles[i].x,
                tiles[i].y,
                tiles[i].x + TILE_WIDTH,
                tiles[i].y + TILE_HEIGHT,
                current_tile_color
            );
        }
    }

    if (rand() % ((current_speed == SPEED_EASY) ? 35 : 
                  (current_speed == SPEED_MEDIUM) ? 30 : 25) == 0) {
        spawn_tile();
    }
}

void check_button_presses() {
    char key = get_key_event();
    if (key & 0x80) {  // If it's a press event
        key &= 0x7F;   // Remove the press flag
        
        int pressed_column;
        switch(key) {
            case 'A': pressed_column = 0; break;
            case 'B': pressed_column = 1; break;
            case 'C': pressed_column = 2; break;
            case 'D': pressed_column = 3; break;
            default: return;
        }

        // Check for hits
        for (int i = 0; i < NUM_TILES; i++) {
            if (tiles[i].active && 
                tiles[i].column == pressed_column && 
                tiles[i].y >= SCREEN_HEIGHT - 2*TILE_HEIGHT) {
                
                score += (10 * multiplier);  // Apply multiplier to score
                tiles[i].active = false;
                
                // Update score display
                char score_str[17];
                snprintf(score_str, 17, "Score: %d x%d", score, multiplier);
                spi2_dma_display2(score_str);
                break;
            }
        }
    }
}

void main_game() {
    LCD_Setup();
    LCD_Clear(BLACK);

    init_spi2();
    spi2_init_oled();
    spi2_setup_dma();
    spi2_enable_dma();
    
    spi2_dma_display1("Pick Mode:");
    spi2_dma_display2("A:Easy B:Med C:Hard");

    while (!game_started) {
        char key = get_key_event();
        if (key & 0x80) {
            key &= 0x7F;
            
            switch(key) {
                case 'A':
                    current_speed = SPEED_EASY;
                    lives = 3;
                    current_tile_color = GREEN;
                    multiplier = 10;
                    spi2_dma_display1("Mode: Easy x10");
                    game_started = true;
                    break;
                case 'B':
                    current_speed = SPEED_MEDIUM;
                    lives = 2;
                    current_tile_color = GREEN;
                    multiplier = 10;
                    spi2_dma_display1("Mode: Medium x10");
                    game_started = true;
                    break;
                case 'C':
                    current_speed = SPEED_HARD;
                    lives = 1;
                    current_tile_color = ORANGE;
                    multiplier = 10;
                    spi2_dma_display1("Mode: Hard x10");
                    game_started = true;
                    break;
            }
        }
    }

    nano_wait(1000000000);

    for (int i = 0; i < NUM_TILES; i++) {
        tiles[i].active = false;
    }

    score = 0;
    game_over = false;

    char lives_str[17];
    snprintf(lives_str, 17, "Lives: %d x%d", lives, multiplier);
    spi2_dma_display1(lives_str);
    spi2_dma_display2("Score: 0 x10");

    init_tim7();
    
    while(1) {
        if (!game_over) {
            update_screen();
            check_button_presses();
            nano_wait(100000000);
        }
    }
}

#define MAIN
#ifdef MAIN
#include <stdio.h>

int main() {
    internal_clock();
    enable_ports();
    init_tim7(); // setup keyboard

    init_usart5();
    enable_tty_interrupt();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);

    main_game();
}
#endif