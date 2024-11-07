// /**
//   ******************************************************************************
//   * @file    main.c
//   * @author  Weili An, Niraj Menon
//   * @date    Feb 3, 2024
//   * @brief   ECE 362 Lab 6 Student template
//   ******************************************************************************
// */

// /*******************************************************************************/

// // Fill out your username, otherwise your completion code will have the 
// // wrong username!
// const char* username = "nmgbodil";

// /*******************************************************************************/ 

// #include "stm32f0xx.h"
// #include <stdio.h>

// void set_char_msg(int, char);
// void nano_wait(unsigned int);
// void game(void);
// void internal_clock();
// void check_wiring();
// void autotest();

// //===========================================================================
// // Configure GPIOC
// //===========================================================================
// void enable_ports(void) {
//     // Only enable port C for the keypad
//     RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
//     GPIOC->MODER &= ~0xffff;
//     GPIOC->MODER |= 0x55 << (4*2);
//     GPIOC->OTYPER &= ~0xff;
//     GPIOC->OTYPER |= 0xf0;
//     GPIOC->PUPDR &= ~0xff;
//     GPIOC->PUPDR |= 0x55;
// }


// uint8_t col; // the column being scanned

// void drive_column(int);   // energize one of the column outputs
// int  read_rows();         // read the four row inputs
// void update_history(int col, int rows); // record the buttons of the driven column
// char get_key_event(void); // wait for a button event (press or release)
// char get_keypress(void);  // wait for only a button press event.
// float getfloat(void);     // read a floating-point number from keypad
// void show_keys(void);     // demonstrate get_key_event()

// //===========================================================================
// // Bit Bang SPI LED Array
// //===========================================================================
// int msg_index = 0;
// uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
// extern const char font[];

// //===========================================================================
// // Configure PB12 (CS), PB13 (SCK), and PB15 (SDI) for outputs
// //===========================================================================
// void setup_bb(void) {
//     RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
//     GPIOB->MODER &= 0x75FFFFFF;
//     GPIOB->MODER |= 0x45000000;
//     GPIOB->BSRR |= 0x20001000;
// }

// void small_delay(void) {
//     nano_wait(50000);
// }

// //===========================================================================
// // Set the MOSI bit, then set the clock high and low.
// // Pause between doing these steps with small_delay().
// //===========================================================================
// void bb_write_bit(int val) {
//     // CS (PB12)
//     // SCK (PB13)
//     // SDI (PB15)
//     int bit = val != 0;
//     GPIOB->BSRR |= ((1 << (15 + !bit * 16)));
//     small_delay();
//     GPIOB->BSRR |= 0x00002000;
//     small_delay();
//     GPIOB->BSRR |= 0x20000000;
// }

// //===========================================================================
// // Set CS (PB12) low,
// // write 16 bits using bb_write_bit,
// // then set CS high.
// //===========================================================================
// void bb_write_halfword(int halfword) {
//     GPIOB->BSRR |= 0x10000000;
//     for (int i = 15; i >= 0; i--) {
//         bb_write_bit(halfword & (1 << i));
//     }
//     GPIOB->BSRR |= 0x00001000;
// }

// //===========================================================================
// // Continually bitbang the msg[] array.
// //===========================================================================
// void drive_bb(void) {
//     for(;;)
//         for(int d=0; d<8; d++) {
//             bb_write_halfword(msg[d]);
//             nano_wait(1000000); // wait 1 ms between digits
//         }
// }

// //============================================================================
// // Configure Timer 15 for an update rate of 1 kHz.
// // Trigger the DMA channel on each update.
// // Copy this from lab 4 or lab 5.
// //============================================================================
// void init_tim15(void) {
//     RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
//     TIM15->PSC = 4799;
//     TIM15->ARR = 9;
//     TIM15->DIER |= TIM_DIER_UDE;
//     TIM15->CR1 |= TIM_CR1_CEN;
// }


// //===========================================================================
// // Configure timer 7 to invoke the update interrupt at 1kHz
// // Copy from lab 4 or 5.
// //===========================================================================
// void init_tim7(void) {
//     RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
//     TIM7->PSC = 4799;
//     TIM7->ARR = 9;
//     TIM7->DIER |= TIM_DIER_UIE;
//     NVIC->ISER[0] = 1 << TIM7_IRQn;
//     TIM7->CR1 |= TIM_CR1_CEN;
// }


// //===========================================================================
// // Copy the Timer 7 ISR from lab 5
// //===========================================================================
// // TODO To be copied
// void TIM7_IRQHandler() {
//     TIM7->SR &= ~TIM_SR_UIF;
//     int rows = read_rows();
//     update_history(col, rows);
//     col = (col + 1) & 3;
//     drive_column(col);
// }

// //===========================================================================
// // Initialize the SPI2 peripheral.
// //===========================================================================
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

// //===========================================================================
// // Configure the SPI2 peripheral to trigger the DMA channel when the
// // transmitter is empty.  Use the code from setup_dma from lab 5.
// //===========================================================================
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

// //===========================================================================
// // 4.4 SPI OLED Display
// //===========================================================================
// void init_spi1() {
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
//     GPIOA->MODER &= ~0xC000CC00;
//     GPIOA->MODER |= 0x80008800;
//     GPIOA->AFR[0] &= ~0xF0F00000;
//     GPIOA->AFR[1] &= ~0xF0000000;
//     SPI1->CR1 &= ~SPI_CR1_SPE;
//     SPI1->CR1 |= SPI_CR1_BR;
//     SPI1->CR2 |= SPI_CR2_DS_3 | SPI_CR2_DS_0;
//     SPI1->CR2 &= ~SPI_CR2_DS_2;
//     SPI1->CR2 &= ~SPI_CR2_DS_1;
//     SPI1->CR1 |= SPI_CR1_MSTR;
//     SPI1->CR2 |= SPI_CR2_SSOE;
//     SPI1->CR2 |= SPI_CR2_NSSP;
//     SPI1->CR2 |= SPI_CR2_TXDMAEN;
//     SPI1->CR1 |= SPI_CR1_SPE;
// }
// void spi_cmd(unsigned int data) {
//     while (!(SPI1->SR & SPI_SR_TXE)) {
//         SPI1->DR = data;
//     }
// }
// void spi_data(unsigned int data) {
//     spi_cmd(data | 0x200);
// }
// void spi1_init_oled() {
//     nano_wait(1000000);
//     spi_cmd(0x38);
//     spi_cmd(0x08);
//     spi_cmd(0x01);
//     nano_wait(2000000);
//     spi_cmd(0x06);
//     spi_cmd(0x02);
//     spi_cmd(0x0C);
// }
// void spi1_display1(const char *string) {
//     spi_cmd(0x02);
//     int i = 0;
//     while (string[i] != 0) {
//         spi_data(string[i]);
//         i++;
//     }
// }
// void spi1_display2(const char *string) {
//     spi_cmd(0xC0);
//     int i = 0;
//     while (string[i] != 0) {
//         spi_data(string[i]);
//         i++;
//     }
// }

// //===========================================================================
// // This is the 34-entry buffer to be copied into SPI1.
// // Each element is a 16-bit value that is either character data or a command.
// // Element 0 is the command to set the cursor to the first position of line 1.
// // The next 16 elements are 16 characters.
// // Element 17 is the command to set the cursor to the first position of line 2.
// //===========================================================================
// uint16_t display[34] = {
//         0x002, // Command to set the cursor at the first position line 1
//         0x200+'E', 0x200+'C', 0x200+'E', 0x200+'3', 0x200+'6', + 0x200+'2', 0x200+' ', 0x200+'i',
//         0x200+'s', 0x200+' ', 0x200+'t', 0x200+'h', + 0x200+'e', 0x200+' ', 0x200+' ', 0x200+' ',
//         0x0c0, // Command to set the cursor at the first position line 2
//         0x200+'c', 0x200+'l', 0x200+'a', 0x200+'s', 0x200+'s', + 0x200+' ', 0x200+'f', 0x200+'o',
//         0x200+'r', 0x200+' ', 0x200+'y', 0x200+'o', + 0x200+'u', 0x200+'!', 0x200+' ', 0x200+' ',
// };

// //===========================================================================
// // Configure the proper DMA channel to be triggered by SPI1_TX.
// // Set the SPI1 peripheral to trigger a DMA when the transmitter is empty.
// //===========================================================================
// void spi1_setup_dma(void) {
//     RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//     DMA1_Channel3->CCR &= ~DMA_CCR_EN;
//     DMA1_Channel3->CMAR = (uint32_t*) display;
//     DMA1_Channel3->CPAR = &SPI1->DR;
//     DMA1_Channel3->CNDTR = 34;
//     DMA1_Channel3->CCR |= DMA_CCR_DIR;
//     DMA1_Channel3->CCR |= DMA_CCR_MINC;
//     DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0;
//     DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0;
//     DMA1_Channel3->CCR |= DMA_CCR_CIRC;
// }

// //===========================================================================
// // Enable the DMA channel triggered by SPI1_TX.
// //===========================================================================
// void spi1_enable_dma(void) {
//     DMA1_Channel3->CCR |= DMA_CCR_EN;
// }

// //===========================================================================
// // Main function
// //===========================================================================

// int main(void) {
//     internal_clock();

//     msg[0] |= font['E'];
//     msg[1] |= font['C'];
//     msg[2] |= font['E'];
//     msg[3] |= font[' '];
//     msg[4] |= font['3'];
//     msg[5] |= font['6'];
//     msg[6] |= font['2'];
//     msg[7] |= font[' '];

//     // GPIO enable
//     enable_ports();
//     // setup keyboard
//     init_tim7();

//     // LED array Bit Bang
// // #define BIT_BANG
// #if defined(BIT_BANG)
//     setup_bb();
//     drive_bb();
// #endif

//     // Direct SPI peripheral to drive LED display
// //#define SPI_LEDS
// #if defined(SPI_LEDS)
//     init_spi2();
//     spi2_setup_dma();
//     spi2_enable_dma();
//     init_tim15();
//     show_keys();
// #endif

//     // LED array SPI
// // #define SPI_LEDS_DMA
// #if defined(SPI_LEDS_DMA)
//     init_spi2();
//     spi2_setup_dma();
//     spi2_enable_dma();
//     show_keys();
// #endif

//     // SPI OLED direct drive
// //#define SPI_OLED
// #if defined(SPI_OLED)
//     init_spi1();
//     spi1_init_oled();
//     spi1_display1("Hello again,");
//     spi1_display2(username);
// #endif

//     // SPI
// //#define SPI_OLED_DMA
// #if defined(SPI_OLED_DMA)
//     init_spi1();
//     spi1_init_oled();
//     spi1_setup_dma();
//     spi1_enable_dma();
// #endif

//     // Uncomment when you are ready to generate a code.
//     // autotest();

//     // Game on!  The goal is to score 100 points.
//     game();
// }

#include "stm32f0xx.h"

// Define game modes
typedef enum {
    EASY,
    MEDIUM,
    HARD
} GameMode;

GameMode current_mode = EASY;
const char* mode_strings[] = {"Mode: Easy", "Mode: Medium", "Mode: Hard"};

// Function prototypes
void internal_clock(void);
void enable_ports(void);
void init_spi1(void);
void spi1_init_oled(void);
void spi_cmd(unsigned int data);
void spi_data(unsigned int data);
void display_game_mode(const char *mode);
void setup_button_interrupt(void);
void nano_wait(unsigned int n);
void cycle_game_mode(void);

// Initialize GPIO for PA0 with external interrupt
void setup_button_interrupt(void) {
    // Enable GPIOA clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Configure PA0 as input for button
    GPIOA->MODER &= ~GPIO_MODER_MODER0; // Set PA0 to input mode
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0; // No pull-up, no pull-down

    // Configure the EXTI (External Interrupt) for PA0
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;  // Map EXTI0 to PA0
    EXTI->IMR |= EXTI_IMR_IM0;                   // Unmask interrupt on EXTI0
    EXTI->RTSR |= EXTI_RTSR_TR0;                 // Trigger on rising edge

    // Enable EXTI0 interrupt in NVIC
    NVIC_SetPriority(EXTI0_1_IRQn, 1);           // Set priority
    NVIC_EnableIRQ(EXTI0_1_IRQn);                // Enable EXTI0_1 interrupt
}

// EXTI0 interrupt handler for button press
void EXTI0_1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) { // Check if EXTI0 triggered
        EXTI->PR |= EXTI_PR_PR0;  // Clear interrupt flag
        cycle_game_mode();        // Cycle to the next game mode
    }
}

// Function to cycle through game modes and display the current mode
void cycle_game_mode() {
    current_mode = (current_mode + 1) % 3;  // Cycle: EASY -> MEDIUM -> HARD -> EASY
    display_game_mode(mode_strings[current_mode]);  // Update OLED display
}

// Function to display the game mode on OLED
void display_game_mode(const char *mode) {
    spi_cmd(0x02);  // Set cursor to the beginning of the first line
    while (*mode) {
        spi_data(*mode++);  // Send each character of mode string to display
    }
}

// Function to initialize internal clock (if required)
void internal_clock(void) {
    // Placeholder function for internal clock setup
}

// Initialize SPI1 for OLED
void init_spi1(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    GPIOA->MODER &= ~(0xC000FC00);             
    GPIOA->MODER |= (0x8000A800);              

    GPIOA->AFR[0] &= ~(0xF0F00000);            
    GPIOA->AFR[1] &= ~(0xF0000000);            

    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_BR | SPI_CR1_MSTR;
    SPI1->CR2 |= SPI_CR2_DS_3 | SPI_CR2_DS_0;  
    SPI1->CR2 &= ~(SPI_CR2_DS_1 | SPI_CR2_DS_2); 
    SPI1->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_SSOE | SPI_CR2_NSSP;
    SPI1->CR1 |= SPI_CR1_SPE;  // Enable SPI1
}

// Initialize the OLED display using SPI commands
void spi1_init_oled(void) {
    nano_wait(1000000);  // wait 1 ms
    spi_cmd(0x38); // Function set
    spi_cmd(0x08); // Display off
    spi_cmd(0x01); // Clear display
    nano_wait(2000000); // wait 2 ms
    spi_cmd(0x06); // Entry mode set
    spi_cmd(0x02); // Cursor to home position
    spi_cmd(0x0C); // Display on
}

// Send a command to the OLED
void spi_cmd(unsigned int data) {
    while (!(SPI1->SR & SPI_SR_TXE)) { }  // Wait until TX buffer is empty
    SPI1->DR = data;
}

// Send data to the OLED
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);  // Add 0x200 to mark data instead of command
}

// Short delay function
void nano_wait(unsigned int n) {
    asm("mov r0,%0\n repeat: sub r0,#83\n bgt repeat\n" : : "r"(n) : "r0", "cc");
}

// Main function
int main(void) {
    internal_clock();        // Initialize system clock
    enable_ports();          // Enable GPIO (placeholder function in your setup)
    setup_button_interrupt();// Initialize button for mode selection
    init_spi1();             // Initialize SPI1 for OLED communication
    spi1_init_oled();        // Initialize OLED display settings

    // Display the initial game mode on OLED
    display_game_mode(mode_strings[current_mode]);

    // Main loop (empty as the button interrupt will handle mode change)
    while (1) {
        // Do nothing; wait for button interrupts
    }
}
