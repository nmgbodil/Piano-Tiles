#include "stm32f0xx.h"
#include <string.h> // for memmove()
#include <stdlib.h> // for srandom() and random()
#include <stdio.h>

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

const char font[] = {
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, // 32: space
    0x86, // 33: exclamation
    0x22, // 34: double quote
    0x76, // 35: octothorpe
    0x00, // dollar
    0x00, // percent
    0x00, // ampersand
    0x20, // 39: single quote
    0x39, // 40: open paren
    0x0f, // 41: close paren
    0x49, // 42: asterisk
    0x00, // plus
    0x10, // 44: comma
    0x40, // 45: minus
    0x80, // 46: period
    0x00, // slash
    // digits
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x67,
    // seven unknown
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    // Uppercase
    0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x6f, 0x76, 0x30, 0x1e, 0x00, 0x38, 0x00,
    0x37, 0x3f, 0x73, 0x7b, 0x31, 0x6d, 0x78, 0x3e, 0x00, 0x00, 0x00, 0x6e, 0x00,
    0x39, // 91: open square bracket
    0x00, // backslash
    0x0f, // 93: close square bracket
    0x00, // circumflex
    0x08, // 95: underscore
    0x20, // 96: backquote
    // Lowercase
    0x5f, 0x7c, 0x58, 0x5e, 0x79, 0x71, 0x6f, 0x74, 0x10, 0x0e, 0x00, 0x30, 0x00,
    0x54, 0x5c, 0x73, 0x7b, 0x50, 0x6d, 0x78, 0x1c, 0x00, 0x00, 0x00, 0x6e, 0x00
};

extern uint16_t msg[8];

void set_digit_segments(int digit, char val) {
    msg[digit] = (digit << 8) | val;
}

void print(const char str[])
{
    const char *p = str;
    for(int i=0; i<8; i++) {
        if (*p == '\0') {
            msg[i] = (i<<8);
        } else {
            msg[i] = (i<<8) | font[*p & 0x7f] | (*p & 0x80);
            p++;
        }
    }
}

void printfloat(float f)
{
    char buf[10];
    snprintf(buf, 10, "%f", f);
    for(int i=1; i<10; i++) {
        if (buf[i] == '.') {
            // Combine the decimal point into the previous digit.
            buf[i-1] |= 0x80;
            memcpy(&buf[i], &buf[i+1], 10-i-1);
        }
    }
    print(buf);
}

void append_segments(char val) {
    for (int i = 0; i < 7; i++) {
        set_digit_segments(i, msg[i+1] & 0xff);
    }
    set_digit_segments(7, val);
}

void clear_display(void) {
    for (int i = 0; i < 8; i++) {
        msg[i] = msg[i] & 0xff00;
    }
}

// 16 history bytes.  Each byte represents the last 8 samples of a button.
uint8_t hist[16];
#define QUEUE_SIZE 16
char queue[QUEUE_SIZE];
int qin = 0;
int qout = 0;

const char keymap[] = "DCBA#9630852*741";

void push_queue(int n) {
    int next = (qin + 1) % QUEUE_SIZE;
    if (next != qout) {  // Check if queue isn't full
        queue[qin] = n;
        qin = next;
    }
}

char pop_queue() {
    if (qin == qout) return 0;  // Empty queue
    char val = queue[qout];
    qout = (qout + 1) % QUEUE_SIZE;
    return val;
}

char get_key_event(void) {
    return pop_queue();
}

void update_history(int c, int rows) {
    for(int i = 0; i < 4; i++) {
        hist[4*c+i] = (hist[4*c+i]<<1) + ((rows>>i)&1);
        if (hist[4*c+i] == 0x01)
            push_queue(0x80 | keymap[4*c+i]);  // Key press only
    }
}

void drive_column(int c)
{
    GPIOC->BSRR = 0xf00000 | ~(1 << (c + 4));
}

int read_rows()
{
    return (~GPIOC->IDR) & 0xf;
}

char get_keypress() {
    char event;
    for(;;) {
        // Wait for every button event...
        event = get_key_event();
        // ...but ignore if it's a release.
        if (event & 0x80)
            break;
    }
    return event & 0x7f;
}

void show_keys(void)
{
    char buf[] = "        ";
    for(;;) {
        char event = get_key_event();
        memmove(buf, &buf[1], 7);
        buf[7] = event;
        print(buf);
    }
}

// Turn on the dot of the rightmost display element.
void dot()
{
    msg[7] |= 0x80;
}

extern uint16_t display[34];
// void spi1_dma_display1(const char *str)
// {
//     for(int i=0; i<16; i++) {
//         if (str[i])
//             display[i+1] = 0x200 + str[i];
//         else {
//             // End of string.  Pad with spaces.
//             for(int j=i; j<16; j++)
//                 display[j+1] = 0x200 + ' ';
//             break;
//         }
//     }
// }

void spi2_dma_display1(const char *str) {
    for (int i = 0; i < 16; i++) {
        if (str[i]) {
            display[i + 1] = 0x200 + str[i];
        } else {
            // End of string. Pad with spaces.
            for (int j = i; j < 16; j++) {
                display[j + 1] = 0x200 + ' ';
            }
            break;
        }
    }
    // No need to reconfigure DMA; SPI2 DMA is already set up in spi2_setup_dma().
}


// void spi1_dma_display2(const char *str)
// {
//     for(int i=0; i<16; i++) {
//         if (str[i])
//             display[i+18] = 0x200 + str[i];
//         else {
//             // End of string.  Pad with spaces.
//             for(int j=i; j<16; j++)
//                 display[j+18] = 0x200 + ' ';
//             break;
//         }
//     }
// }

void spi2_dma_display2(const char *str) {
    for (int i = 0; i < 16; i++) {
        if (str[i]) {
            display[i + 18] = 0x200 + str[i];
        } else {
            // End of string. Pad with spaces.
            for (int j = i; j < 16; j++) {
                display[j + 18] = 0x200 + ' ';
            }
            break;
        }
    }
    // No need to reconfigure DMA; SPI2 DMA is already set up in spi2_setup_dma().
}


// int score = 0;
char disp1[17] = "                ";
char disp2[17] = "                ";
char score_str[6];
volatile int pos = 0;
// void TIM17_IRQHandler(void)
// {
//     TIM17->SR &= ~TIM_SR_UIF;
//     memmove(disp1, &disp1[1], 16);
//     memmove(disp2, &disp2[1], 16);
//     if (pos == 0) {
//         if (disp1[0] != ' ')
//             score -= 1;
//         if (disp2[0] != ' ')
//             score += 1;
//         disp1[0] = '>';
//     } else {
//         if (disp2[0] != ' ')
//             score -= 1;
//         if (disp1[0] != ' ')
//             score += 1;
//         disp2[0] = '>';
//     }
//     int create = random() & 3;
//     if (create == 0) { // one in four chance
//         int line = random() & 1;
//         if (line == 0) { // pick a line
//             disp1[15] = 'x';
//             disp2[15] = ' ';
//         } else {
//             disp1[15] = ' ';
//             disp2[15] = 'x';
//         }
//     } else {
//         disp1[15] = ' ';
//         disp2[15] = ' ';
//     }
//     if (pos == 0)
//         disp1[0] = '>';
//     else
//         disp2[0] = '>';
//     if (score >= 100) {
//         print("Score100");
//         spi1_dma_display1("Game over");
//         spi1_dma_display2("You win");
//         NVIC->ICER[0] = 1<<TIM17_IRQn;
//         return;
//     }
//     char buf[9];
//     snprintf(buf, 9, "Score% 3d", score);
//     print(buf);
//     spi1_dma_display1(disp1);
//     spi1_dma_display2(disp2);
//     TIM17->ARR = 250 - 1 - 2*score;
// }

void init_tim17(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    TIM17->PSC = 48000 - 1;
    TIM17->ARR = 250 - 1;
    TIM17->CR1 |= TIM_CR1_ARPE;
    TIM17->DIER |= TIM_DIER_UIE;
    TIM17->CR1 |= TIM_CR1_CEN;
}

void init_spi2(void);
void spi2_setup_dma(void);
void spi2_enable_dma(void);
void init_spi1(void);
void spi1_init_oled(void);
void spi1_setup_dma(void);
void spi1_enable_dma(void);
int increment_score(void);

// void game(void)
// {
//     // char* disp1_ = malloc(sizeof(char) * 17);
//     // char* disp2_ = malloc(sizeof(char) * 17);
//     // print("Score  0");
//     // init_spi2();
//     // spi2_setup_dma();
//     // spi2_enable_dma();
//     spi1_dma_display1("Hello!");
//     spi1_dma_display2("Pick Mode: A B C");
//     init_spi1();
//     spi1_init_oled();
//     spi1_setup_dma();
//     spi1_enable_dma();
//     // init_tim17(); // start timer
//     // get_keypress(); // Wait for key to start
//     // spi1_dma_display1(">               ");
//     // spi1_dma_display2("                ");
//     // Use the timer counter as random seed...
//     // srandom(TIM17->CNT);
//     // Then enable interrupt...
//     // NVIC->ISER[0] = 1<<TIM17_IRQn;
//     int start = 1;
//     for(;;) {
//         char key = get_keypress();
//         if (start && (key == 'A' || key == 'B' || key == 'C')) {
//             // If the A or B key is pressed, disable interrupts while
//             // we update the display.
//             // asm("cpsid i");
//             // pos = 0;
//             if (key == 'A') {
//                 spi1_dma_display1("Mode: Easy");
//             }
//             else if (key == 'B') {
//                 spi1_dma_display1("Mode: Medium");
//             }
//             else {
//                 spi1_dma_display1("Mode: Hard");
//             }
//             spi1_dma_display2("Score: 000");
//             start = 0;

//             // asm("cpsie i");
//         }
//         else if (!start){
//             score += increment_score();
//             snprintf(disp2, 12, "Score: %03d", score);
//             spi1_dma_display2(disp2);
//         }

//     }
// }

int increment_score(void) {
    return rand() % 2;
}
