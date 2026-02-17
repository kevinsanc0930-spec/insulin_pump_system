#include "lcdLib_432.h"
#include <stdio.h>

/* ---------- Minimal delay helpers (safe if your project lacks them) ---------- */
#ifndef LCDLIB_HAS_DELAY_FUNCS
void delay_us(uint32_t us){ while(us--) __delay_cycles(6); }     /* ~2 us @ ~3 MHz DCO */
void delay_ms(uint32_t ms){ while(ms--) __delay_cycles(6000); }  /* ~1 ms */
#endif

/* ---------- Write the low nibble to split ports: P4.0, P6.4, P6.5, P4.3 ---------- */
/* bit0 -> D4(P4.0), bit1 -> D5(P6.4), bit2 -> D6(P6.5), bit3 -> D7(P4.3) */
#define LOWNIB(x) do {                                  \
    P4->OUT &= ~(D4_P4 | D7_P4);                         \
    P6->OUT &= ~(D5_P6 | D6_P6);                         \
    if ((x) & 0x1) P4->OUT |= D4_P4;   /* D4 */          \
    if ((x) & 0x2) P6->OUT |= D5_P6;   /* D5 (P6.4) */   \
    if ((x) & 0x4) P6->OUT |= D6_P6;   /* D6 (P6.5) */   \
    if ((x) & 0x8) P4->OUT |= D7_P4;   /* D7 */          \
} while(0)

/* ---------- EN pulse on P4.4 ---------- */
void lcdTriggerEN(void) {
    P4->OUT |= EN;
    /* E high width; spec > 230 ns, we’re very safe */
    __delay_cycles(12); /* ~4 us at ~3 MHz */
    P4->OUT &= ~EN;
    __delay_cycles(12);
}

/* ---------- Init ---------- */
void lcdInit(void) {
    delay_ms(100);                 /* >=40 ms after power */

    /* Set pins to GPIO outputs and low */
    /* Control: RS, EN on P4 */
    P4->DIR  |= (RS | EN | D4_P4 | D7_P4);
    P4->SEL0 &= ~(RS | EN | D4_P4 | D7_P4);
    P4->SEL1 &= ~(RS | EN | D4_P4 | D7_P4);
    P4->OUT  &= ~(RS | EN | D4_P4 | D7_P4);

    /* Data middle bits on P6 */
    P6->DIR  |= (D5_P6 | D6_P6);
    P6->SEL0 &= ~(D5_P6 | D6_P6);
    P6->SEL1 &= ~(D5_P6 | D6_P6);
    P6->OUT  &= ~(D5_P6 | D6_P6);

    /* Wake-up knocks in 8-bit style (send 0x3 on the high nibble path) */
    P4->OUT &= ~RS;                /* RS = 0 (command) */

    LOWNIB(0x03); lcdTriggerEN();  delay_ms(5);
    LOWNIB(0x03); lcdTriggerEN();  delay_ms(5);
    LOWNIB(0x03); lcdTriggerEN();  delay_ms(5);

    /* Switch to 4-bit (send 0x2) */
    LOWNIB(0x02); lcdTriggerEN();  delay_ms(5);

    /* Function set, display off, clear, entry mode, display on */
    lcdWriteCmd(0x28);   /* 4-bit, 2 line, 5x8 */
    lcdWriteCmd(0x08);   /* display off */
    lcdWriteCmd(0x01);   /* clear */
    lcdWriteCmd(0x06);   /* entry inc, no shift */
    lcdWriteCmd(0x0C);   /* display on, cursor off, blink off */
}

/* ---------- Write a data byte ---------- */
void lcdWriteData(unsigned char data) {
    P4->OUT |= RS;                 /* RS = 1 (data) */
    LOWNIB(data >> 4); lcdTriggerEN();
    LOWNIB(data     ); lcdTriggerEN();
    delay_us(50);                  /* > 47 us per datasheet */
}

/* ---------- Write a command byte ---------- */
void lcdWriteCmd(unsigned char cmd) {
    P4->OUT &= ~RS;                /* RS = 0 (command) */
    LOWNIB(cmd >> 4); lcdTriggerEN();
    LOWNIB(cmd     ); lcdTriggerEN();
    delay_ms(2);                   /* safe for most cmds; clear/home need >1.5 ms */
}

/* ---------- Position + print helpers ---------- */
void lcdSetText(char* text, int x, int y) {
    int i = 0;

    if (x < 16) {
        unsigned char addr = (unsigned char)x | 0x80;  /* row 0 base */
        switch (y) {
            case 1: addr = (unsigned char)x | (0x80 | 0x40); break; /* row 1 base 0x40 */
            case 2: addr = (unsigned char)x | (0x80 | 0x60); break; /* “reverse” row 0 */
            case 3: addr = (unsigned char)x | (0x80 | 0x20); break; /* “reverse” row 1 */
            default: break;
        }
        lcdWriteCmd(addr);
    }

    while (text[i] != '\0') {
        lcdWriteData((unsigned char)text[i++]);
    }
}

void lcdSetInt(int val, int x, int y) {
    char number_string[16];
    /* If you prefer no stdio, replace with manual itoa; stdio is fine for lab */
    sprintf(number_string, "%d", val);
    lcdSetText(number_string, x, y);
}

void lcdClear(void) {
    /* Clear display (takes ~1.52 ms); our lcdWriteCmd delay covers it */
    lcdWriteCmd(0x01);
}
