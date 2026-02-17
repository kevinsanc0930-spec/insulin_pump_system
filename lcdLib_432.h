#ifndef LCDLIB_432_H
#define LCDLIB_432_H

#include "msp.h"
#include <stdint.h>

/* -----------------------------------------------------------
   Pin mapping (4-bit mode, remapped middle bits):
   RS -> P4.5,  E  -> P4.4
   D4 -> P4.0,  D5 -> P6.4,  D6 -> P6.5,  D7 -> P4.3
   LCD R/W (pin 5) must be tied to GND in hardware.
   ----------------------------------------------------------- */

#define RS        BIT5   /* on P4 */
#define EN        BIT4   /* on P4 */

#define D4_P4     BIT0   /* P4.0 -> LCD D4 (pin 11) */
#define D7_P4     BIT3   /* P4.3 -> LCD D7 (pin 14) */
#define D5_P6     BIT4   /* P6.4 -> LCD D5 (pin 12) */
#define D6_P6     BIT5   /* P6.5 -> LCD D6 (pin 13) */

/* Public API expected by your lab code */
void lcdInit(void);
void lcdTriggerEN(void);
void lcdWriteData(unsigned char data);
void lcdWriteCmd(unsigned char cmd);
void lcdSetText(char* text, int x, int y);
void lcdSetInt(int val, int x, int y);
void lcdClear(void);

/* If your project already provides these, great.
   If not, you can implement them as busy-waits with __delay_cycles. */
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#endif /* LCDLIB_432_H */
