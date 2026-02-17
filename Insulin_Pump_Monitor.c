#include "msp.h"
#include "lcdLib_432.h"
#include <stdio.h>
#include <stdint.h>

#define LED BIT4        // P2.4
#define BUTTON BIT0     // P3.0
#define ADC_PIN 5       // P5.5

#define UART_TX BIT3    // P3.3
#define UART_RX BIT2    // P3.2

volatile uint16_t adc_raw = 0;
volatile float glucose = 0;
volatile uint8_t systemOn = 0;

void initGPIO(void);
void initADC(void);
void initUART(void);
void initSysTick(void);
void measureGlucose(void);
float convertToGlucose(uint16_t raw);
void transmitSignal(void);
void delay_ms_simple(int ms);
void transmitHigh(void);

int main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // Stop watchdog

    lcdInit();
    lcdClear();
    lcdSetText("MONITOR READY", 0, 0);

    initGPIO();
    initADC();
    initUART();

    __enable_irq();

    while(1){
        __sleep(); // sleep until interrupt
    }
}

// GPIO initialization
void initGPIO(void){
    // LED
    P2->DIR |= LED;
    P2->OUT &= ~LED;

    // Button P3.0
    P3->DIR &= ~BUTTON;
    P3->REN |= BUTTON;
    P3->OUT |= BUTTON;        // Pull-up
    P3->IES |= BUTTON;        // High-to-low edge
    P3->IFG &= ~BUTTON;
    P3->IE  |= BUTTON;

    NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31);
}

// ADC initialization
void initADC(void){
    P5->SEL0 |= BIT5;
    P5->SEL1 |= BIT5;

    ADC14->CTL0 = 0x00000010;
    ADC14->CTL0 |= ADC14_CTL0_SHP;
    ADC14->CTL1 = ADC14_CTL1_RES_3; // 14-bit
    ADC14->MCTL[0] = 5; // A5
    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

// UART initialization
void initUART(void){
    P3->SEL0 |= UART_TX | UART_RX;
    P3->SEL1 &= ~(UART_TX | UART_RX);

    EUSCI_A2->CTLW0 = 1;          // Reset
    EUSCI_A2->CTLW0 = 0x0081;     // SMCLK
    EUSCI_A2->BRW = 19;           // 3MHz / (16*9600)
    EUSCI_A2->MCTLW = (9<<4)|1;
    EUSCI_A2->CTLW0 &= ~1;        // Enable
}

// SysTick initialization
void initSysTick(void){
    SysTick->LOAD = 15000000-1; // 5 sec at 3 MHz
    SysTick->VAL = 0;
    SysTick->CTRL = 0x07;        // Enable, interrupt, use system clock
}

// Button ISR P3.0
void PORT3_IRQHandler(void){
    if(P3->IFG & BUTTON){
        P3->IFG &= ~BUTTON;

        if(!systemOn){
            systemOn = 1;
            P2->OUT |= LED;       // Turn on IR LED
            lcdClear();
            lcdSetText("MONITOR ON", 0, 0);

            initSysTick(); // Start 5-sec measurement
        }
    }
}

// SysTick ISR
void SysTick_Handler(void){
    if(systemOn){
        measureGlucose();
    }
}

// Measure glucose function
void measureGlucose(void){
    ADC14->CTL0 |= ADC14_CTL0_SC;
    while(!(ADC14->IFGR0 & 1));
    adc_raw = ADC14->MEM[0];

    //if(adc_raw > 4100) adc_raw = 4100;
       // if(adc_raw < 3400) adc_raw = 3400;

    glucose = convertToGlucose(adc_raw);

    // Display on LCD
    char buf[17];
    char glu[17];
    lcdClear();
    sprintf(buf,"ADC:%u", adc_raw);
    sprintf(glu,"G: %.0f",glucose);
    lcdSetText(buf,0,0);
    lcdSetText(glu, 10,0);



    if(glucose > 180.0f){
        transmitSignal();
        lcdSetText("TX SIGNAL",0,1);
        transmitHigh();
    }
}

// Convert ADC raw to glucose (linear mapping using your measured range)
float convertToGlucose(uint16_t raw){
    // Clamp values
    if(raw > 4100) raw = 4100;
    if(raw < 3400) raw = 3400;

    // Linear mapping: 4100 -> 80 mg/dL, 3400 -> 220 mg/dL
    float mgdl = 80.0f + ((4100.0f - (float)raw) / (4100.0f - 3400.0f)) * (220.0f - 80.0f);
    return mgdl;
}

// Transmit signal function
void transmitSignal(void){
    char c = 'I'; // insulin needed
    while(EUSCI_A2->STATW & 0x01); // wait TX ready
    EUSCI_A2->TXBUF = c;
}
void transmitHigh(void){
    char h = 'H';
    while(EUSCI_A2->STATW & 0x01); // wait TX ready
    EUSCI_A2->TXBUF = h;
}

// Simple delay
void delay_ms_simple(int ms){
    int i;
    for(i=0;i<ms;i++){
        __delay_cycles(3000);
    }
}
