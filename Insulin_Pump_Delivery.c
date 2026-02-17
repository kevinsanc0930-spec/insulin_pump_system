#include "msp.h"
#include "lcdLib_432.h"
#include <stdio.h>

/*Starting insulin reservoir in ml*/
float remainingInsulin = 1.0f;

/*Function prototypes*/
void servoNeutral(void);
void servoDeliver(void);
void delay_ms_simple(int ms);
void beepTriple(void);

void main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    /*P2.3 insulin indicator LED*/
    P2->DIR |= BIT3;
    P2->OUT &= ~BIT3;

    /*P1.1 refill button (negative logic)*/
    P1->DIR &= ~BIT1;      /*input*/
    P1->REN |= BIT1;       /*enable resistor*/
    P1->OUT |= BIT1;       /*pull-up*/
    P1->IES |= BIT1;       /*high->low edge*/
    P1->IFG &= ~BIT1;      /*clear flag*/
    P1->IE  |= BIT1;       /*enable interrupt*/

    /*Alarm LED on P2.6 (only for alarms, not for normal dose)*/
    P2->DIR |= BIT6;
    P2->OUT &= ~BIT6;

    /*Buzzer on P2.7*/
    P2->DIR |= BIT7;
    P2->OUT &= ~BIT7;

    /*Servo PWM on P2.4 (TA0.1)*/
    P2->DIR |= BIT4;
    P2->SEL0 |= BIT4;
    P2->SEL1 &= ~BIT4;

    TIMER_A0->CCR[0]  = 60000;    /*20ms period at 3MHz*/
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;   /*reset/set*/
    TIMER_A0->CCR[1]  = 4500;     /*1.5ms neutral*/
    TIMER_A0->CTL     = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;   /*SMCLK, up mode*/

    /*UART A2 for monitor with Ben (P3.2 RX, P3.3 TX)*/
    P3->SEL0 |= BIT2 | BIT3;
    P3->SEL1 &= ~(BIT2 | BIT3);

    EUSCI_A2->CTLW0 |= 1;              /*reset*/
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_A_CTLW0_SSEL__SMCLK;      /*SMCLK*/
    EUSCI_A2->BRW = 19;                /*3MHz / (16*9600) 19*/
    EUSCI_A2->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16;    /*Oversampling*/
    EUSCI_A2->CTLW0 &= ~1;             /*Enable */
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;                 /*RX interrupt*/

    /*Enable UART and Port 1 interrupts in NVIC*/
    NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);
    NVIC->ISER[1] = 1 << ((PORT1_IRQn)  & 31);

    __enable_irq();

    lcdInit();
    lcdClear();
    lcdSetText("PUMP READY", 0, 0);

    while (1) {
        __sleep();  /*sleep until interrupt*/
    }
}

/*UART A2 ISR*/
void EUSCIA2_IRQHandler(void) {
    char c = EUSCI_A2->RXBUF;
    char line[17];

    /*Insulin dose command: 'I'*/
    if (c == 'I') {
        if (remainingInsulin >= 0.5f) {

            remainingInsulin -= 0.5f;

            lcdClear();
            sprintf(line, "Dose: 0.5ml");
            lcdSetText(line, 0, 0);
            sprintf(line, "Left: %.1fml", remainingInsulin);
            lcdSetText(line, 0, 1);

            /*Visual indicator ONLY on P5.0*/
            P2->OUT |= BIT3;   /*insulin LED on*/

            /*Servo movement*/
            servoDeliver();
            delay_ms_simple(2000);
            servoNeutral();

            /*Turn off insulin LED*/
            P2->OUT &= ~BIT3;

            /*Return to ready automatically after successful dose*/
            delay_ms_simple(1500);
            lcdClear();
            lcdSetText("PUMP READY", 0, 0);
        }
        else {
            /*Out of insulin alarm – ONLY P1.1 can refill*/
            remainingInsulin = 0.0f;

            lcdClear();
            lcdSetText("PUMP EMPTY!", 0, 0);
            lcdSetText("REFILL: P1.1", 0, 1);

            P2->OUT |= BIT6;   /*alarm LED ON (empty is an alarm)*/
            beepTriple();      /*3 beeps, KEEP this screen + LED*/
        }
    }

    /*High glucose warning: 'H'*/
    if (c == 'H') {
        lcdClear();
        lcdSetText("GLUCOSE HIGH!", 0, 0);
        lcdSetText("220+ mg/dL", 0, 1);

        P2->OUT |= BIT6;       /*alarm LED ON*/
        beepTriple();          /*3 beeps, KEEP this screen on*/

    }

    /*Normal: 'N'*/
    if (c == 'N') {
        /*Clear alarms and go to ready*/
        P2->OUT &= ~BIT6;   /*alarm LED off*/
        P2->OUT &= ~BIT3;   /*insulin LED off*/
        P2->OUT &= ~BIT7;   /*buzzer off*/

        lcdClear();
        lcdSetText("PUMP READY", 0, 0);
    }
}

/*Port 1 ISR (P1.1 refill button)*/
void PORT1_IRQHandler(void) {
    char line[17];

    if (P1->IFG & BIT1) {
        P1->IFG &= ~BIT1;   /*clear flag*/

        /*Refill reservoir ONLY from here*/
        remainingInsulin = 10.0f;

        /*Turn off alarms*/
        P2->OUT &= ~BIT6;
        P2->OUT &= ~BIT3;
        P2->OUT &= ~BIT7;

        lcdClear();
        lcdSetText("PUMP REFILLED", 0, 0);
        sprintf(line, "Left: %.1fml", remainingInsulin);
        lcdSetText(line, 0, 1);

        delay_ms_simple(1500);
        lcdClear();
        lcdSetText("PUMP READY", 0, 0);
    }
}

/*Servo Helpers*/
void servoNeutral(void) {
    int pos;

    for (pos = 6000; pos >= 4500; pos -= 50) {
        TIMER_A0->CCR[1] = pos;
        delay_ms_simple(20);
    }
}


void servoDeliver(void) {
    int pos;

    /*start from neutral (4500) and move slowly to 6000*/
    for (pos = 4500; pos <= 6000; pos += 50) {
        TIMER_A0->CCR[1] = pos;
        delay_ms_simple(20);   /*20 ms between steps*/
    }
}


/*Simple delay*/
void delay_ms_simple(int ms) {
    int i;
    for (i = 0; i < ms; i++) {
        __delay_cycles(3000);  /*1 ms at 3 MHz*/
    }
}

/*Triple beep alarm*/
void beepTriple(void) {
    int j, i;

    for (j = 0; j < 3; j++) {
        /*short beep*/
        for (i = 0; i < 3000; i++) {
            P2->OUT ^= BIT7;       /*toggle buzzer pin*/
            __delay_cycles(800);
        }

        P2->OUT &= ~BIT7;          /*ensure off*/
        delay_ms_simple(200);      /*pause between beeps*/
    }
}
