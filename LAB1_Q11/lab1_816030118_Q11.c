/*
 * @File:   lab1.c
 * Author: Amir Ali 
 * ID: 816030118
 *
 * @brief LED blinking code 1Hz for 8MHz Osc
 * 
 */

// PIC18F4620 Configuration Bit Settings
// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)


#include <xc.h>
#include <stdio.h>
#include <time.h>

#define _XTAL_FREQ 8000000

unsigned long tick = 0;

char y = 'A';

void putch(char c)
{
    while(!TXIF)
        continue;
    TXREG = c;
}

//int getch(void)
//{
//    while(!PIR1bits.RCIF);
//	return RCREG;
//}

void USARTsetup(void)
{
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    RCSTAbits.SPEN = 1;
    RCSTA = 0b1001000;
    TXSTAbits.BRGH = 1;
    TXSTAbits.SYNC = 0;
    TXSTAbits.TXEN = 1;
    SPBRGH = 0x00;
    SPBRG = 0b00011001;
    BAUDCON = 0b00000000;
    INTCONbits.GIE = 0;
    PIE1bits.RCIE = 1;
    PIR1bits.RCIF = 0;
}

void timer0setup(void)
{
    T0CON = 0b00000100;
    INTCONbits.GIE = 1;
}

//void __interrupt(high_priority) tcint(void)
//{
//    //static int tick;
//    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF)
//    {
//        INTCONbits.TMR0IE = 0;
//        INTCONbits.TMR0IF = 0;
//        tick++;
//        printf("tick = %d", tick);
//    }
//    
//    return;
//}


void __interrupt(high_priority) tcint(void)
{
    if (PIR1bits.RCIF &&PIE1bits.RCIE)
    {
        PIR1bits.RCIF = 0;
        y = RCREG;
    }
    return;
}

void timer2setup(void)
{
    T1CON = 0b10111001;
    INTCONbits.GIE = 1;
    PIE1bits.TMR1IE = 1;
}

//void __interrupt(high_priority) tcint(void)
//{
//    if (PIR1bits.TMR1IF && PIE1bits.TMR1IE)
//    {
//        PIR1bits.TMR2IF = 0;
//        tick += 1;
//        printf("tick = %ld", tick);
//    }
//    if (tick == 25)
//    {
//        tick = 0;
//    }
//    return;
//}

void sleep(int sec)
{
    
    
    for (int i =0; i < sec; i++)
    {
        T0CONbits.TMR0ON = 1;
        INTCONbits.T0IE = 1;
        while(INTCONbits.TMR0IE == 1)
            continue;
        T0CONbits.TMR0ON = 0;
        //printf("tick = %d", tick);
        
    }
    
    
}
 
void main(void) 
{
    //timer0setup();
    USARTsetup();
    TRISDbits.TRISD2 = 0;
    LATDbits.LATD2 = 0;
    while(1)
    {
        //y = getchar();
        printf("%c",y);
        printf("Y");
        LATDbits.LATD2 = LATDbits.LATD2 ^ 1;   //Invert LATD bit 2 
        //printf("Hello World!\n");
        //if (PIR1bits.TXIF == 0)
        //{
        //  TXREG = 0x54;  
        //}
        __delay_ms(500);
        //sleep(5);
    }
    
    return;
}

/***End of File***/