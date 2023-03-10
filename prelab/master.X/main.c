/*
 * File:   main.c
 * Author: aldoa
 *
 * Created on 2 de febrero de 2023, 10:53 PM
 */

#pragma config FOSC = EXTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//*****************************************************************************
// Definición e importación de librerías
//*****************************************************************************
#include <xc.h>
#include <stdint.h>
#include "SPI.h"
#include "LCD.h"
#include "ADC.h"
#include <stdio.h>
//*****************************************************************************
// Definición de variables
//*****************************************************************************
#define _XTAL_FREQ 4000000


#define RS RD2
#define EN RD3
#define D4 RD4
#define D5 RB5
#define D6 RD6
#define D7 RD7

uint8_t ADC;
uint8_t ADC1;
uint8_t contador;
char verificacion;
char valor [];
char centenas;
char decenas;
char unidad;
char centenas1;
char decenas1;
char unidad1;
char centenas2;
char decenas2;
char unidad2;
//*****************************************************************************
// Definición de funciones para que se puedan colocar después del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*****************************************************************************
void setup(void);

//*****************************************************************************
// Código Principal
//*****************************************************************************
void main(void) {
    setup();
    Lcd_Init();
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("ADC   ADC2   CONT");
    //*************************************************************************
    // Loop infinito
    //*************************************************************************
    while(1){
        PORTCbits.RC2 = 0;       //Slave Select
        __delay_ms(1);
       
       spiWrite(PORTD);
       ADC = spiRead();
       
        __delay_ms(1);
        PORTCbits.RC2 = 1;       //Slave Deselect 
        
        centenas = (ADC/100);
        decenas = (ADC/10)%10;
        unidad = ADC%10;

        Lcd_Set_Cursor(2,1);
        Lcd_Write_Char(centenas + 48);
        Lcd_Write_Char(decenas + 48);
        Lcd_Write_Char(unidad + 48);
        
        
        PORTCbits.RC1 = 0;       //Slave Select
        __delay_ms(1);
       
       spiWrite(PORTD);
       verificacion = spiRead();
       
       if (verificacion == 's'){
           spiWrite(PORTD);
           ADC1 = spiRead();
       }
       
       else if (verificacion == 'n'){
           spiWrite(PORTD);
           contador = spiRead();
       }
       
        __delay_ms(1);
        PORTCbits.RC1 = 1;       //Slave Deselect 
        
        PORTB = ADC;
        //sprintf(valor, "%u", ADC);

        
        centenas1 = (ADC1/100);
        decenas1 = (ADC1/10)%10;
        unidad1 = ADC1%10;

        Lcd_Set_Cursor(2,7);
        Lcd_Write_Char(centenas1 + 48);
        Lcd_Write_Char(decenas1 + 48);
        Lcd_Write_Char(unidad1 + 48);
        
        centenas2 = (contador/100);
        decenas2 = (contador/10)%10;
        unidad2 = contador%10;

        Lcd_Set_Cursor(2,14);
        Lcd_Write_Char(centenas2 + 48);
        Lcd_Write_Char(decenas2 + 48);
        Lcd_Write_Char(unidad2 + 48);
    }
    return;
}
//*****************************************************************************
// Función de Inicialización
//*****************************************************************************
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
    TRISB = 0;
    TRISD = 0;
    PORTB = 0;
    PORTD = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC1 = 0;
    PORTCbits.RC2 = 1;
    OSCCONbits.IRCF = 0b110;        
    OSCCONbits.SCS = 1;    
    spiInit(SPI_MASTER_OSC_DIV4, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);

}