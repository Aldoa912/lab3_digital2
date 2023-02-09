/*
 * File:   main.c
 * Author: aldoa
 *
 * Created on 2 de febrero de 2023, 10:19 PM
 */

//*****************************************************************************
//*****************************************************************************
// Palabra de configuración
//*****************************************************************************
// CONFIG1
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
#include "ADC.h"
//*****************************************************************************
// Definición de variables
//*****************************************************************************
#define _XTAL_FREQ 4000000

uint8_t flag = 0;
uint8_t contador = 0;
uint8_t bandera = 0;
//*****************************************************************************
// Definición de funciones para que se puedan colocar después del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*****************************************************************************
unsigned int ADC;

void setup(void);
//*****************************************************************************
// Código de Interrupción 
//*****************************************************************************
void __interrupt() isr(void){
   if(SSPIF == 1){
       if (flag == 0){
        ADC = ADC_Read(0); 
        PORTD = spiRead();
        spiWrite('s');
        __delay_ms(1);
        PORTD = spiRead();
        spiWrite(ADC);
       }
       if (flag == 1){
           PORTD = spiRead();
           spiWrite('n');
            __delay_ms(1);
           PORTD = spiRead();
           spiWrite(contador);
           flag = 0;
       }
       SSPIF = 0;
    }
   if (INTCONbits.RBIF){
        if (PORTBbits.RB0 == 0){
            contador++;
            flag = 1;
            
        }
        
   else if (PORTBbits.RB1 == 0){
            contador--;
            flag = 1;
        }
    INTCONbits.RBIF = 0;
   }
}
//*****************************************************************************
// Código Principal
//*****************************************************************************
void main(void) {
    setup();
    setupADC();
    //*************************************************************************
    // Loop infinito
    //*************************************************************************
    while(1){
      
    }
    return;
}
//*****************************************************************************
// Función de Inicialización
//*****************************************************************************
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
    
    TRISB = 0b00000011;
    TRISD = 0;
    
    PORTA = 0;
    PORTB = 0;
    PORTD = 0;
    
    OSCCONbits.IRCF = 0b110;        
    OSCCONbits.SCS = 1;   
    INTCONbits.GIE = 1;         // Habilitamos interrupciones
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones PEIE
    PIR1bits.SSPIF = 0;         // Borramos bandera interrupción MSSP
    PIE1bits.SSPIE = 1;         // Habilitamos interrupción MSSP
    TRISAbits.TRISA5 = 1;       // Slave Select
    TRISAbits.TRISA0 = 1;       // Slave Select
    OPTION_REGbits.nRBPU = 0;   // Se habilitan las resistencias pullup
    WPUBbits.WPUB0 = 1;        
    WPUBbits.WPUB1 = 1;
                                // Se asigna que puertos tendran resistencias
                                // pullup
    IOCB = 0b00000111;          // que pines contaran con interrupcion
    
//    INTCONbits.PEIE = 1;
    INTCONbits.RBIF = 0;        // se baja la bandera del puerto b
    INTCONbits.RBIE = 1;        // se activa la interrupcion del puerto b
    INTCONbits.GIE = 1;         // se activan las interrupciones globales
 
    spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);
   
}
