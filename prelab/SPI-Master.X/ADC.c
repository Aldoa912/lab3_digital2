#include "ADC.h"
#include  <xc.h>
#define _XTAL_FREQ 4000000

unsigned int ADC_Read(unsigned int channel) {
    ADCON0bits.CHS = channel; // Select the input channel
    ADCON0bits.GO = 1;       // Start the conversion
    while(ADCON0bits.GO);    // Wait for the conversion to complete
    __delay_us(100);
    return ((ADRESH << 8)+ ADRESL); // Return the result
   
}

void setupADC(void){
    ANSEL = ANSEL | 0x01;
    ADCON0 = 0x00;  // Clear the register
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 0;       // Fosc/ 8
    ADCON1bits.VCFG1 = 0;       // Ref VSS
    ADCON1bits.VCFG0 = 0;       // Ref VDD
    
    ADCON1bits.ADFM = 1;        // Justificado hacia derecha
    ADCON0bits.ADON = 1;        // Habilitamos el ADC
    __delay_us(100);
}
