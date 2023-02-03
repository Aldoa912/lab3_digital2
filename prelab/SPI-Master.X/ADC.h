/* 
 * File:   ADC.h
 * Author: aldoa
 *
 * Created on 26 de enero de 2023, 08:34 PM
 */


#ifndef ADC_H
#define	ADC_H
#include <xc.h>

void setupADC(void);
unsigned int ADC_Read(unsigned int channel);


#endif	/* ADC_H */


