/*
 * File:   spi.c
 * Author: paolo
 *
 * Created on 31 marzo 2025, 10.29
 */

#include "xc.h"
#include "spi.h"

void spi_init() {
    SPI1STATbits.SPIEN = 0;    // Disabilita SPI per configurazione

    SPI1CON1bits.MSTEN = 1;    // Master mode
    SPI1CON1bits.MODE16 = 0;   // 8-bit mode
    SPI1CON1bits.CKE = 1;      // Trasmissione su transizione attivo?idle
    SPI1STATbits.SPIROV = 0;   // Clear overflow

    // Fcy = 72MHz ? F_SPI = Fcy / (Primary � Secondary)
    //TODO: calcolare quali valori settare
    SPI1CON1bits.PPRE = 0b00;  // Primary prescaler 64:1
    SPI1CON1bits.SPRE = 0b101; // Secondary prescaler 3:1
    
    // Remapping configuration
    TRISAbits.TRISA1 = 1; // RA1-RPI17 MISO
    TRISFbits.TRISF12 = 0; // RF12-RP108 SCK
    TRISFbits.TRISF13 = 0; // RF13-RP109 MOSI
    
    // configurare i 3 slave come output
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    TRISDbits.TRISD6 = 0;
    
    RPINR20bits.SDI1R = 0b0010001; // MISO (SDI1) - RPI17
    RPOR12bits.RP109R = 0b000101; // MOSI (SDO1) - RF13;
    RPOR11bits.RP108R = 0b000110; // SCK1; 
    
    ACC_CS = 1;
    GYR_CS = 1;
    MAG_CS = 1;

    SPI1STATbits.SPIEN = 1;    // Abilita SPI
}

unsigned int spi_write(unsigned int read_addr){
    unsigned int value;
    unsigned int trash;
    
    MAG_CS = 0;
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = read_addr | 0x80;
    while (SPI1STATbits.SPIRBF == 0);
    trash = SPI1BUF;
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x00;
    while (SPI1STATbits.SPIRBF == 0);
    value = SPI1BUF;
    MAG_CS = 1;
    
    if (SPI1STATbits.SPIROV == 1){
        SPI1STATbits.SPIROV = 0;
    }
    
    return value;
}

void spi_write_2_reg(unsigned int read_addr, unsigned int* value1, unsigned int* value2){
    unsigned int trash;

    MAG_CS = 0;

    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = read_addr | 0xC0;

    while (SPI1STATbits.SPIRBF == 0);
    trash = SPI1BUF; 

    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x00;

    while (SPI1STATbits.SPIRBF == 0);
    *value1 = SPI1BUF;

    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x00;

    while (SPI1STATbits.SPIRBF == 0);
    *value2 = SPI1BUF;

    MAG_CS = 1;

    //clear overflow
    if (SPI1STATbits.SPIROV == 1){
        SPI1STATbits.SPIROV = 0;
    }
}
/*Nel main:
 * unsigned char v1, v2;
spi_read_double(0x42, &v1, &v2);
// Ora v1 e v2 contengono i due byte letti
 */

void mag_enable(){
    unsigned int addr;
    unsigned int trash;
    
    // sleep mode
    MAG_CS = 0;
    addr = 0x004B;
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = addr & 0x7F;
    while (SPI1STATbits.SPIRBF == 0);
    trash = SPI1BUF;
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x01;
    while (SPI1STATbits.SPIRBF == 0);
    trash = SPI1BUF;
    MAG_CS = 1;
    
    tmr_wait_ms(1, 4); //tmr_wait_ms(1, 4); // wait 4ms for the magnetometer to switch to sleep mode 
    
    // active mode
    MAG_CS = 0;
    addr = 0x004C;
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = addr & 0x7F;
    while (SPI1STATbits.SPIRBF == 0);
    trash = SPI1BUF;
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x00;
    while (SPI1STATbits.SPIRBF == 0);
    trash = SPI1BUF;
    MAG_CS = 1;
    
    //clear overflow
    if (SPI1STATbits.SPIROV == 1){
        SPI1STATbits.SPIROV = 0;
    }
}