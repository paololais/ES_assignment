/*
 * File:   main.c
 * Author: paolo
 *
 * Created on April 18, 2025, 10:57 AM
 */

#include "xc.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"

// function that waits 7 ms
void algorithm() {
    tmr_wait_ms(TIMER2, 7);
}

int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000; // disable analog inputs
    
    int ret;
    int i = 0;
    int missed_deadlines = 0; // variable to count missed deadlines of algorithm
    
    TRISGbits.TRISG9 = 0; // LED2 output
    LATGbits.LATG9 = 0; // switch off LED2
    
    tmr_setup_period(TIMER1, 10); // Timer 1 for algorithm() - 100 Hz = 10ms
    while(1){
        algorithm();
        
         //blink led2
        i++;
        // after 50 ticks (500ms) blink LED2
        if (i == 50) {
            i = 0;
            LATGbits.LATG9 = !LATGbits.LATG9; // blink LED2
        }
        
        ret = tmr_wait_period(TIMER1);
        
        if(ret) missed_deadlines++;
    }
    return 0;
}