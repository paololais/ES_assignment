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
#include <string.h>

typedef enum {IDLE, S_dollar, S_R, S_A, S_T, S_E, S_comma, S_asterisk} UART_State;
UART_State uartState = IDLE;
int messageOk = 0; // flag to check if $RATE,xx* is correct
char receivedChar[2]; // store values for $RATE,xx*
int success = 0;

// circular buffer for TX
// used to send error messages, magnetometer data and yaw
CircularBuffer cb_tx;

// circular buffer for RX
// used to receive rate by user
CircularBuffer cb_rx;

int readFrequency(){
    if(cb_is_empty(&cb_rx)) return 0;
    
    for (int i = 0; i<2; i++){
        IEC0bits.U1RXIE = 0;   // disable interrupt RX
        cb_pop(&cb_rx, &receivedChar[i]);
        IEC0bits.U1RXIE = 1;   // enable interrupt RX
    }
    
    if(strcmp(receivedChar, "00") == 0 || strcmp(receivedChar, "01") == 0 || strcmp(receivedChar, "02") == 0 || 
            strcmp(receivedChar, "04") == 0 || strcmp(receivedChar, "05") == 0 || strcmp(receivedChar, "10") == 0){
        
        return 1;
    }
    
    else return 0;
}

// Funzione che elabora i caratteri dal buffer circolare
void processReceivedData() {
    char receivedChar;
    
    // Se ci sono caratteri nel buffer
    while (!cb_is_empty(&cb_rx)) {
        IEC0bits.U1RXIE = 0;   // disabilita interrupt RX
        // Pop del carattere dal buffer
        cb_pop(&cb_rx, &receivedChar);
        IEC0bits.U1RXIE = 1;   // Abilita interrupt RX
        
        handle_UART_FSM(receivedChar); // Gestisce il carattere in base alla FSM
    }
}

void handle_UART_FSM(char receivedChar) {
    switch (uartState) {
        case IDLE:
            if (receivedChar == '$') uartState = S_dollar;
            break;
        case S_dollar:
            if (receivedChar == 'R') uartState = S_R;
            else uartState = IDLE;
            break;
        case S_R:
            if (receivedChar == 'A') uartState = S_A;
            else uartState = IDLE;
            break;
            
        case S_A:
            if (receivedChar == 'T') uartState = S_T;
            else uartState = IDLE;
            break;
        
        case S_T:
            if (receivedChar == 'E') uartState = S_E;
            else uartState = IDLE;
            break; 
        case S_E:
            if (receivedChar == ',') uartState = S_comma;
            else uartState = IDLE;
            break;
        case S_comma:
            success = readFrequency();
            if(success) uartState = S_asterisk;
            else uartState = IDLE;
            break;
        case S_asterisk:
            if (receivedChar == '*') messageOk = 1;
            else messageOk = 0;
            
            uartState = IDLE;
            break;  
        
       
    }
}

// function that waits 7 ms
void algorithm() {
    tmr_wait_ms(TIMER2, 7);
}

int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000; // disable analog inputs
    
    //variables
    int ret;
    int i = 0;
    int missed_deadlines = 0; // variable to count missed deadlines of algorithm
    
    // led2
    TRISGbits.TRISG9 = 0; // LED2 output
    LATGbits.LATG9 = 0; // switch off LED2
    
    UART1_Init(); // initialize UART1
    
    cb_init(&cb_tx);
    cb_init(&cb_rx);
    
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