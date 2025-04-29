/*
 * File:   uart.c
 * Author: paolo
 *
 * Created on 1 aprile 2025, 9.44
 */


#include "uart.h"

// Inizializzazione UART1
void UART1_Init(void) {
    TRISDbits.TRISD11 = 1; // set RD11 as input (U1RX)
    TRISDbits.TRISD0 = 0;  // set RD0 as output (U1TX)
    
    RPINR18bits.U1RXR = 75; // RD11 mapped on U1RX
    RPOR0bits.RP64R = 1;    // RD0 mapped on U1TX
    
    U1BRG = BRGVAL; // baudrate setting
    
    U1STAbits.UTXISEL0 = 0; // Interrupt after one TX Character is transmitted
    U1STAbits.UTXISEL1 = 0;
    
    U1MODEbits.UARTEN = 1; // enbale UART1
    U1STAbits.UTXEN = 1; // enable TX
    IEC0bits.U1RXIE = 1;   // enable RX interrupt
}

//circular buffer
void cb_init(CircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

void cb_push(CircularBuffer *cb, char value) {
    if (cb->count == BUFFER_SIZE){
         // Buffer full: overwrite oldest
        cb->tail = (cb->tail + 1) % BUFFER_SIZE;
        cb->count--;
    }
    cb->buffer[cb->head] = value; // write the value
    cb->head = (cb->head + 1) % BUFFER_SIZE; // increment circularly
    cb->count++;
}

void cb_pop(CircularBuffer *cb, char *value) {
    *value = cb->buffer[cb->tail]; // read the value
    cb->tail = (cb->tail + 1) % BUFFER_SIZE; // increment circularly
    cb->count--;
}

int cb_is_empty(CircularBuffer *cb) {
    return cb->count == 0;
}
