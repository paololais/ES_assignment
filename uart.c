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
    
    U1MODEbits.UARTEN = 1; // enbale UART1
    U1STAbits.UTXEN = 1; // enable TX
    U1STAbits.URXDA = 1; // enab RX
    IEC0bits.U1RXIE = 1;   // enable RX interrupt
}

// writes a character to the UART1 using a circular buffer
// This function pushes the character into the circular buffer and enables the TX interrupt
void UART1_WriteChar(char c) {
    while (U1STAbits.UTXBF); // wait Tx buffer to be empty
    U1TXREG = c; // send character
    
    //IEC0bits.U1TXIE = 0; // Disable TX interrupt
    //cb_push(cb_tx, c);  // Push the character into the TX buffer
    //IEC0bits.U1TXIE = 1; // Enable TX interrupt
}

// reads a character from the UART1
char UART1_ReadChar(void) {
    return U1RXREG;
}

//circular buffer
void cb_init(CircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

void cb_push(CircularBuffer *cb, char value) {
    
    cb->buffer[cb->head] = value; // write the value
    cb->head = (cb->head + 1) % BUFFER_SIZE; // increment circularly
    if (cb->count == BUFFER_SIZE){
        cb->tail = (cb->tail + 1) % BUFFER_SIZE;
        return; // buffer is full, overwrite the oldest value, don't increment count
    }
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
