/*
 * File:   uart.c
 * Author: paolo
 *
 * Created on 1 aprile 2025, 9.44
 */


#include "uart.h"

// Inizializzazione UART1
void UART1_Init(void) {
    TRISDbits.TRISD11 = 1; // Imposta RD11 come input (U1RX)
    TRISDbits.TRISD0 = 0;  // Imposta RD0 come output (U1TX)
    
    RPINR18bits.U1RXR = 75; // RD11 è mappato su U1RX
    RPOR0bits.RP64R = 1;    // RD0 è mappato su U1TX
    
    U1BRG = BRGVAL; // Configura il baud rate
    
    U1MODEbits.UARTEN = 1; // Abilita UART1
    U1STAbits.UTXEN = 1; // Abilita la trasmissione
    U1STAbits.URXDA = 1; // Abilita la ricezione
    IEC0bits.U1RXIE = 1;   // Abilita interrupt RX
}

// Scrive un carattere sulla UART1
void UART1_WriteChar(char c) {
    while (U1STAbits.UTXBF); // Attende se il buffer di trasmissione è pieno
    U1TXREG = c; // Invia il carattere
}

// Legge un carattere dalla UART1
char UART1_ReadChar(void) {
    return U1RXREG;
}

// Funzione di echo (reinvia il carattere ricevuto)
void UART1_Echo(void) {
    char receivedChar = UART1_ReadChar(); // Legge un carattere
    //updateWindow(receivedChar); // Aggiorna la finestra di rilevamento comandi
    UART1_WriteChar(receivedChar); // Reinvia il carattere ricevuto
}

//circular buffer

void cb_init(CircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

void cb_push(CircularBuffer *cb, char value) {
    
    cb->buffer[cb->head] = value; // Scrive il valore
    cb->head = (cb->head + 1) % BUFFER_SIZE; // Incremento circolare
    if (cb->count == BUFFER_SIZE){
        cb->tail = (cb->tail + 1) % BUFFER_SIZE;
        return; // Buffer pieno, non incremento count
    }
    cb->count++;
}

void cb_pop(CircularBuffer *cb, char *value) {
    *value = cb->buffer[cb->tail]; // Legge il valore
    cb->tail = (cb->tail + 1) % BUFFER_SIZE; // Incremento circolare
    cb->count--;
}

int cb_is_empty(CircularBuffer *cb) {
    return cb->count == 0;
}
