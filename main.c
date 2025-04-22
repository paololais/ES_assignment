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
#include "stdio.h"
//#include <stdlib.h> // For atoi

// Finite State Machine (FSM) states for UART communication
typedef enum {IDLE, S_dollar, S_R, S_A, S_T, S_E, S_comma, S_asterisk} UART_State;
char receivedXX[2]; // store values for $RATE,xx*
int success = 0; // flag to check if the value is valid
UART_State uartState = IDLE; // Initialize the UART state to IDLE

// circular buffer for TX
// used to send error messages, magnetometer data and yaw
CircularBuffer cb_tx;

// circular buffer for RX
// used to receive rate by user
CircularBuffer cb_rx;

char buffer[32];
unsigned int read_addr = 0x42;
unsigned int lsb;
unsigned int msb;
unsigned int raw;
int signed_value;

int mag_frequency = 5;

// Interrupt UART RX
void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
    char receivedChar = U1RXREG; // Legge carattere ricevuto
    cb_push(&cb_rx, receivedChar);
    IFS0bits.U1RXIF = 0; // Reset flag interrupt
}

// Reads the frequency value specified by the user.
// performs a check to see if the value is valid.
// Returns 1 if the value is valid, 0 otherwise.
// The valid values are 0, 1, 2, 4, 5, and 10.
int readFrequency(){
    if(cb_is_empty(&cb_rx)) return 0;
    
    IEC0bits.U1RXIE = 0;   // disable interrupt RX
    cb_pop(&cb_rx, &receivedXX[1]);
    IEC0bits.U1RXIE = 1;   // enable interrupt RX
    
    if(strcmp(receivedXX, "00") == 0 || strcmp(receivedXX, "01") == 0 || strcmp(receivedXX, "02") == 0 || 
            strcmp(receivedXX, "04") == 0 || strcmp(receivedXX, "05") == 0 || strcmp(receivedXX, "10") == 0){
        
        return 1;
    }
    
    else return 0;

    /*
    receivedXX[2] = '\0'; // Null-terminate the string

    // Convert the string to an integer
    int frequency = atoi(receivedXX);

    // Check if the frequency is one of the valid values
    switch (frequency) {
        case 0:
        case 1:
        case 2:
        case 4:
        case 5:
        case 10:
            return 1; // Valid 
        default:
            return 0; // Invalid, flag error
    }
    */
}

// Function that processes characters from the circular buffer
void processReceivedData() {
    char receivedChar;
    
    // If there are characters in the buffer
    while (!cb_is_empty(&cb_rx)) {
        IEC0bits.U1RXIE = 0;   // Disable RX interrupt
        // Pop the character from the buffer
        cb_pop(&cb_rx, &receivedChar);
        IEC0bits.U1RXIE = 1;   // Enable RX interrupt
        
        handle_UART_FSM(receivedChar); // Handle the character based on the FSM
    }
}

// Handles the UART Finite State Machine (FSM) based on the received character.
// This function processes the input character received via UART and updates the state of the FSM accordingly.
// recognizes a specific command format: $RATE,xx*.
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
            sprintf(buffer, "T*");
            for (int i = 0; i < strlen(buffer); i++){
                UART1_WriteChar(buffer[i]);
            }
            memset(buffer, 0, sizeof(buffer));
            break; 
        case S_E:
            if (receivedChar == ',') uartState = S_comma;
            else uartState = IDLE;
            break;
        case S_comma:
            receivedXX[0] = receivedChar;
            success = readFrequency();
            
            if(success) uartState = S_asterisk;
            else uartState = IDLE;
            
            break;
        case S_asterisk:
            if (receivedChar == '*'){
                sprintf(buffer, "$OK*");
                for (int i = 0; i < strlen(buffer); i++){
                    UART1_WriteChar(buffer[i]);
                }
                memset(buffer, 0, sizeof(buffer));
            }
            else{
                sprintf(buffer, "$ERR,1*");
                for (int i = 0; i < strlen(buffer); i++){
                    UART1_WriteChar(buffer[i]);
                }
                memset(buffer, 0, sizeof(buffer));
            }          
            uartState = IDLE;
            break;  
        default:
            uartState = IDLE;           
            break;  
    }
}

void printMagData(){
    spi_write_2_reg(read_addr, &lsb, &msb);
    lsb = lsb & 0x00F8;
    msb = msb << 8; //left shift by 8
    raw = msb | lsb; //put together the two bytes
    //raw = raw >> 3; //right shift by 3
    signed_value = (int) raw / 8; // alternativa più robusta

    sprintf(buffer, "$MAGX=%d*", signed_value);

    int l = strlen(buffer);
    for (int i = 0; i < l; i++) {
        UART1_WriteChar(buffer[i]);
    }
}

// function that waits 7 ms
void algorithm() {
    tmr_wait_ms(TIMER2, 7);
}

int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000; // disable analog inputs
    
    //variables
    int ret;    // variable to check if the algorithm has missed a deadline
    int i = 0; // variable to count 50 ticks (500ms)
    int missed_deadlines = 0; // variable to count missed deadlines of algorithm
    int mag_out = 0; // feedback magnetometer data
    
    // led2
    TRISGbits.TRISG9 = 0; // LED2 output
    LATGbits.LATG9 = 0; // switch off LED2
    
    spi_init();
    
    // Make the magnetometer switch to Sleep mode; then make it go to active mode; 
    // data rate = 25Hz
    mag_enable();
    
    UART1_Init(); // initialize UART1
    
    cb_init(&cb_tx);
    cb_init(&cb_rx);
    
    tmr_setup_period(TIMER1, 10); // Timer 1 for algorithm() - 100 Hz = 10ms
    while(1){
        algorithm();
        
        // after 50 ticks (500ms) blink LED2
        i++;
        if (i == 50) {
            i = 0;
            LATGbits.LATG9 = !LATGbits.LATG9; // blink LED2
        }
                
        processReceivedData();
        
        mag_out++;
        if(mag_out == 20){
            mag_out = 0;
            printMagData();        
        }

        ret = tmr_wait_period(TIMER1);
        
        if(ret) missed_deadlines++;
    }
    return 0;
}