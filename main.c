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
#include <stdlib.h>
#include <math.h>

#define NUM_SAMPLES 5 // Number of samples to average
// macros for axes
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

// Finite State Machine (FSM) states for UART communication
typedef enum {IDLE, S_dollar, S_R, S_A, S_T, S_E, S_comma, S_asterisk} UART_State;
char receivedXX[3]; // store values for $RATE,xx*
int success = 0; // flag to check if the value is valid
UART_State uartState = IDLE; // Initialize the UART state to IDLE

// circular buffer for TX
// used to send error messages, magnetometer data and yaw
CircularBuffer cb_tx;

// circular buffer for RX
// used to receive rate by user
CircularBuffer cb_rx;

char buffer[32];    // buffer for UART messages
unsigned int read_addr_x = 0x42; // address of X axis
unsigned int read_addr_y = 0x44; // address of Y axis
unsigned int read_addr_z = 0x46; // address of Z axis
unsigned int lsb;
unsigned int msb;
unsigned int raw;
int signed_value;

int mag_frequency = 5;                 // default frequency 5Hz
// magnetometer data
int x_axis_values[NUM_SAMPLES] = {0}; // Array to store last 5 measurement
int current_index_x = 0;                  // Index to track the oldest measurement
int samples_collected_x = 0;              // Counter for total samples collected
int y_axis_values[NUM_SAMPLES] = {0}; // Array to store last 5 measurement
int current_index_y = 0;                  // Index to track the oldest measurement
int samples_collected_y = 0;              // Counter for total samples collected
int z_axis_values[NUM_SAMPLES] = {0}; // Array to store last 5 measurement
int current_index_z = 0;                  // Index to track the oldest measurement
int samples_collected_z = 0;              // Counter for total samples collected
// magnetometer data average values
double x_avg;
double y_avg;
double z_avg;

// Interrupt UART RX
void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
    char receivedChar = U1RXREG; // reads the received character
    cb_push(&cb_rx, receivedChar);
    IFS0bits.U1RXIF = 0; // Reset flag interrupt
}

// Interrupt UART TX
void __attribute__((__interrupt__, __auto_psv__)) _U1TXInterrupt() {
    IFS0bits.U1TXIF = 0; // Clear the TX interrupt flag
    
    char c;
    
    while(U1STAbits.UTXBF == 0){
        // If there are characters in the TX buffer, send them
        if (!cb_is_empty(&cb_tx)) {
            cb_pop(&cb_tx, &c); // Pop a character from the TX buffer
            U1TXREG = c;        // Write the character to the UART TX register
        } else {
            IEC0bits.U1TXIE = 0;
            break;
        }
    }
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
    
    receivedXX[2]='\0';
    if(strcmp(receivedXX, "00") == 0 || strcmp(receivedXX, "01") == 0 || strcmp(receivedXX, "02") == 0 || 
            strcmp(receivedXX, "04") == 0 || strcmp(receivedXX, "05") == 0 || strcmp(receivedXX, "10") == 0){
        
        mag_frequency = atoi(receivedXX);
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
            break; 
        case S_E:
            if (receivedChar == ',') uartState = S_comma;
            else uartState = IDLE;
            break;
        case S_comma:
            receivedXX[0] = receivedChar;
            success = readFrequency();
            
            if(success) uartState = S_asterisk;
            else {
                sprintf(buffer, "$ERR,1*");
                IEC0bits.U1TXIE = 0;
                for (int i = 0; i < strlen(buffer); i++) {
                    cb_push(&cb_tx, buffer[i]);
                }
                IEC0bits.U1TXIE = 1;
                memset(buffer, 0, sizeof(buffer));
            
                uartState = IDLE;
            }
            
            break;
        case S_asterisk:
            if (receivedChar == '*'){
                sprintf(buffer, "$OK - %d*", mag_frequency);
                IEC0bits.U1TXIE = 0;
                for (int i = 0; i < strlen(buffer); i++) {
                    cb_push(&cb_tx, buffer[i]);
                }
                IEC0bits.U1TXIE = 1;
                memset(buffer, 0, sizeof(buffer));                
            }
            
            uartState = IDLE;
            break;  
        default:
            uartState = IDLE;           
            break;  
    }
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

// Function to add a new measurement to the corresponding axis array
void addMeasurement(int axis, int new_value) {
    switch(axis) {
        case AXIS_X:
            x_axis_values[current_index_x++] = new_value;
             // Update index to point to the next position (which will be the oldest value)
            current_index_x = (current_index_x + 1) % NUM_SAMPLES;
            // Keep track of how many samples we've collected
            if (samples_collected_x < NUM_SAMPLES) samples_collected_x++;
            break;
        case AXIS_Y:
            y_axis_values[current_index_y++] = new_value;
            // Update index to point to the next position (which will be the oldest value)
            current_index_y = (current_index_y + 1) % NUM_SAMPLES;
            // Keep track of how many samples we've collected
            if (samples_collected_y < NUM_SAMPLES) samples_collected_y++;
            break;
        case AXIS_Z:
            z_axis_values[current_index_z++] = new_value;
            // Update index to point to the next position (which will be the oldest value)
            current_index_z = (current_index_z + 1) % NUM_SAMPLES;
            // Keep track of how many samples we've collected
            if (samples_collected_z < NUM_SAMPLES) samples_collected_z++;
            break;
    }
}

// function to get magnetometer data of each axis
// and store it in the corresponding array
void getMagData(){
    // X axis
    spi_write_2_reg(read_addr_x, &lsb, &msb);
    lsb = lsb & 0x00F8;
    msb = msb << 8; //left shift by 8
    raw = msb | lsb; //put together the two bytes
    //raw = raw >> 3; //right shift by 3
    signed_value = (int) raw / 8; // right shift by 3 corresponds to dividing by 8
    addMeasurement(AXIS_X, signed_value); 

    // Y axis
    spi_write_2_reg(read_addr_y, &lsb, &msb);
    lsb = lsb & 0x00F8;
    msb = msb << 8;
    raw = msb | lsb;
    signed_value = (int) raw / 8;
    addMeasurement(AXIS_Y, signed_value);
    
    // Z axis
    spi_write_2_reg(read_addr_z, &lsb, &msb);
    lsb = lsb & 0x00FE; // for z axis LSB the register bits are different
    msb = msb << 8;
    raw = msb | lsb;
    //signed_value = (int) raw >> 1;
    signed_value = (int) raw / 2; // right shift by 1 corresponds to dividing by 2
    addMeasurement(AXIS_Z, signed_value);
}

// Calculate the average of stored measurements
float averageMeasurements(int axis) {    
    float sum = 0;
    int count = 0;
    
     switch(axis) {
        case AXIS_X:
            count = samples_collected_x;
            for (int i = 0; i < count; i++) sum += x_axis_values[i];
            break;
        case AXIS_Y:
            count = samples_collected_y;
            for (int i = 0; i < count; i++) sum += y_axis_values[i];
            break;
        case AXIS_Z:
            count = samples_collected_z;
            for (int i = 0; i < count; i++) sum += z_axis_values[i];
            break;
    }
    
    // Return the average
    return (count == 0) ? 0 : sum / count;
}

// Function to print magnetometer data using protocol $MAG,x,y,z*
void printMagData(){    
    sprintf(buffer, "$MAG,%.1f,%.1f,%.1f*", x_avg,y_avg,z_avg);

    IEC0bits.U1TXIE = 0;
    for (int i = 0; i < strlen(buffer); i++) {
        cb_push(&cb_tx, buffer[i]);
    }
    IEC0bits.U1TXIE = 1;
}

// Function to print yaw angle using protocol $YAW,xx*
void printYawAngle(){
    double heading_rad = atan2(y_avg, x_avg);
    double heading_deg = heading_rad * (180.0 / M_PI); // Convert to degrees

    if (heading_deg < 0)
        heading_deg += 360.0; // Normalize to 0-360
    
    sprintf(buffer, " $YAW,%.1f*", heading_deg);

    IEC0bits.U1TXIE = 0;
    for (int i = 0; i < strlen(buffer); i++) {
        cb_push(&cb_tx, buffer[i]);
    }
    IEC0bits.U1TXIE = 1;
}

// periodic function that runs for 7ms
void algorithm() {
    tmr_wait_ms(TIMER2, 7);
}

int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000; // disable analog inputs
    
    //variables
    int ret;    // variable to check if the algorithm has missed a deadline
    int i = 0; // variable to count 50 ticks (500ms)
    int missed_deadlines = 0; // variable to count missed deadlines of algorithm
    int count_magPrint = 0; // feedback magnetometer data
    int count_getMagData = 0; // counter to sincronize getMagData at 25Hz
    int count_yaw = 0; // counter to sincronize print yaw angle at 5Hz
    int count_dead = 0; // counter to sincronize print missed deadlines
    
    TRISGbits.TRISG9 = 0; // LED2 output
    LATGbits.LATG9 = 0; // switch off LED2 at the beginning
    
    spi_init(); // initialize SPI
    
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
        
        count_getMagData++;
        // get magnetometer data at 25Hz
        // every 4 ticks of the algorithm (4*10ms = 40ms)
        if(count_getMagData == 4){
            count_getMagData = 0;
            getMagData();
            x_avg = averageMeasurements(AXIS_X);
            y_avg = averageMeasurements(AXIS_Y);
            z_avg = averageMeasurements(AXIS_Z);
        }
        
        count_magPrint++;
        // print magnetometer data at mag_frequency
        // every 100/mag_frequency ticks of the algorithm
        if(mag_frequency!=0 && count_magPrint >= (100/mag_frequency)){
            count_magPrint = 0;
            printMagData();        
        }
        
        count_yaw++;
        // print yaw angle at 5Hz
        // every 20 ticks of the algorithm (20*10ms = 200ms)
        if(count_yaw == 20){
            count_yaw = 0;
            printYawAngle();
        }

        ret = tmr_wait_period(TIMER1);
        if(ret) missed_deadlines++;
        
        // OPTIONAL: print missed deadlines of algorithm
        count_dead++;
        // every 500 ticks of the algorithm (500*10ms = 5s)
        if(count_dead==500){
            count_dead=0;
            sprintf(buffer, "$MISS%d*", missed_deadlines);

            IEC0bits.U1TXIE = 0;
            for (int i = 0; i < strlen(buffer); i++) {
                cb_push(&cb_tx, buffer[i]);
            }
            IEC0bits.U1TXIE = 1;
        }
    }
    return 0;
}