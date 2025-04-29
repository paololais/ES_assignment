/*
 * File:   timer.c
 * Author: paolo
 *
 * Created on February 28, 2025, 2:04 PM
 */

#include "timer.h"

void tmr_setup_period(int timer, int ms){
    //the first function setups the timer timer to count for the specified number of milliseconds. 
    //The function should support values up to 200 millisecond. 
    //It should start the timer
    if(ms > 200) return;
    
    long int Fcy = 72000000;
    // Formula: PRx = (Fcy / Prescaler) * (ms / 1000)
    //PR1 = 56250;
    
    if(timer == 1){
        PR1 = (int) ((Fcy / 256) * (ms / 1000.0));
    
        T1CONbits.TCS = 0;
        TMR1 = 0; // reset timer counter
        T1CONbits.TCKPS = 3; // prescaler 1:256
        IFS0bits.T1IF = 0;
        T1CONbits.TON = 1; // starts the timer!
    }
    
    else if(timer == 2){
        PR2 = (int) ((Fcy / 256) * (ms / 1000.0));

        T2CONbits.TCS = 0;
        TMR2 = 0; // reset timer counter
        T2CONbits.TCKPS = 3; // prescaler 1:256
        IFS0bits.T2IF = 0;
        T2CONbits.TON = 1; // starts the timer!
    }
    else if(timer == 3){
        PR3 = (int) ((Fcy / 256) * (ms / 1000.0));

        T3CONbits.TCS = 0;
        TMR3 = 0; // reset timer counter
        T3CONbits.TCKPS = 3; // prescaler 1:256
        IFS0bits.T3IF = 0;
        T3CONbits.TON = 1; // starts the timer!
    }
    else if(timer == 4){
        PR4 = (int) ((Fcy / 256) * (ms / 1000.0));

        T4CONbits.TCS = 0;
        TMR4 = 0; // reset timer counter
        T4CONbits.TCKPS = 3; // prescaler 1:256
        IFS1bits.T4IF = 0;
        T4CONbits.TON = 1; // starts the timer!
    }
    
}

int tmr_wait_period(int timer){
    if (timer == 1){
        if(IFS0bits.T1IF == 0){
            while (IFS0bits.T1IF == 0);
            IFS0bits.T1IF = 0;
            return 0; 
        }
        else if(IFS0bits.T1IF == 1){
            IFS0bits.T1IF = 0;
            return 1;
        }
    }
    else if(timer == 2){
        if(IFS0bits.T2IF == 0){
            while (IFS0bits.T2IF == 0);
            IFS0bits.T2IF = 0;
            return 0;
        }
        else if(IFS0bits.T2IF == 1){
            IFS0bits.T2IF = 0;
            return 1;
        }
    }
    else if(timer == 3){
        if(IFS0bits.T3IF == 0){
            while (IFS0bits.T3IF == 0);
            IFS0bits.T2IF = 0;
            return 0;
        }
        else if(IFS0bits.T3IF == 1){
            IFS0bits.T3IF = 0;
            return 1;
        }
    }
    else if(timer == 4){
        if(IFS1bits.T4IF == 0){
            while (IFS1bits.T4IF == 0);
            IFS1bits.T4IF = 0;
            return 0;
        }
        else if(IFS1bits.T4IF == 1){
            IFS1bits.T4IF = 0;
            return 1;
        }
    }
}
void tmr_wait_ms(int timer, int ms){
    long int Fcy = 72000000;
    // Formula: PRx = (Fcy / Prescaler) * (ms / 1000)
    //PR1 = 56250;
    if(ms > 200) return;
    
    if(timer == 1){
        PR1 = (int) ((Fcy / 256) * (ms / 1000.0));
    
        T1CONbits.TCS = 0;
        TMR1 = 0; // reset timer counter
        T1CONbits.TCKPS = 3; // prescaler 1:256
        IFS0bits.T1IF = 0;
        T1CONbits.TON = 1; // starts the timer!
    }
    
    else if(timer == 2){
        PR2 = (int) ((Fcy / 256) * (ms / 1000.0));

        T2CONbits.TCS = 0;
        TMR2 = 0; // reset timer counter
        T2CONbits.TCKPS = 3; // prescaler 1:256
        IFS0bits.T2IF = 0;
        T2CONbits.TON = 1; // starts the timer!
    }
    else if(timer == 3){
        PR3 = (int) ((Fcy / 256) * (ms / 1000.0));

        T3CONbits.TCS = 0;
        TMR3 = 0; // reset timer counter
        T3CONbits.TCKPS = 3; // prescaler 1:256
        IFS0bits.T3IF = 0;
        T3CONbits.TON = 1; // starts the timer!
    }
    else if(timer == 4){
        PR4 = (int) ((Fcy / 256) * (ms / 1000.0));

        T4CONbits.TCS = 0;
        TMR4 = 0; // reset timer counter
        T4CONbits.TCKPS = 3; // prescaler 1:256
        IFS1bits.T4IF = 0;
        T4CONbits.TON = 1; // starts the timer!
    }
    
    if(timer == 1){
        while (IFS0bits.T1IF == 0);

        IFS0bits.T1IF = 0;      
    }
    else if(timer == 2){
        while (IFS0bits.T2IF == 0);

        IFS0bits.T2IF = 0;      
    }
    else if(timer == 3){
        while (IFS0bits.T3IF == 0);

        IFS0bits.T3IF = 0;      
    }
    else if(timer == 4){
        while (IFS1bits.T4IF == 0);

        IFS1bits.T4IF = 0;      
    }
}