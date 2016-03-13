/* 
 * File:   main_1501.c
 * Author: John Ash
 *
 * Created on July 9, 2015, 8:49 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
/*
 *
 *
 *
 *
 */

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LPBOR = ON       // Low-Power Brown Out Reset (Low-Power BOR is enabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions

#define DELAY(ms)   do { int i; for (i = 0; i < (ms << 9); i++) { asm ("nop"); } } while(0);


#define RED 1
#define BLUE 2
#define GREEN 4

#define ON  0
#define OFF 1000

#define STAY_DELAY 10000
#define FADE_DELAY 6


void SetDutyCycle(int Channel, signed int duty_cycle_value);

//**********************************************************************************
// This subroutine takes in a 10 bit number and sets the duty cycle register
// for the PWM accordingly
//**********************************************************************************
void SetDutyCycle(int Channel, signed int duty_cycle_value)
{
    if( Channel & RED){
        PWM1DCL = duty_cycle_value & 0xc0; //first set the 2 lsb bits
        PWM1DCH =  (duty_cycle_value >> 2);           //now set upper 8 msb bits
    }
    if( Channel & BLUE){
        PWM4DCL = duty_cycle_value & 0xc0; //first set the 2 lsb bits
        PWM4DCH =  (duty_cycle_value >> 2);           //now set upper 8 msb bits
    }
    if( Channel & GREEN){
        PWM3DCL = duty_cycle_value & 0xc0; //first set the 2 lsb bits
        PWM3DCH =  (duty_cycle_value >> 2);           //now set upper 8 msb bits
    }
}

int main(int argc, char** argv) {
//    TRISA4=0; //Step 1. Disable PWM1 pin with TRIS
//    PWM3CON=0; //Step 2. Clear the PWM1CON
//    PR2=0xC7; //Step 3. Load the PR2 register with the PWM period value
//    //TRISA3=1;
//    PWM3DCH = 0; //Step 4. Clear PWM1DCH
//    PWM3DCL = 0b00111111&PWM1DCL;//and bits (7-6) of PWM1DCL
//    PIR1 = 0b11111101&PIR1;// Step 5. Clear the TMR2IF interrupt flag bit of the PIR1 register.
//    T2CON=0x04; //Step 5b. Confirgure bits with Timer2 prescale value
//    T2CON=0b00000100|T2CON; //Step 5c. Enable Timer2 by setting TMR2ON
//    PWM3CON=0b10000000|PWM1CON;//step 6. Enable PWM output pin
//    while(TMR2IF ==0); //Wait till overflow
//                        //step 7. Enable the PWMx pin driver by clearing the tris
//    PWM3CON=0b10000000|PWM1CON; // and setting PWMxOE
//
//    TMR2IF=0;
//    PWM3CON=0xE0; //step 8. Configure PWM module by loading values


    OSCCONbits.IRCF=0x0F;   //set OSCCON IRCF bits to select OSC frequency=16Mhz
    OSCCONbits.SCS=0x02;    //set the SCS bits to select internal oscillator block
    // OSCON should be 0x7Ah now.

    // Set up I/O pins
    ANSELAbits.ANSELA=0;    // set all analog pins to digital I/O
    ADCON0bits.ADON=0;      // turn ADC off
    DACCON0bits.DACEN=0;    // turn DAC off

      // PORT A Assignments
    TRISAbits.TRISA0 = 0;   // RA0 = nc
    TRISAbits.TRISA1 = 0;   // RA1 = nc
    TRISAbits.TRISA2 = 0;   // RA2 = PWM Output (CCP1) connected to LED
    TRISAbits.TRISA3 = 0;   // RA3 = nc (MCLR)
    TRISAbits.TRISA4 = 0;   // RA4 = nc
    TRISAbits.TRISA5 = 0;   // RA5 = nc
    //APFCONbits.CLC1SEL=0;       // The CCP1SEL bit selects which pin the PWM output is on.
                                // The default value for this bit is 0 which sets the
                                // PWM output on RA2.  If you want the PWM output on
                                // RA5 instead then set this bit to a 1.

    //******************************************************************************************
    // PWM Period = (1/Fosc) * 4 * (TMR2 Prescaler)* (PR2+1)
    //******************************************************************************************
    // Here are sample PWM periods for different TMR2 Prescalar values for Fosc=16Mhz and PR2=255
    //******************************************************************************************
    // TMR2 Prescalar=1: PWM Period = (1/16000000)*4*1*256 = 64 us or 15.63 khz
    // TMR2 Prescalar=4: PWM Period = (1/16000000)*4*4*256 = 256 us or 3.91 khz
    // TMR2 Prescalar=16: PWM Period = (1/16000000)*4*16*256= 1.024 ms or .976 khz
    // TMR2 Prescalar=64: PWM Period = (1/16000000)*4*64*256= 4.096 ms or .244 khz
    //
    // For this example we will choose the PWM period of 64us (15.63 kHz) so most people
    // will not be able to hear it.
    // ***** Setup PWM output

    TRISAbits.TRISA2 = 1;       // disable pwm pin output for the moment
    TRISAbits.TRISA4 = 1;
    TRISAbits.TRISA5 = 1;

    PR2 = 0xff;                 // set PWM period as 255 per our example above

    PIR1bits.TMR2IF=0;      // clear TMR2 interrupt flag
    T2CONbits.T2CKPS=0x00;      // select TMR2 prescalar as divide by 1 as per our example above
    T2CONbits.TMR2ON=1;     // turn TMR2 on
    TRISAbits.TRISA2 = 0;   // turn PWM output back on
    TRISAbits.TRISA4 = 0;
    TRISAbits.TRISA5 = 0;

    PWM1CON = 0b11000000;
    PWM3CON = 0b11000000;
    PWM4CON = 0b11000000;


//    //OSCCON=0x7A;            // 16 MHz
//    T2CON=0x04;             // was 04,
//    while(TMR2IF==0);
//    TMR2IF=0;
//    PR2=0x7C;
    int Duty_cycle = 0;
    int Duty_cycle2 = 0;
    //TRISA4 = 0;
    int light_delay = 3;
    
    int Curr_chan = RED;

    while(1)
    {
/*
        SetDutyCycle(Curr_chan,(Duty_cycle)%1000);
        Duty_cycle += 16;
        DELAY(1000);
//        LATA4 = 1;
//        DELAY(1000);
//        LATA4 = 0;
        if(Duty_cycle > 900)
        {
            Duty_cycle = 0;
            Curr_chan *= 2;
            if(Curr_chan == 8)
                Curr_chan = RED;
        }


        break;
*/
        
        //Start White
        Curr_chan = BLUE||GREEN||RED;
        Duty_cycle = ON;
        SetDutyCycle(Curr_chan, Duty_cycle);
        DELAY(STAY_DELAY)
 //Light blue: Blue + Green(yellow) ==>Turn off Red
        Duty_cycle = ON;
        while(Duty_cycle != OFF){
            Curr_chan = RED;
            Duty_cycle += 1;
            SetDutyCycle(Curr_chan, Duty_cycle);
            DELAY( light_delay );
        }
        DELAY(STAY_DELAY)
    //Purple: Blue + Red(white) ==> Turn Red on, Turn Green off
        Duty_cycle = ON;
        Duty_cycle2 = OFF;
        while(Duty_cycle != OFF){
            Duty_cycle  += 1; // green
            Duty_cycle2 -= 1; //red
            SetDutyCycle(GREEN, Duty_cycle);
            SetDutyCycle(RED, Duty_cycle2);
            DELAY( light_delay );
        }
        DELAY(STAY_DELAY)
    //Dark Blue: Blue ==> Red off
        Duty_cycle = ON;
        while(Duty_cycle != OFF){
            Curr_chan = RED;
            Duty_cycle += 1;
            SetDutyCycle(Curr_chan, Duty_cycle);
            DELAY( light_delay );
        }
        DELAY(STAY_DELAY)
    //White ==> Red on, Green on
        Duty_cycle = OFF;
        while(Duty_cycle != ON){
            Duty_cycle -= 1;
            SetDutyCycle(RED, Duty_cycle);
            SetDutyCycle(GREEN, Duty_cycle);
            DELAY( light_delay );
        }

    }
    return (EXIT_SUCCESS);
}

