/* 
 * File:   main_1501.c
 * Author: John Ash
 *
 * Created on July 9, 2015, 8:49 PM
 */

 
 
 /******************************** Includes **********************************/
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>


 /******************************** Pragma's **********************************/
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

#define _XTAL_FREQ  16000000    // this is used by the __delay_ms(xx) and __delay_us(xx) functions

//Delay function to wait for LED changes.
#define DELAY(ms)   do { int i; for (i = 0; i < (ms << 9); i++) { asm ("nop"); } } while(0);

/******************************** function #defines **********************************/
//Similar to an ENUM, but we are using bit wise manipulation 
#define RED 1        // 0b0001 
#define BLUE 2       // 0b0010
#define GREEN 4      // 0b0100

#define ON  0        // When the duty cycle is set to 0, the LED's turn on
#define OFF 1000     // The duty cycle uses a resolution of 1000, and when it is fully on, the LED will be off.

#define STAY_DELAY 10000   // Once the LED's hit a steady state of on or off, they will stay bright this long.
#define FADE_DELAY 6       // As the LED's are changing from ON to OFF, they will stay at each state for this long.



/******************************** private function definitions **********************************/
void SetDutyCycle(int Channel, signed int duty_cycle_value);


/******************************** private functions **********************************/
/**********************************************************************************
 *@brief This subroutine takes in a 10 bit number and sets the duty cycle register
 * for the PWM accordingly
 *
 **********************************************************************************/
void SetDutyCycle(
   int Channel,                  /*<< [in] Which PWM signal are we using? RED, BLUE, GREEN? */
   signed int duty_cycle_value   /*<< [in] What is the duty cycle you would like to set? */
)
{
    if( Channel & RED)
    {
        PWM1DCL = duty_cycle_value & 0xc0; //first set the 2 lsb bits
        PWM1DCH =  (duty_cycle_value >> 2);           //now set upper 8 msb bits
    }
    if( Channel & BLUE)
    {
        PWM4DCL = duty_cycle_value & 0xc0; //first set the 2 lsb bits
        PWM4DCH =  (duty_cycle_value >> 2);           //now set upper 8 msb bits
    }
    if( Channel & GREEN)
    {
        PWM3DCL = duty_cycle_value & 0xc0; //first set the 2 lsb bits
        PWM3DCH =  (duty_cycle_value >> 2);           //now set upper 8 msb bits
    }
}



/**
 *@brief This is the main function of the project, set the port assignments
 * than follow a hardcoded procedure for dimming and un-dimming the lighs
 *
 */
int main(int argc, char** argv) {
   
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


    int Duty_cycle = 0;
    int Duty_cycle2 = 0;
    int light_delay = 3;
    
    int Curr_chan = RED;

    while(1)
    {   
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

