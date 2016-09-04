# Controlling RGB LED PWMs

As part of my wedding table center pieces I setup a  micro controller to control a string of RGB LEDS to fade between our wedding colors.

I used a PIC12F1501 due to its small formfactor, and its access to 3 PWM signals. 

Each PWM signal is terminated at the microcontroller lines, this means that when the PWM singal is high the LED's are off, and when the PWM's are low the LED's turn on.