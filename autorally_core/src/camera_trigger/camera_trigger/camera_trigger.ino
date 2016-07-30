/**********************************************
 * @file cameraTrigger.ino
 * @author Kamil Saigol <kamilsaigol@gatech.edu>
 * @date July 24, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief Handles camera triggering
 *
 * @details This program triggers the onboard
 *          cameras and ensures that the trigger
 *          frequency updates are synchronous with
 *          the GPS PPS signal.
 ***********************************************/

#include <avr/io.h>
#include <avr/interrupt.h>

/********** Arduino Micro **********/
// Board: https://www.arduino.cc/en/Main/ArduinoBoardMicro
// Microcontroller: http://www.atmel.com/Images/Atmel-7766-8-bit-AVR-ATmega16U4-32U4_Datasheet.pdf
// Clock frequency: 16 MHz
// External interrupts: Pins 0 (RX), 1 (TX), 2, 3, 7
// Timers: Timer0 (8-bit), Timer1 (16-bit), Timer3 (16-bit)
/***********************************/

const byte PPS_PIN = 4;                 // Input pin for PPS signal (must be PCInt-capable)
const byte CAMERA_TRIGGER_PIN = 5;      // Output pin to trigger cameras
volatile int trigger_frequency = 40;    // FPS
volatile bool secondPassed = true;      // Flag indicating whether messages should be transmitted via Serial

void reconfigure_camera_trigger()
{
    // Disable interrupts globally
    cli();

    // Reset timer control register
    TCCR3A = 0;
    TCCR3B = 0;

    // Josh: 623-499-3149

    // Prescaler = 1/256
    // Arduino Micro clock = 16 MHz
    // Timer Resolution = 1.0 / ((16 * 10^6) / 256) = 0.000016
    const float resolution = 0.000016;

    // Target time = 1.0 / (2.0 * trigger_frequency) -- bit needs to be flipped on and off to trigger camera, so twice FPS
    // CTC count = (Target time / resolution) - 1.0
    //           = ((1.0 / 2.0 * trigger_frequency) / resolution) - 1.0
    //           = (1.0 / (2.0 * trigger_frequency * resolution)) - 1.0
    OCR3A = (int) (1.0 / (2.0 * trigger_frequency * resolution) - 1.0);

    TCCR3B |= (1 << WGM32); // Set Timer 3 to CTC mode
    TCCR3B |= (1 << CS32);  // Set prescaler to 1/256
    TIMSK3 = (1 << OCIE3A); // enable compare ISR for Timer 3

    // Enable interrupts globally
    sei();
}

void setup()
{
    pinMode(PPS_PIN, INPUT_PULLUP);
    pinMode(CAMERA_TRIGGER_PIN, OUTPUT);

    // Initial camera trigger timer configuration
    reconfigure_camera_trigger();

    /** Set up Timer 1 input capture pin to act as PCInt **/
    // Disable global interrupts
    cli();

    // reset timer control register
    TCCR1A = 0;
    TCCR1B = 0;

    TCCR1B |= (1 << ICES1); // enable input capture on falling edge for Timer 1
    TIMSK1 = (1 << ICIE1);  // enable input capture ISR for Timer 1

    // Enable interrupts globally
    sei();

    // Start serial connection
    Serial.begin(115200);
}

void loop()
{
    if(secondPassed)
    {
        if(trigger_frequency <= 0)
        {
            Serial.print("#eTrigger frequency negative or zero but should be positive,");
        }
        Serial.print("#t");
        Serial.print(trigger_frequency, DEC);
        Serial.print(',');
        Serial.print(digitalRead(PPS_PIN));
        Serial.print('\n');
        secondPassed = false;
    }
}

// Triggered when serial data is available
void serialEvent()
{
    while(Serial.available())
    {
        // We only expect an integer representing FPS
        trigger_frequency = Serial.parseInt();
    }
}

// Timer1 input capture interrupt
ISR(TIMER1_CAPT_vect)
{
    // Reconfigure using current trigger frequency
    reconfigure_camera_trigger();

    // Flag messages for transmission
    if(!secondPassed) secondPassed = true;
}

// Timer3 compare interrupt
ISR(TIMER3_COMPA_vect)
{
    // Trigger camera
    digitalWrite(CAMERA_TRIGGER_PIN, !digitalRead(CAMERA_TRIGGER_PIN));
}
