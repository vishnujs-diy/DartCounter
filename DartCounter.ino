#include "LowPower.h"
#include <time.h>

#define ON 1
#define OFF 0
#define DEBOUNCE_TIME 100    // 100ms
#define IDLE_TIMEOUT  5000  // 1 min

#define COUNTER_INPUT 2
#define BTN_RESET 3
#define BTN_SET_MAG A4
                                // A  B  C  D  E F G   A B  C  D  E F G
volatile int SevenSegPins[2][7]={{10,11,12,13,A0,9,8},{6,7,A1,A2,A3,5,4}};
volatile int counter;
volatile unsigned long last_interrupt;
volatile int display_state;
volatile int mag_size;

/*
 * Since the Seven segment display is wired directly, use the following table for displaying numbers
 *        +-A-+
 *        F   B
 *        +-G-+
 *        E   C
 *        +-D-+
 */
int digits[10][7] = {//A,B,C,D,E,F,G
                      {0,0,0,0,0,0,1}, // 0
                      {1,0,0,1,1,1,1}, // 1
                      {0,0,1,0,0,1,0}, // 2
                      {0,0,0,0,1,1,0}, // 3
                      {1,0,0,1,1,0,0}, // 4
                      {0,1,0,0,1,0,0}, // 5
                      {0,1,0,0,0,0,0}, // 6
                      {0,0,0,1,1,1,1}, // 7
                      {0,0,0,0,0,0,0}, // 8
                      {0,0,0,0,1,0,0}  // 9
                      };

/*
 * Predifine MAG sizes here.
 */
#define MAX_MAG_SIZES 4
int mag_sizes[MAX_MAG_SIZES] = {6,10,12,18};

/*
 * Set a digit onto a seven segment display
 */
void setDigit(int pinArray[7], int digit) {
  if (digit > 9) {
    return;
  }
  for(int i=0;i<7;i++) {
    digitalWrite(pinArray[i],digits[digit][i]);
  }
}

/*
 * Set a number to the display
 */
void setNumber(int num) {
  setDigit(SevenSegPins[0],num%10);
  setDigit(SevenSegPins[1],num/10);
  display_state = ON;
}

/*
 * reset display to mag size
 */
void resetCounter() {
  counter = mag_sizes[mag_size];
  setNumber(counter);
}

/*
 * Timer 1 call back, timer1 is set to 1sec interval.
 * This is used to switch of the display and put the uC to deep sleep.
 * This helps in bringing down the power consumption from ~30mA to ~18mA.
 * Cannot bring it down further as the IR sensor, power LED and pin 13 LED will be on.
 * Power LED 3mA, pin 13 3mA, IR sensor with opAmp 12mA, uC in sleep takes only 0.3mA
 */
ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz

  /*
   * Check if we have reached idle time.
   */
  if (((millis() - last_interrupt) > IDLE_TIMEOUT) && display_state != OFF) {

    /*
     * Switch off the display
     */
    for(int i=0;i<2;i++) {
        for(int j=0;j<7;j++) {
        digitalWrite(SevenSegPins[i][j],1);
      }
    }
    display_state = OFF;

    /*
     * Set uC to deep sleep for ever
     * It will wake up on interrupt.
     */
    LowPower.powerDown(SLEEP_FOREVER , ADC_OFF, BOD_OFF);  
  }  
}

/*
 * handle pin change interrupt for A4 which is the set mag pin
 */
ISR (PCINT1_vect) {
  unsigned long now = millis();

  /*
   * SW Debounce
   */
  if (now - last_interrupt < DEBOUNCE_TIME) {
    return;
  }
  
  /*
   * Tickle last interrupt
   */
  last_interrupt = now;

  /*
   * Move Mag size to next and reset.
   */
  mag_size = (mag_size+1)% MAX_MAG_SIZES;
  resetCounter();
}  

ISR (PCINT2_vect) { // handle pin change interrupt for D2 which is the reset pin
  /*
   * Tickle last interrupt
   */
  last_interrupt = millis();

  /*
   * Reset
   */
  resetCounter();
}  

void counterInterrupt() {
  /*
   * Tickle last interrupt
   */
    last_interrupt = millis();

    /*
     * reduce counter, 
     * If the counter is at 0 and we get an interrupt, that means the mag has been reloaded and a shot was fired.
     * if so, reset the counter and reduce 1
     */
    if (counter) {
      counter--;
    } else {
      counter = mag_sizes[mag_size] - 1;
    }
    setNumber(counter);
}

/*
 * Enable pin change interrupt
 */
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}
   

void setup() {

  /*
   * set all seven segment display pins to output
   */
  for (int i=0;i<2;i++) {
      for(int j=0;j<7;j++){
        pinMode(SevenSegPins[i][j],OUTPUT);
      }
  }

  /*
  * set counter input and button pins
  */
  pinMode(COUNTER_INPUT, INPUT);
  attachInterrupt(digitalPinToInterrupt(COUNTER_INPUT), counterInterrupt, FALLING);
  
  pinMode(BTN_RESET, INPUT_PULLUP);
  pciSetup(BTN_RESET);
  
  pinMode(BTN_SET_MAG, INPUT_PULLUP);
  pciSetup(BTN_SET_MAG);

  /*
   * Set mag size to default
   */
  mag_size = 0;
  resetCounter();
  
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts

}

void loop() {
}
