/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Linear speed ramp controller.
 *
 * Stepper motor driver, increment/decrement the position and outputs the
 * correct signals to stepper motor.
 *
 * - File:               speed_cntr.c
 * - Compiler:           IAR EWAAVR 4.11A
 * - Supported devices:  All devices with a 16 bit timer can be used.
 *                       The example is written for ATmega48
 * - AppNote:            AVR446 - Linear speed control of stepper motor
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support email: avr@atmel.com
 *
 * $Name: RELEASE_1_0 $
 * $Revision: 1.2 $
 * $RCSfile: speed_cntr.c,v $
 * $Date: 2006/05/08 12:25:58 $
 *****************************************************************************/

#include "speed_cntr.h"
//! Cointains data for timer interrupt.
//speedRampData spool;

/*! \brief Move the stepper motor a given number of steps.
 *
 *  Makes the stepper motor move the given number of steps.
 *  It accelrate with given accelration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  \param step  Number of steps to move (pos - CW, neg - CCW).
 *  \param accel  Accelration to use, in 0.01*rad/sec^2.
 *  \param decel  Decelration to use, in 0.01*rad/sec^2.
 *  \param speed  Max speed, in 0.01*rad/sec.
 * 
 */
speedRampData spool1;
speedRampData winder1;
GLOBAL_FLAGS status_state;

void stop_running(void) {
  spool1.run_state = DECEL;
}

void winder_stop_running(void) {
  winder1.run_state = DECEL;
}

void winderSetSpeed(unsigned speed, unsigned acceleration)  {
  
  //TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));
  winder1.min_delay = T2_A_T_x100 / speed;
  Serial.print("Min delay winder ");
  Serial.println(winder1.min_delay);
  // Set accelration by calc the first (c0) step delay .
  // step_delay = 1/tt * sqrt(2*alpha/accel)
  // step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
  if(winder1.run_state == STOP) {
    winder1.step_delay = (T2_FREQ_148 * sqrt_calc(T2_A_SQ / acceleration))/100;
  }
    
  if(winder1.step_delay <= winder1.min_delay){
    winder1.step_delay = winder1.min_delay;
    winder1.run_state = RUN;
  }
  else{
    winder1.run_state = ACCEL;
  }

  winder1.accel_count = 0;
  status_state.running = TRUE;
  OCR2A = winder1.step_delay;
  // Set Timer/Counter to divide clock by 64
  TCCR2B |= ((0<<CS22)|(1<<CS21)|(1<<CS20));
  //Serial.print("Running state ");
  //Serial.println(spool.run_state);

}

void setSpeed(unsigned speed, unsigned acceleration)  {
  
  //TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));
  spool1.min_delay = A_T_x100 / speed;
  //Serial.print("Min delay ");
  //Serial.println(spool.min_delay);
  // Set accelration by calc the first (c0) step delay .
  // step_delay = 1/tt * sqrt(2*alpha/accel)
  // step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
  if(spool1.run_state == STOP) {
    spool1.step_delay = (T1_FREQ_148 * sqrt_calc(A_SQ / acceleration))/100;
  }
    
  if(spool1.step_delay <= spool1.min_delay){
    spool1.step_delay = spool1.min_delay;
    spool1.run_state = RUN;
  }
  else{
    spool1.run_state = ACCEL;
  }

  spool1.accel_count = 0;
  status_state.running = TRUE;
  OCR1A = spool1.step_delay;
  // Set Timer/Counter to divide clock by 64
  TCCR1B |= ((0<<CS12)|(1<<CS11)|(1<<CS10));
  //Serial.print("Running state ");
  //Serial.println(spool.run_state);

}


/*! \brief Init of Timer/Counter1.
 *
 *  Set up Timer/Counter1 to use mode 1 CTC and
 *  enable Output Compare A Match Interrupt.
 */
void spoolTimer1Init (void)
{
  TCCR1A = 0b00000000;//WGM1 3:0 = 4 0b0100 - CTC mode
  TCCR1B = 0b00001011;//WGM1 3 = 1; prescaler 0b011 /64
  TIMSK1 |= 0b00000010;       //set for output compare interrupt
  // Tells what part of speed ramp we are in.
  spool1.run_state = STOP;
    
}

void winderTimer2Init (void)
{
  TCCR2A = 0b00000010;//WGM 2:0 = 2 0b010 - CTC mode
  TCCR2B = 0b00000110;//WGM02 = 0; prescaler 0b111 /1024
  TIMSK2 |= 0b00000010;       //set for output compare interrupt
  //OCR2A = 77;//
  winder1.run_state = STOP;
    
}

/*! \brief Timer/Counter1 Output Compare A Match Interrupt.
 *
 *  Timer/Counter1 Output Compare A Match Interrupt.
 *  Increments/decrements the position of the stepper motor
 *  exept after last position, when it stops.
 *  The \ref step_delay defines the period of this interrupt
 *  and controls the speed of the stepper motor.
 *  A new step delay is calculated to follow wanted speed profile
 *  on basis of accel/decel parameters.
 */

ISR(TIMER1_COMPA_vect)
{
  //Serial.print("ISR run state ");
  //Serial.print(spool.run_state);
  // Holds next delay period.
  unsigned int new_step_delay;
  // Remember the last step delay used when accelrating.
  static int last_accel_delay;
  // Counting steps when moving.
  static unsigned int step_count = 0;
  // Keep track of remainder from new_step-delay calculation to incrase accurancy
  static unsigned int rest = 0;

  OCR1A = spool1.step_delay;

  switch(spool1.run_state) {
    case STOP:
      step_count = 0;
      rest = 0;
      // Stop Timer/Counter 1.
      TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));
      status_state.running = FALSE;
      break;

    case ACCEL:
      digitalWrite(PIND3, HIGH);
      digitalWrite(PIND3, LOW);
      spool1.accel_count++;
      new_step_delay = spool1.step_delay - (((2 * (long)spool1.step_delay) + rest)/(4 * spool1.accel_count + 1));
      rest = ((2 * (long)spool1.step_delay)+rest)%(4 * spool1.accel_count + 1);
      if(new_step_delay <= spool1.min_delay) {
        last_accel_delay = new_step_delay;
        new_step_delay = spool1.min_delay;
        rest = 0;
        spool1.run_state = RUN;
        //Serial.println("New run");
      }
      break;

    case RUN:
      digitalWrite(PIND3, HIGH);
      digitalWrite(PIND3, LOW);
      new_step_delay = spool1.min_delay;
      
      break;

    case DECEL:
      digitalWrite(PIND3, HIGH);
      digitalWrite(PIND3, LOW);
      
      spool1.accel_count++;
      new_step_delay = spool1.step_delay - (((2 * (long)spool1.step_delay) + rest)/(4 * spool1.accel_count + 1));
      rest = ((2 * (long)spool1.step_delay)+rest)%(4 * spool1.accel_count + 1);
      // Check if we at last step
      if(spool1.accel_count >= 0){
        spool1.run_state = STOP;
      }
      break;
  }
  spool1.step_delay = new_step_delay;
  //Serial.print("ISR step delay ");
  //Serial.print(spool.step_delay);
}

ISR(TIMER2_COMPA_vect)
{
  //Serial.print("ISR run state ");
  //Serial.print(spool.run_state);
  // Holds next delay period.
  unsigned int new_step_delay;
  // Remember the last step delay used when accelrating.
  static int last_accel_delay;
  // Counting steps when moving.
  static unsigned int step_count = 0;
  // Keep track of remainder from new_step-delay calculation to incrase accurancy
  static unsigned int rest = 0;

  OCR2A = winder1.step_delay;

  switch(winder1.run_state) {
    case STOP:
      step_count = 0;
      rest = 0;
      // Stop Timer/Counter 1.
      TCCR2B &= ~((1<<CS22)|(1<<CS21)|(1<<CS20));
      status_state.running = FALSE;
      break;

    case ACCEL:
      digitalWrite(PIND5, HIGH);
      digitalWrite(PIND5, LOW);
      winder1.accel_count++;
      new_step_delay = winder1.step_delay - (((2 * (long)winder1.step_delay) + rest)/(4 * winder1.accel_count + 1));
      rest = ((2 * (long)winder1.step_delay)+rest)%(4 * winder1.accel_count + 1);
      if(new_step_delay <= winder1.min_delay) {
        last_accel_delay = new_step_delay;
        new_step_delay = winder1.min_delay;
        rest = 0;
        winder1.run_state = RUN;
        //Serial.println("New run");
      }
      break;

    case RUN:
      digitalWrite(PIND5, HIGH);
      digitalWrite(PIND5, LOW);
      new_step_delay = winder1.min_delay;
      
      break;

    case DECEL:
      digitalWrite(PIND5, HIGH);
      digitalWrite(PIND5, LOW);
      
      winder1.accel_count++;
      new_step_delay = winder1.step_delay - (((2 * (long)winder1.step_delay) + rest)/(4 * winder1.accel_count + 1));
      rest = ((2 * (long)winder1.step_delay)+rest)%(4 * winder1.accel_count + 1);
      // Check if we at last step
      if(winder1.accel_count >= 0){
        winder1.run_state = STOP;
      }
      break;
  }
  winder1.step_delay = new_step_delay;
  //Serial.print("ISR step delay ");
  //Serial.print(spool.step_delay);
}
/*! \brief Square root routine.
 *
 * sqrt routine 'grupe', from comp.sys.ibm.pc.programmer
 * Subject: Summary: SQRT(int) algorithm (with profiling)
 *    From: warwick@cs.uq.oz.au (Warwick Allison)
 *    Date: Tue Oct 8 09:16:35 1991
 *
 *  \param x  Value to find square root of.
 *  \return  Square root of x.
 */
static unsigned long sqrt_calc(unsigned long x)
{
  register unsigned long xr;  // result register
  register unsigned long q2;  // scan-bit register
  register unsigned char f;   // flag (one bit)

  xr = 0;                     // clear result
  q2 = 0x40000000L;           // higest possible result bit
  do
  {
    if((xr + q2) <= x)
    {
      x -= xr + q2;
      f = 1;                  // set flag
    }
    else{
      f = 0;                  // clear flag
    }
    xr >>= 1;
    if(f){
      xr += q2;               // test flag
    }
  } while(q2 >>= 2);          // shift twice
  if(xr < x){
    return xr +1;             // add for rounding
  }
  else{
    return xr;
  }
}

/*! \brief Find minimum value.
 *
 *  Returns the smallest value.
 *
 *  \return  Min(x,y).
 */
unsigned int min_calc(unsigned int x, unsigned int y)
{
  if(x < y){
    return x;
  }
  else{
    return y;
  }
}
