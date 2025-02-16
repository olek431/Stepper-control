/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Header file for speed_cntr.c.
 *
 * - File:               speed_cntr.h
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
 * $RCSfile: speed_cntr.h,v $
 * $Date: 2006/05/08 12:25:58 $
 *****************************************************************************/

#ifndef SPEED_CNTR_H
#define SPEED_CNTR_H

#include "Arduino.h"

/*! \brief Holding data used by timer interrupt for speed ramp calculation.
 *
 *  Contains data used by timer interrupt to calculate speed profile.
 *  Data is written to it by move(), when stepper motor is moving (timer
 *  interrupt running) data is read/updated when calculating a new step_delay
 */
void stop_running(void );
void winder_stop_running(void );
void setSpeed(unsigned int, unsigned int );
void winderSetSpeed(unsigned , unsigned );


struct speedRampData{
  //! What part of the speed ramp we are in.
  volatile unsigned char run_state;
  //! Direction stepper motor should move.
  volatile unsigned char dir;
  //! Peroid of next timer delay. At start this value set the accelration rate.
  volatile unsigned int step_delay = 10;
  //! What step_pos to start decelaration
  volatile unsigned int decel_start;
  //! Sets deceleration rate.
  volatile signed int decel_val;
  //! Minimum time delay (max speed)
  volatile signed int min_delay;
  //! Counter used when accelerateing/decelerateing to calculate step_delay.
  volatile signed int accel_count;
};


/*! \Brief Frequency of timer1 in [Hz].
 *
 * Modify this according to frequency used. Because of the prescaler setting,
 * the timer1 frequency is the clock frequency divided by 8.
 */
// Timer/Counter 1 running on 16MHz / 64 = 250kHz (4uS). (T1-FREQ 250000)
#define T1_FREQ 250000
#define HALFSTEPS 1
//! Number of (full)steps per round on stepper motor in use.
#define FSPR 200
#ifdef HALFSTEPS
  #define SPR (FSPR*2)  
#endif
#ifdef FULLSTEPS
  #define SPR FSPR
  
#endif

#define T2_FREQ 62500 //16MHz / 1024 = 15625kHz
#define T2_HALFSTEPS 1
//! Number of (full)steps per round on stepper motor in use.
#define T2_FSPR 200
#ifdef T2_HALFSTEPS
  #define T2_SPR (T2_FSPR*2)  
#endif
#ifdef T2_FULLSTEPS
  #define T2_SPR T2_FSPR
  
#endif

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000

// Maths constants for Timer2 - Winder. To simplify maths when calculating in speed_cntr_Move().
#define T2_ALPHA (2*3.14159/T2_SPR)                    // 2*pi/spr
#define T2_A_T_x100 ((long)(T2_ALPHA*T2_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T2_FREQ_148 ((int)((T2_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define T2_A_SQ (long)(T2_ALPHA*2*10000000000)         // ALPHA*2*10000000000
#define T2_A_x20000 (int)(T2_ALPHA*20000)              // ALPHA*20000

#define LPR 8 //ball screw length per rotation
#define SPOOL_LENGTH 100
#define SPOOL_DIA 100
#define FIBER_DIA 0.25

// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

// Direction of stepper motor movement
#define CW  0
#define CCW 1

#define TRUE 1
#define FALSE 0

/*! \brief Status flags
 */
struct GLOBAL_FLAGS {
  //! True when stepper motor is running.
  unsigned char running;
  //! True when uart has received a string (ended with '/r').
  unsigned char cmd;
  //! Dummy bits to fill up a byte.
  unsigned char dummy;
};


void spoolTimer1Init(void);
void winderTimer2Init(void);
static unsigned long sqrt_calc(unsigned long v);
unsigned int min_calc(unsigned int x, unsigned int y);

//! Global status flags


#endif
