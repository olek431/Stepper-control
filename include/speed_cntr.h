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
#define F_CPU 16000000L
#define T1_PRESCALER 64
#define HALFSTEPS 1
//! Number of (full)steps per round on stepper motor in use.
#define FSPR 200
#ifdef HALFSTEPS
  #define SPR (FSPR*2)  
#endif
#ifdef FULLSTEPS
  #define SPR FSPR
  
#endif

#define T2_PRESCALER 1024
#define T2_HALFSTEPS 1
//! Number of (full)steps per round on stepper motor in use.
#define T2_FSPR 200
#ifdef T2_HALFSTEPS
  #define T2_SPR (T2_FSPR*2)  
#endif
#ifdef T2_FULLSTEPS
  #define T2_SPR T2_FSPR
  
#endif

#define M1_ALPHA 1.8
#define M2_ALPHA 1.8

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
