#include "speed_cntr.h"

#define POT A0
#define BETA 0.1

struct stepper
{
   uint16_t w; //angular velocity in rev/s
   uint16_t L; //winding length, mm
   uint16_t V; //linear speed of winding, mm/s
   uint8_t d; //in mm*10 0.1mm = 1
   uint8_t STEPS; //steps per revolution
   uint16_t w_min; //minimum angular velocity
   uint16_t w_max; //maximum angular velocity
   uint8_t MS1;
   uint8_t MS2;
   uint8_t STEP;
   uint8_t DIR;
   uint16_t FREQ = 0;
   uint8_t mode;
};

stepper spool;
stepper winder;

int previousPotPosition = 0; //previous position of the lever
uint32_t previousTime = 0;


enum{
    NEUTRAL = 0,
    WIND,
    UNDWIND
};

void stepperInit (stepper &motor, uint8_t steps, uint16_t w_min, uint16_t w_max, uint8_t MS1, uint8_t MS2, uint8_t DIR, uint8_t STEP) {
    motor.STEPS = steps;
    motor.w_min = w_min;
    motor.w_max = w_max;
    motor.MS1 = MS1;
    motor.MS2 = MS2;
    motor.DIR = DIR;
    motor.STEP = STEP;
    pinMode(motor.MS1, OUTPUT);
	pinMode(motor.MS2, OUTPUT);
    pinMode(motor.DIR, OUTPUT);
    pinMode(motor.STEP, OUTPUT);

    digitalWrite(motor.MS1, HIGH); //quater steps 1/4th
	digitalWrite(motor.MS2, LOW);
    digitalWrite(motor.DIR, HIGH);
    digitalWrite(motor.STEP, LOW);
    
}

float value = 400;

void setup()
{
    Serial.begin(9600);
	stepperInit(spool, 200, 0, 60, 10, 11, 2, 3);
    stepperInit(winder, 200, 0, 60, 7, 6, 4, 5);
	
    pinMode(POT, INPUT);
        
    /*setup Timer0 - 16bit*/ //main Winding
    spoolTimer1Init();
    winderTimer2Init ();
    Serial.println(M1_ALPHA);
    Serial.println(F_CPU);
    Serial.println(T1_PRESCALER);
    
}

void loop()
{   
    int potPosition = analogRead(POT);
    value = value - BETA*((int)value - potPosition);
    
    if(abs(value-previousPotPosition) > 5)    {
        int speed = map(value, 0, 1024, 10, 700);
        setSpeed(speed, 50);
        winderSetSpeed(speed/32, 50);
        previousPotPosition = value;
        Serial.print("value =");
        Serial.println(value);
        Serial.print("speed rpm =");
        Serial.println(speed);
    }
    
/*
    value = value - BETA*(value - potPosition);     
    
        if (value <= 410 && value >= 390)   {
            spool.mode = NEUTRAL;
            stop_running();
        }
            
        else if (value < 390) {
            digitalWrite(spool.DIR, HIGH);
            spool.mode = WIND;
            uint16_t speed = map(value, 300, 390, 6000, 500);
            setSpeed(speed, 10000);
            winderSetSpeed(speed, 1000);
        }
        else if (value > 410){
            spool.mode = UNDWIND;
            digitalWrite(spool.DIR, LOW);
            uint16_t speed = map(value, 410, 500, 500, 6000);
            setSpeed(speed, 10000);
            winderSetSpeed(speed, 1000);

        }
        
        //Serial.println("New prev");
        previousPotPosition = potPosition;
        //Serial.println(previousPotPosition);
        
*/        
     
    
    
   //Serial.println(potPosition);
    /*
    if(millis() - previousTime > 1000)
    {
        Serial.println(potPosition);
        //Serial.println(winding.FREQ);
        previousTime = millis();
    }
    
	*/
	
}



/*
ISR(TIMER1_COMPA_vect) {

   

    PORTD = PORTD |= 0b00001000;
    PORTD = PORTD &= 0b11110111;
    //digitalWrite(spool.STEP, LOW);
        
}


ISR(TIMER2_COMPA_vect) {

    PORTD = PORTD |= 0b00100000;
    PORTD = PORTD &= 0b11011111;
    //digitalWrite(winder.STEP, HIGH);
    //digitalWrite(winder.STEP, LOW);

        
}
*/