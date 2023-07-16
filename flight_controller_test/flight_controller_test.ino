// ---------------------------------------------------------------------------
#include <Servo.h>
#include <HCSR04.h>
// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH      1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH      2000 // Maximum pulse length in µs
#define MIN_FLIGHT_CONTROLLER 0 // Minimum input to the flight controller
#define MAX_FLIGHT_CONTROLLER 255 // Maximum input to the flight controller
#define MOT1                  5
#define MOT2                  6
#define MOT3                  9
#define MOT4                  10
#define FREQ                  250   // Sampling frequency
// ---------------------------------------------------------------------------
Servo MotA, MotB, MotC, MotD;
float data;
float v1 = 0;
float e1 = 0;
float v;
float ref = 5; // cm
float u;
float e;
float gain = 0.1; // proportional gain constant
float control;
unsigned int  period; // Sampling period
unsigned long loop_timer;
unsigned long now, difference;

const byte MOTOR_PIN[4]; // extern
byte MOTOR_SPEED[4]; // extern

unsigned long pulse_length_esc1 = 1000,
      pulse_length_esc2 = 1000,
      pulse_length_esc3 = 1000,
      pulse_length_esc4 = 1000;
// ---------------------------------------------------------------------------




/**
 * Initialisation routine
 */

HCSR04 hc(3, 2); // initialization class HCSR04 (trig pin , echo pin)
float distanceController();
void applyMotorSpeed();
void keepSafe();
void MOTOR_CONTROL();

// void MOTOR_SET(){
//   analogWrite(5,MOTOR_SPEED[0]);
//   analogWrite(6,MOTOR_SPEED[1]);
//   analogWrite(9,MOTOR_SPEED[2]);
//   analogWrite(10,MOTOR_SPEED[3]);
// }

void setup() {
    Serial.begin(9600);
    
    // pinMode(5,OUTPUT);
    // pinMode(6,OUTPUT);
    // pinMode(9,OUTPUT);
    // pinMode(10,OUTPUT);

    MotA.attach(MOT1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    MotB.attach(MOT2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    MotC.attach(MOT3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    MotD.attach(MOT4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

    // MotA.attach(MOT1);
    // MotB.attach(MOT2);
    // MotC.attach(MOT3);
    // MotD.attach(MOT4);
    
    // // Set pins #4 #5 #6 #7 as outputs
    //DDRD |= B11110000;

    // period = (1000000/FREQ) ; // Sampling period in µs

    // Initialize loop_timer
    // loop_timer = micros();    

    displayInstructions();
}

/**
 * Main function
 */
void loop() {
  
    if (Serial.available()) {
        data = Serial.parseFloat();
        control = map(data, 0, 9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
        Serial.println(control);

        // analogWrite(5,min(max(control,0),MAX_PULSE_LENGTH));
        // analogWrite(6,min(max(control,0),MAX_PULSE_LENGTH));
        // analogWrite(9,min(max(control,0),MAX_PULSE_LENGTH));
        // analogWrite(10,min(max(control,0),MAX_PULSE_LENGTH));

    MotA.writeMicroseconds(control); // Roll angle
    MotB.writeMicroseconds(control); // Pitch angle
    MotC.writeMicroseconds(control); // Throttle
    MotD.writeMicroseconds(control); // Yaw angle

        // void keepSafe();      
    }
    // applyMotorSpeed();   

    // analogWrite(5,min(max(control,0),MAX_PULSE_LENGTH));
    // analogWrite(6,min(max(control,0),MAX_PULSE_LENGTH));
    // analogWrite(9,min(max(control,0),MAX_PULSE_LENGTH));
    // analogWrite(10,min(max(control,0),MAX_PULSE_LENGTH));

    Serial.print("The drone height is: "); Serial.println(hc.dist());

    MOTOR_CONTROL();
    // MotA.writeMicroseconds(control); // Roll angle
    // MotB.writeMicroseconds(control); // Pitch angle
    // MotC.writeMicroseconds(control); // Throttle
    // MotD.writeMicroseconds(control); // Yaw angle
  // control = distanceController();
  // Serial.println(control);

  // MotA.writeMicroseconds(control); // Roll angle
  // MotB.writeMicroseconds(control); // Pitch angle
  // MotC.writeMicroseconds(control); // Throttle
  // MotD.writeMicroseconds(control); // Yaw angle


  //MOTOR_CONTROL();


  // analogWrite(MOT1, control);
  // analogWrite(MOT2, control);
  // analogWrite(MOT3, control);
  // analogWrite(MOT4, control);
  // Serial.println(hc.dist()); //return current distance (cm) in serial
  // delay(60);                   // we suggest to use over 60ms measurement cycle, in order to prevent trigger signal to the echo signal.


}

float distanceController() { // difference controller using a proportional gain
  u = hc.dist(); // returns distance from ground in cm
  e = ref - u; // calculates the error from the reference distance
  v = v1 + gain*(2*e-e1); // calculates the voltage output to correct for the error

  // get ready for the next time step
  e1 = e;
  v1 = v;
  return v;
}

void MOTOR_CONTROL() {
  v = distanceController();

  Serial.print("v is: "); Serial.println(v);

  if (v > 9) {
    v = 2;
  }
  if (v < 0) {
    v = 0;
  }

  float new_control = map(v, 0, 9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

  MotA.writeMicroseconds(new_control); // Roll angle
  MotB.writeMicroseconds(new_control); // Pitch angle
  MotC.writeMicroseconds(new_control); // Throttle
  MotD.writeMicroseconds(new_control); // Yaw angle

  // analogWrite(5,min(max(v,0),MAX_PULSE_LENGTH));
  // analogWrite(6,min(max(v,0),MAX_PULSE_LENGTH));
  // analogWrite(9,min(max(v,0),MAX_PULSE_LENGTH));
  // analogWrite(10,min(max(v,0),MAX_PULSE_LENGTH));
  
}

void applyMotorSpeed() {
    // Refresh rate is 250Hz: send ESC pulses every 4000µs
    while ((now = micros()) - loop_timer < period);

    // Update loop timer
    loop_timer = now;

    // Set pins #4 #5 #6 #7 HIGH
    PORTD |= B11110000;

    // Wait until all pins #4 #5 #6 #7 are LOW
    while (PORTD >= 16) {
        now        = micros();
        difference = now - loop_timer;

        if (difference >= pulse_length_esc1) PORTD &= B11101111; // Set pin #4 LOW
        if (difference >= pulse_length_esc2) PORTD &= B11011111; // Set pin #5 LOW
        if (difference >= pulse_length_esc3) PORTD &= B10111111; // Set pin #6 LOW
        if (difference >= pulse_length_esc4) PORTD &= B01111111; // Set pin #7 LOW
    }
}

void keepSafe() {
    // Prevent out-of-range-values
    pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 2000);
    pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 2000);
    pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 2000);
    pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 2000);
}    

float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}


/**
 * Displays instructions to user
 */
void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("Send a number between 0 and 9");
    Serial.println("Min: 0");
    Serial.println("Max: 9");
}
