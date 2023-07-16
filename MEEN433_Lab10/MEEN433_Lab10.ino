#include "TimerThree.h" 
#include "TimerOne.h" 
#define pwm_pin 2 

int pot_signal; 
float duty_cycle; 
long cnt = 0; 
int b1, b2;  
int encoder_pin = 5; 
long v_current; 
double conversion_factor = 0.00528; 
 
void setup() { 
  Serial.begin(9600); 
  Timer1.initialize(200000); // interrupt triggered every 200 ms 
  Timer3.initialize(1000); // PWM carrier freq 1 kHz 
  Timer1.attachInterrupt(Timer1_Interrupt); 
  pinMode(encoder_pin,INPUT); 
  pinMode(A0,INPUT);
} 
 
void loop() { 
  if (digitalRead(encoder_pin)) 
      b1 = 1; 
    else 
      b1 = 0; 
    if (b1 != b2)  
      cnt++; 
    b2 = b1; 
} 
 
void Timer1_Interrupt() {  
  v_current = conversion_factor*cnt; 
  Serial.print("Speed of DC motor (rps) is: "); 
  Serial.println(v_current);  
  cnt = 0;  
} 

void Timer3_Interrupt() {
  pot_signal = analogRead(A0); 
  duty_cycle = pot_signal/1023.0; 
  analogWrite(pwm_pin, duty_cycle*255); // switch from duty_cycle*255 to 0/1
  Serial.print(duty_cycle*100); 
  Serial.println("%"); 
  }
