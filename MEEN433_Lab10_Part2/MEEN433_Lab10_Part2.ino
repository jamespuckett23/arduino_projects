#include "TimerThree.h" 
#include "TimerOne.h" 
#define pwm_pin 2 

int pot_signal; 
float duty_cycle; 
long cnt = 0; 
int b1, b2;  
int encoder_pin = 5; 
int v_current; 
double conversion_factor = 0.00528; 
int r = 50; // set speed
int e;
int e1 = 0;
int u;
int u1 = 0;
 
void setup() { 
  Serial.begin(9600); 
  Timer1.initialize(200000); // interrupt triggered every 200 ms 
  Timer3.initialize(1000); // PWM carrier freq 1 kHz 
  Timer1.attachInterrupt(Timer1_Interrupt); 
  pinMode(encoder_pin,INPUT); 
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
  e = r-v_current;
  u = u1 + 2*e - e1;
  u1 = u;
  e1 = e;
  Serial.println(e);
} 

void Timer3_Interrupt() {
  // pot_signal = analogRead(A0); 
  // duty_cycle = pot_signal/1023.0; 
  analogWrite(pwm_pin, u); // switch from duty_cycle*255 to 0/1
  Serial.print(u); 
  Serial.println("%"); 
  }
