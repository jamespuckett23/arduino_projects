// This is how we include non-standard libraries like Timer3 
#include "TimerThree.h" 
#define led_pin 3 
volatile unsigned long i; 
 
void setup() { 
  Serial.begin(9600); 
  pinMode(3,OUTPUT); 
  Timer3.initialize(5000000); // time before overflow in us 
  Timer3.attachInterrupt(isr); // isr triggered at overflow
  attachInterrupt(0, isr2, FALLING); 
} 
 
void loop(){  
} 
 
void isr(){ 
  Serial.println("T"); 
  digitalWrite(led_pin,HIGH); 
  for(i=0;i<228791;i++); // Give some delay 
  //delayMicroseconds(500000);  // another way to add delay 
  digitalWrite(led_pin,LOW); 
}

void isr2(){ 
  Serial.println("E"); 
  digitalWrite(led_pin,HIGH); 
  for(i=0;i<228791;i++); // Give some delay 
  //delayMicroseconds(500000);  // another way to add delay 
  digitalWrite(led_pin,LOW); 
}
