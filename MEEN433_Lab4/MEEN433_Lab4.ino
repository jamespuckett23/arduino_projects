#define led_pin 3 
volatile unsigned long i; 
 
void setup() { 
  Serial.begin(9600); 
  pinMode(led_pin, OUTPUT); 
  // Create an interrupt service on interrupt 0 
  attachInterrupt(0, isr, FALLING); 
} 
 
void loop() { 
  Serial.println("..."); 
  delay(100); 
} 
 
void isr(){ 
  Serial.println("External Interrupt"); 
  digitalWrite(led_pin,HIGH); 
  for(i=0;i<500000;i++); // Give some delay 
  //delayMicroseconds(10000);  // another way to add delay 
  digitalWrite(led_pin,LOW); 
}
