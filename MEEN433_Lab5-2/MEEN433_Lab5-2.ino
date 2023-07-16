#define pin_a  2 
#define pin_b  3 
int last_a=0;
int last_b=0;
long counter=0;

void setup() {  
   pinMode(pin_a, INPUT);  
   pinMode(pin_b, INPUT);  
   //attachInterrupt(0, doEncoder_a, CHANGE);  
      // encoder pin on interrupt0-pin 2 
   //attachInterrupt(1, doEncoder, CHANGE);  
      // encoder pin on interrupt1-pin 3 
   Serial.begin (9600); 
   Serial.println("start"); 
}  
 
void loop() { 
  if (digitalRead(pin_a)) {    
      // Serial.println("A");
      if (last_a==0 and last_b==0) {
        // A then B
        Serial.println("Clockwise");
        counter += 1;
      }
      if ((last_a==0) and (last_b==1)) {
        Serial.println("Counterclockwise");
        counter -= 1;
      }
      if (last_a==1 and last_b==1) {
        // A then B
        Serial.println("Counterclockwise");
        counter -= 1;
      }
      last_a=1;
    } 
  if (digitalRead(pin_b)) {    
      // Serial.println("B");
      if (last_a==0 and last_b==0) {
        // B then A
        Serial.println("CounterClockwise");
        counter -= 1;
      }
      if (last_a==1 and last_b==0) {
        // B then A
        Serial.println("Clockwise");
        counter += 1;
      }
      if (last_a==1 and last_b==1) {
        Serial.println("Clockwise");
        counter += 1;
      }
      last_b=1;
  }
  if (not (digitalRead(pin_a)) and not (digitalRead(pin_b))) {
    if ((last_b == 1) and (last_a == 0)) {
      Serial.println("Clockwise");
      counter += 1;
    }
    if ((last_b == 0) and (last_a == 1)) {
      Serial.println("Counterclockwise");
      counter -= 1;
    }
  }
  if (not (digitalRead(pin_a))) {
    last_a=0;
  }
  if (not (digitalRead(pin_b))) {
    last_b=0;
  }
  
  Serial.println(counter);
}  

void doEncoder_a() { 
// your ISR to count and determine direction
//  if (digitalRead(pin_a)) { 
//    counter += 1;
//  }
//  if (digitalRead(pin_b)) { 
//    counter -= 1;
//  }
//  Serial.println(counter);
} 
