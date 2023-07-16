#define pin_a  2 
#define pin_b  3 

int last_a = 0;
int last_b = 0;
 
void setup() { 
  Serial.begin(9600); 
  pinMode(pin_a,INPUT); 
  pinMode(pin_b,INPUT); 
} 
 
void loop() { 
  if (digitalRead(pin_a)) {    
      Serial.println("A");
      if (last_a==0 and last_b==0) {
        Serial.println("A then B - Clockwise");
      }
      last_a=1;
    } 
  if (digitalRead(pin_b)) {    
      Serial.println("B");
      if (last_a==0 and last_b==0) {
        Serial.println("A then B - CounterClockwise");
      }
      last_b=1;
  }
  if (not (digitalRead(pin_a))) {
    last_a=0;
  }
  if (not (digitalRead(pin_b))) {
    last_b=0;
  }
} 
