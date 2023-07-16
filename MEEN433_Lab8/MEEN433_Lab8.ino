#define DQ 2 
#define CLK 3 
#define RST 4 
unsigned char Temp; 
unsigned char sign_bit; 
 
void setup() { 
   Serial.begin(9600); 
   pinMode(CLK,OUTPUT); 
   pinMode(RST,OUTPUT); 
 
} 
 
void loop() { 
   digitalWrite(RST,LOW); 
   digitalWrite(RST,HIGH); 
   PutByte(0xEE); 
   digitalWrite(RST,LOW); 
   delay(500); 
   digitalWrite(RST,HIGH); 
   PutByte(0xAA); 
   Temp = GetByte(); 
   sign_bit = GetByte(); 
   digitalWrite(RST,LOW); 
   Temp >>= 1; 
   Serial.print("Temp: "); 
   Serial.println(Temp); 
} 


unsigned char GetByte(void) { 
  int i; 
  int data; 
  int D; 
 
  D = 0; 
  for (i=0; i<=7; i++) { 
    digitalWrite(CLK,LOW); 
    digitalWrite(CLK,HIGH); 
    pinMode(DQ,INPUT); 
    data=digitalRead(DQ); 
    D = D + (data << i); 
  } 
  return D; 
} 
 
 
void PutByte(unsigned char d) { 
  int i; 
  pinMode(DQ,OUTPUT); 
  for(i=0; i<=7; i++) { 
    if(d%2 == 1) 
      digitalWrite(DQ,HIGH); 
    else 
      digitalWrite(DQ,LOW); 
    digitalWrite(CLK,HIGH); 
    digitalWrite(CLK,LOW); 
    d >>= 1; 
  } 
}
