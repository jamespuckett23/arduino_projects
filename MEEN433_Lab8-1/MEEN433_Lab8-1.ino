// Initialize an integer to store the analog pin number
// In Arduino, A0 is an alias for the PIN_A0 number
int pot_pin = A0;

void setup() {
  Serial.begin(9600);
  // Set the potentiometer pin as an input
  pinMode(pot_pin,INPUT);
}

void loop() {
  // Read and display the analog value of the pot pin
  Serial.println(analogRead(pot_pin));
}
