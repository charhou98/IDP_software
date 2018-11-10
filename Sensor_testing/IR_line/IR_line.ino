int IR_pin0 = 28;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  pinMode(IR_pin0, INPUT);
}

// the loop function runs over and over again forever
void loop() {
  bool no_line;
  no_line = digitalRead(IR_pin0);// wait for a second
  if (no_line == true)
  {
    Serial.println("No Line");
  }
  else
  {
    Serial.println("Line");
  }
  delay(100);
}
