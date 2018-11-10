int ledR = 26;
int ledY = 27;
int ldr0 = A0;
int red_intensity = 0;
int yel_intensity = 0;

void setup()
{
  Serial.begin(9600);          // opens serial port, sets data rate to 9600 bps
  pinMode(ldr0,INPUT);
  pinMode(ledR,OUTPUT);
  pinMode(ledY,OUTPUT);
}
void loop()
{
  digitalWrite(ledR, HIGH);
  delay(20);
  red_intensity = analogRead(ldr0);
  digitalWrite(ledR, LOW);
  digitalWrite(ledY, HIGH);
  delay(20);
  yel_intensity = analogRead(ldr0);
  digitalWrite(ledY, LOW);
  if (red_intensity > yel_intensity)
  {
    Serial.println("Red");
  }
   else if (red_intensity < yel_intensity)
  {
    Serial.println("Yellow");
  }  
}

                  
