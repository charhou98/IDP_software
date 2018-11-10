int ledR = 26;
int ledY = 27;
int ldr0 = A0;
int intensity = 0;
int red_intensity = 0;
int yel_intensity = 0;
int movingavg = 1000;

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
  digitalWrite(ledY, HIGH);
  intensity = analogRead(ldr0);
  movingavg = movingavg*0.95 + intensity*0.05;
  //Serial.println(intensity);
  if (intensity > movingavg*1.1)
  {
    digitalWrite(ledR, LOW);
    digitalWrite(ledY, LOW);
    digitalWrite(ledR, HIGH);
    delay(20);
    red_intensity = analogRead(ldr0);
    digitalWrite(ledR, LOW);
    digitalWrite(ledY, HIGH);
    delay(20);
    yel_intensity = analogRead(ldr0);
    digitalWrite(ledY, LOW);
    if (red_intensity > yel_intensity*1.01)
    {
      Serial.println("Red");
    }
    else if (red_intensity < yel_intensity*0.995)
    {
      Serial.println("Yellow");
    }
    else
    {
      Serial.println("White");
    }
  }
  else
  {
  //  Serial.println("Black");
  }
  delay(20);
}

                  
