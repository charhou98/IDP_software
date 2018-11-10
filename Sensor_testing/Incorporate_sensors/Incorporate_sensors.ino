#include<Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_MotorShield.h>

#define ledR 26 //red led
#define ledY 27  // yellow led 
#define ldr0 A0
#define trigPin 13
#define echoPin 12
#define IR_pin0 28 //define infrared pin
#define pwm_a  3  //PWM control for motor outputs 1 and 2 
#define pwm_b  9  //PWM control for motor outputs 3 and 4 
#define dir_a  2  //direction control for motor outputs 1 and 2 
#define dir_b  8  //direction control for motor outputs 3 and 4

/* Assign a unique ID to magnetometer */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

/* Assign a unique ID to accelerometer */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

int intensity = 0;
int red_intensity = 0;
int yel_intensity = 0;
int movingavg = 1000;


void setup()
{
#ifndef ESP8266
  while (!Serial);     // will pause arduino until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("Magnetometer Test"); Serial.println("");

  /* Enable auto-gain */
  mag.enableAutoRange(true);

  /* Initialise the magnetometer */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  /* Initialise the accelerometer */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  AFMS.begin(); // Initiate motor driver
  
  pinMode(echoPin, INPUT);
  pinMode(ldr0,INPUT);
  pinMode(IR_pin0, INPUT);
  pinMode(ledR,OUTPUT);
  pinMode(ledY,OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);
  analogWrite(pwm_a, 100);  //set both motors to run at (100/255 = 39)% duty cycle (slow)
  analogWrite(pwm_b, 100);
  
}


//Functions to use:




void motor_shield()
{
  motor1->run(FORWARD);
  motor1->setSpeed(100);
  motor2->run(BACKWARD);
  motor2->setSpeed(200);
}



void IR_line() 
{
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
  
 }



void ultrasonic_sensor()
{ 
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  if (distance >= 300 || distance <= 0){
    Serial.println("Out of range");
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");
  }
}
  
 
void color_sensing()
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
    else if (red_intensity < yel_intensity)
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
  
}


void heading()
{
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);
  
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  if(heading < 0)
    heading += 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
}


void acceleration()
{
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
}



//Main body of the code:
void loop()
{
  
 color_sensing();
 ultrasonic_sensor();
 heading();
 acceleration();
 motor_shield();
 delay(1000);
  
 }
                  
