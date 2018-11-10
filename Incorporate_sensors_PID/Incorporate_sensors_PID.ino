#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_MotorShield.h>
#include <PID_v1.h>

#define ledR 26 //red led
#define ledY 27  // yellow led 
#define IR_pin0 28 // infrared pin
#define ldr0 A0
#define trigPin 13
#define echoPin 12


// Assign a unique ID to magnetometer
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// Assign a unique ID to accelerometer
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

//Specify the variables and initial tuning parameters for PID control
double setpoint, heading, cont_heading, correction;
const int Kp=40, Ki=10, Kd=0;
PID myPID(&cont_heading, &correction, &setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);


// Global Variables
int intensity = 0;
int red_intensity = 0;
int yel_intensity = 0;
int movingavg = 1000;
int tot_speed = 0;



//Functions to use:

// Takes two values -255 to 255 and sets correct motor directions/speeds
void motor_shield(int m1speed, int m2speed)
{
  if (m1speed >= 0){
    motor1->run(FORWARD);
  }else{
    motor1->run(BACKWARD);
  }
  
  if (m2speed >= 0){
    motor1->run(FORWARD);
  }else{
    motor1->run(BACKWARD);
  }

  motor1->setSpeed(abs(m1speed));
  motor2->setSpeed(abs(m2speed));
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
      Serial.println("White"); // Don't trust yellow/white differentiation
    }
  }
  
}


void getheading()
{
  // Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);
  
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  heading = atan2(event.magnetic.y, event.magnetic.x);

  if(heading < 0)
    heading += 2*PI;
  
  Serial.print("Heading (radians): "); Serial.println(heading);
}

// Function to make heading continuous, i.e -PI to +3PI range 
void get_cont_heading(double setpoint)
{
  if (setpoint < PI && heading > setpoint + PI)
  {
    cont_heading = heading - 2*PI;
  }
  else if (setpoint > PI && heading < setpoint - PI)
  {
    cont_heading = heading + 2*PI;
  }
  else
  {
    cont_heading = heading;
  }
}

void acceleration()
{
  // Get a new sensor event
  sensors_event_t event;
  accel.getEvent(&event);

  // Display the results (acceleration is measured in m/s^2)
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
}


// Setup runs once
void setup()
{
  Serial.begin(9600);
#ifndef ESP8266
  while (!Serial);     // Will pause arduino until serial console opens
#endif

  /* Enable magnetometer auto-gain */
  mag.enableAutoRange(true);

  // Initialise the magnetometer
  if(!mag.begin())
  {
    // Print error messsage if magnetometer cannot be detected
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  // Initialise the accelerometer
  if(!accel.begin())
  {
    // Print error messsage if accelerometer cannot be detected
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  AFMS.begin(); // Initiate motor driver

  myPID.SetMode(AUTOMATIC); // Turn PID control on
  myPID.SetOutputLimits(-255, 255); // Set limits to motor speed range
  myPID.SetSampleTime(100); // Sample time in ms
  setpoint = 3.91;
  
  pinMode(ledR,OUTPUT);
  pinMode(ledY,OUTPUT);
  pinMode(IR_pin0, INPUT);
  pinMode(ldr0,INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}


// Main body of the code:
void loop()
{
 
 //color_sensing();
 //ultrasonic_sensor();
 //acceleration();
 getheading();
 get_cont_heading(setpoint);
 myPID.Compute();
 Serial.println(correction);
 motor_shield(tot_speed + correction, tot_speed - correction);
   
}
                  
