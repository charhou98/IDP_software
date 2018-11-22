#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

#define ledY 6  // yellow LEDs 
#define ledR 7 //red LEDs
#define IRline 41 // infrared pin
#define echo1 23
#define trig1 25
#define echo2 35
#define trig2 37
#define pushb 39
#define R_mine_indc 45
#define Y_mine_indc 47
#define dist1_indc 49
#define dist2_indc 51
#define mag_indc 53
const uint8_t ldr_pins[8] = {A0, A1, A2, A3, A12, A13, A14, A15};
int order;

// Global Variables
int moter1 = 0;
int moter2 = 0;
int servo_pos = 0;
long u_dist1, u_dist2;
long heading;
int ldr_avg = 0;
uint8_t mine_pos[8] = {0,0,0,0,0,0,0,0};
const float ldr_calibration[8] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

// Assign a unique ID to magnetometer
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// Assign a unique ID to accelerometer
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

Servo scoop_servo;  // create servo object to control a servo



//Functions to use:

// Holds up code until push button is pressed (pushb pin high)
void wait_for_push()
{
  const uint8_t indc_pins[] = {R_mine_indc, Y_mine_indc, dist1_indc, dist2_indc, mag_indc};
  int led = 0;
  bool up = true;
  while (digitalRead(pushb) == LOW) {
    // Make pretty patterns while waiting
    digitalWrite(indc_pins[led], HIGH);
    delay(100);
    digitalWrite(indc_pins[led], LOW);
    
    if (led == 4)
      up = false;
    else if (led == 0)
      up = true;
    
    if (up == true)
      led++;
    else
      led--;
  }
  for (int i = 0; i <= 4; i++) {
    digitalWrite(indc_pins[led], LOW);
  }
  
}


// Takes two values -255 to 255 and sets correct motor directions/speeds
void motor_shield(int m1speed, int m2speed)
{
  if (m1speed >= 0){
    motor1->run(FORWARD);
  }else{
    motor1->run(BACKWARD);
  }
  
  if (m2speed >= 0){
    motor2->run(FORWARD);
  }else{
    motor2->run(BACKWARD);
  }

  motor1->setSpeed(abs(m1speed));
  motor2->setSpeed(abs(m2speed));
}


// Check for line under rear scoop
void IR_line_sensor() 
{
  bool no_line;
  no_line = digitalRead(IRline);
  if (no_line == true)
  {
    Serial.println("No Line");
  }
  else
  {
    Serial.println("Line");
  }
  
 }


// Update global u_dist (ultrasonic distance) variables
void ultrasonic_sensor()
{ 
  long duration;
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  duration = pulseIn(echo1, HIGH);
  u_dist1 = (duration/2) / 29.1;

  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);
  duration = pulseIn(echo2, HIGH);
  u_dist2 = (duration/2) / 29.1;
  
  
  if (u_dist1 >= 300 || u_dist1 <= 0){
    //Serial.println("Forward facing ultrasonic sensor out of range");
    digitalWrite(dist1_indc, HIGH);
  }
  else {
    digitalWrite(dist1_indc, LOW);
  }
  
  if (u_dist2 >= 300 || u_dist2 <= 0){
    //Serial.println("Left facing ultrasonic sensor out of range");
    digitalWrite(dist2_indc, HIGH);
  }
    else {
    digitalWrite(dist2_indc, LOW);
  }
  
  //Serial.print("(Forward, Side) Distances: "); Serial.print((u_dist1)); Serial.print((", ")); Serial.println((u_dist2));
}
  

// Mine detection based on whether one LDR sees significantly higher intensity than average
void mine_detection()
{
  int ldr_values[8] = {0,0,0,0,0,0,0,0};
  int ldr_val = 0;
  int ldr_sum = 0;
  for (int i = 0; i <= 7; i++) {
    ldr_val = analogRead(ldr_pins[i]);
    ldr_val *= ldr_calibration[i];
    ldr_values[i] = ldr_val;
    ldr_sum += ldr_val;
    Serial.print(ldr_val); Serial.print(" ");
  }
  ldr_avg = ldr_sum / 8;
  Serial.println();
  Serial.println(ldr_avg);

  for (int i = 0; i <= 7; i++) {
    if (ldr_values[i] > ldr_avg*1.2) {
      mine_pos[i] = 1;   
    }
    else {
      mine_pos[i] = 0;
    }
  }

}


// Determine the colour under a single LDR
void colour_sensing(int ldr_num)
{
  int red_intensity;
  int yel_intensity;
  int red_total = 0;
  int yel_total = 0;
  int ldr = ldr_pins[ldr_num];
  for (int i = 0; i < 8; i++) {
    digitalWrite(ledY, LOW);
    digitalWrite(ledR, HIGH);
    delay(100);
    red_intensity = analogRead(ldr);
    Serial.print(red_intensity); Serial.print(" ");
    digitalWrite(ledR, LOW);
    digitalWrite(ledY, HIGH);
    delay(100);
    yel_intensity = analogRead(ldr);
    Serial.println(yel_intensity);

    red_total += red_intensity * ldr_calibration[ldr_num];
    yel_total += yel_intensity * ldr_calibration[ldr_num];
  }
  
  if (yel_total > 8*ldr_avg*2) {
    Serial.print("Yellow mine under ldr number "); Serial.println(ldr_num);
  }
  else if (yel_total <= 8*ldr_avg*1.8) {
    Serial.print("Red mine under ldr number "); Serial.println(ldr_num);
  }
  else {
    Serial.print("Unknown object under ldr number "); Serial.println(ldr_num);
  }
  digitalWrite(ledR, HIGH);
  digitalWrite(ledY, HIGH);
}


// Update the global variable heading based on magnetometer readings
void get_heading()
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


// Print accelerations over serial
void get_acceleration()
{
  // Get a new sensor event
  sensors_event_t event;
  accel.getEvent(&event);

  // Display the results (acceleration is measured in m/s^2)
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
}


// Print information required by software over serial  
void print_all()  
{ 
  Serial.print(u_dist1); Serial.print(" "); Serial.println(u_dist2);  
}



// Setup runs once
void setup()
{
  pinMode(ledR,OUTPUT);
  pinMode(ledY,OUTPUT);
  pinMode(IRline, INPUT);
  for (int i = A0; i <= A7; i++)
    pinMode(i, INPUT);
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(R_mine_indc, OUTPUT);
  pinMode(Y_mine_indc, OUTPUT);
  pinMode(dist1_indc, OUTPUT);
  pinMode(dist2_indc, OUTPUT);
  pinMode(mag_indc, OUTPUT);

  Serial.begin(9600);
#ifndef ESP8266
  while (!Serial);     // Will pause arduino until serial console opens
#endif

  // Enable magnetometer auto-gain
  mag.enableAutoRange(true);

  //Check for magnetometer/accelerometer
  if(!mag.begin())
  {
    // Print error messsage if magnetometer cannot be detected
    //Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    digitalWrite(mag_indc, HIGH);
    //while(1);
  }

  // Initialise the accelerometer
  if(!accel.begin())
  {
    // Print error messsage if accelerometer cannot be detected
    //Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    digitalWrite(mag_indc, HIGH);
    //while(1);
  }

  AFMS.begin(); // Initiate motor driver

  scoop_servo.attach(9); // Attaches scoop servo to pin 9

  //wait_for_push();
  digitalWrite(ledR, HIGH);
  digitalWrite(ledY, HIGH);
}



// Main body of the code:
// All functions listed for testing - comment out as required
void loop()
{
  while(Serial.available()){
    order = Serial.read();
  }
  if (order == '6'){
    moter1 = -105;
    moter2 = 105;
  }
  else if (order == '5'){
    moter1 = 105;
    moter2 = 105;
  }
  else if (order == '7'){
    moter1 = 0;
    moter2 = 0;
  }
//else if (order =='8'){
  //codes to turn on the yellow light}
//else if (order == '9'){
  //codes to turn on the red light
//}
  else if (order == '10'){
    moter1 = 110;
    moter2 = 100;
  }
  else if (order == '11'){
    moter1 = 100;
    moter2 = 110;
  }
  else if (order == '12'){
    moter1 = -105;
    moter2 = -105;
  }
    

  mine_detection();
  for (int i = 0; i <= 7; i++) { 
    if (mine_pos[i] == 1) {
      colour_sensing(i);
    }
  }
  
  ultrasonic_sensor();
  //get_acceleration();
  //get_heading();
  //Serial.println(correction);
  motor_shield(moter1, moter2);
  //scoop_servo.write(servo_pos);
  print_all();
  delay(200);
   
}
                  
