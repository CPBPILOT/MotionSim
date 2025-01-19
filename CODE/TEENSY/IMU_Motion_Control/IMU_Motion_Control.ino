#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>

// Define the stepper motor connections
#define XDIR_PIN 3
#define XSTEP_PIN 2
#define YDIR_PIN 5
#define YSTEP_PIN 4
#define PASSTHROUGH_PIN 16
#define LED_PIN 6
#define NUM_LEDS 5


// Define the maximum speed and acceleration
#define MAX_SPEED 10000.0 // Steps per second
#define ACCELERATION 10000.0 // Steps per second^2

#define HISTORY_SIZE 10

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_MPU6050 mpu;
float pitch=0;
float roll=0;
float pitch_1=0;
float roll_1=0;
float init_start;
int init_end=0;
int step_size;
float MOTION_MULT = 100.0;

// Create a new instance of the AccelStepper class
AccelStepper x_stepper(AccelStepper::DRIVER, XSTEP_PIN, XDIR_PIN);
AccelStepper y_stepper(AccelStepper::DRIVER, YSTEP_PIN, YDIR_PIN);
// Variables to track target position and whether motor is moving
//long targetPosition = 0;
int state=0;
bool x_isMoving = false;
bool y_isMoving = false;

bool x_init=false;
bool y_init=false;

int x_zero=0;
int y_zero=-4;

int x_pos=0;
int y_pos=0;
int steps_per_rev=1600;

int roll_raw;
int pitch_raw;
int roll_scaled;
int pitch_scaled;
int x_pot_loc;
int y_pot_loc;

int minPos = -8000;
int maxPos = 8000;
long t_1=0;
int dt=10;
byte i=0;

int LED_ANI_COUNT = 0;
float num1 = 0.0;
float num2 = 0.0;
String inputString = "";
boolean stringComplete = false;


// Buffer to hold incoming data
float receivedData[3];
char incomingBuffer[50];
int bufferIndex = 0;

// Variables to keep track of the conversion status

void setup() {
  // Set the maximum speed and acceleration
  
  pinMode(XDIR_PIN,OUTPUT);
  pinMode(YDIR_PIN,OUTPUT);
  pinMode(XSTEP_PIN,OUTPUT);
  pinMode(YSTEP_PIN,OUTPUT);
  pinMode(PASSTHROUGH_PIN,INPUT_PULLUP);


  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  x_stepper.setMaxSpeed(MAX_SPEED);
  y_stepper.setMaxSpeed(MAX_SPEED);

  x_stepper.setAcceleration(ACCELERATION);
  y_stepper.setAcceleration(ACCELERATION);
  // Set the initial position to 0
  x_stepper.setCurrentPosition(0);
  y_stepper.setCurrentPosition(0);

  // Set the direction to clockwise
  x_stepper.setSpeed(MAX_SPEED/2);
  y_stepper.setSpeed(MAX_SPEED/2);

  Serial2.begin(9600, SERIAL_8N1);

  strip.begin();
  strip.show(); // Turn off all LEDs initially
}

void loop() {
  // Your main code logic here

if ((millis()-t_1)>dt){
  t_1=millis();

  getIMUData();
  

  x_stepper.run();
  y_stepper.run();
  
  if (state==0){
    if ((abs(roll-x_zero)<0.05) && (abs(pitch-y_zero)<0.05)){
      init_end=1;
    }
    
    if ((init_end==0) && (x_init==false)){
      step_size=abs(roll-x_zero)*10;
      step_size=min(step_size,10);
      if (((roll-x_zero)>0) && (x_isMoving==false)){
        x_pos-=step_size;
      }
      if (((roll-x_zero)<0) && (x_isMoving==false)){
        x_pos+=step_size;
      }
      moveToPosition(x_pos,y_pos);
      x_stepper.run();
      y_stepper.run();
      updateMovement();
      
    }
    else{
      x_init=true;
      x_stepper.setCurrentPosition(0);
      x_pos=0;
    }
    if ((init_end==0) && (y_init==false)){
      step_size=abs(pitch-y_zero)*10;
      step_size=min(step_size,10);
      if (((pitch-y_zero)>0) && (y_isMoving==false)){
        y_pos+=step_size;
      }
      if (((pitch-y_zero)<0) && (y_isMoving==false)){
        y_pos-=step_size;
      }
      moveToPosition(x_pos,y_pos);
      x_stepper.run();
      y_stepper.run();
      updateMovement();
      
    }

    
    else{
      y_init=true;
      y_stepper.setCurrentPosition(0);
      y_pos=0;
    }
    
    if (LED_ANI_COUNT>NUM_LEDS-1){
      LED_ANI_COUNT=0;
    }
    for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0)); // Blue: Red=0, Green=0, Blue=255
    }
    strip.setPixelColor(LED_ANI_COUNT, strip.Color(255, 255, 0)); // Blue: Red=0, Green=0, Blue=255
    strip.show();
    LED_ANI_COUNT++;
    if ((x_init==true) && (y_init==true)){
      state=2;
    }
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print("  ");
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("  ");
    Serial.print("x_pos: ");
    Serial.print(x_pos);
    Serial.print("  ");
    Serial.print("y_pos: ");
    Serial.print(y_pos);
    Serial.print("  ");
    Serial.print(init_end);
    Serial.println("");
  
    //updateMovement();
    
  }
  else if (state==1){
    
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(0, 0, 255)); // Blue: Red=0, Green=0, Blue=255
    }

  strip.show();
    while (Serial2.available()) {
    char incomingByte = Serial2.read();

    // Check for end-of-message marker
    if (incomingByte == '\n') {
      incomingBuffer[bufferIndex] = '\0'; // Null-terminate the string
      bufferIndex = 0;

      // Parse the incoming data
      sscanf(incomingBuffer, "%f,%f,%f", &receivedData[0], &receivedData[1], &receivedData[2]);

      // Print the received data for debugging
//      Serial.print("Received: ");
//      Serial.print(receivedData[0]);
//      Serial.print(", ");
//      Serial.print(receivedData[1]);
//      Serial.print(", ");
//      Serial.println(receivedData[2]);

      x_pos=map(receivedData[0],1000,2000,minPos,maxPos);
      y_pos=map(receivedData[1],2000,1000,minPos,maxPos);
    
    } else {
      // Append the byte to the buffer
      if (bufferIndex < sizeof(incomingBuffer) - 1) {
        incomingBuffer[bufferIndex++] = incomingByte;
      }
    }
  }
  


    if (digitalRead(PASSTHROUGH_PIN)==1){
      state=2;
    }
  }
  else if (state==2){
        for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 255, 0)); // Blue: Red=0, Green=0, Blue=255
  }
  strip.show();
  
    if (digitalRead(PASSTHROUGH_PIN)==0){
      state=1;
    }
    if (stringComplete) {
      parseVector(inputString);
      inputString = "";
      stringComplete = false;
      x_pos=num1*MOTION_MULT;
      y_pos=num2*MOTION_MULT;
    }

    
    // Read serial input and build the input string
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      inputString += inChar;
      // Check if the input string is complete (ends with a newline character)
      if (inChar == '\n') {
        stringComplete = true;
        break;
      }
    }
  }
  Serial.println(state);

    //Serial.print(" X: ");
    //Serial.print(x_pot_loc);
    //Serial.print(" Y: ");
    //Serial.print(y_pot_loc);
    //Serial.print(" XPos    ");
    //Serial.print(pitch_scaled);
    //Serial.print(" Ypos    ");
    //Serial.println(roll_scaled);
}

moveToPosition(x_pos,y_pos);
x_stepper.run();
y_stepper.run();
updateMovement();

}

void moveToPosition(long xposition,long yposition) {
  // Move to the specified position
  x_stepper.moveTo(xposition);
  y_stepper.moveTo(yposition);
  x_isMoving = true;
  y_isMoving = true;
}

void updateMovement() {
  // Check if the stepper motor has reached the target position
  if (x_isMoving && x_stepper.distanceToGo() == 0) {
    x_isMoving = false;
  }
  if (y_isMoving && y_stepper.distanceToGo() == 0) {
    y_isMoving = false;
  }
}

void parseVector(String input) {
  // Remove the newline character from the input string
  input.trim();
  
  // Find the comma separating the two numbers
  int commaIndex = input.indexOf(',');

  if (commaIndex > 0) {
    // Extract the first and second number as substrings
    String num1Str = input.substring(0, commaIndex);
    String num2Str = input.substring(commaIndex + 1);

    // Convert the substrings to floats
    num1 = num1Str.toFloat();
    num2 = num2Str.toFloat();

    // Print the numbers to the Serial Monitor for verification
    Serial.print("Received: ");
    Serial.print(num1);
    Serial.print(", ");
    Serial.println(num2);
  }
}

void getIMUData(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // Calculate pitch and roll
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll = atan2(ay, az) * 180.0 / PI;
  pitch=pitch*0.1+pitch_1*0.9;
  roll=roll*0.1+roll_1*0.9;
  pitch_1=pitch;
  roll_1=roll;
}
