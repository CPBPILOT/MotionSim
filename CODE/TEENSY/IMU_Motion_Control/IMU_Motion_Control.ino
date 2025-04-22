#include <AccelStepper.h>           // Library for controlling stepper motors
#include <Adafruit_NeoPixel.h>      // Library for controlling NeoPixel LEDs

// Define the stepper motor connections for the X and Y axes
#define XDIR_PIN 3          // Direction pin for the X-axis stepper
#define XSTEP_PIN 2         // Step pin for the X-axis stepper
#define YDIR_PIN 5          // Direction pin for the Y-axis stepper
#define YSTEP_PIN 4         // Step pin for the Y-axis stepper
#define PASSTHROUGH_PIN 16  // Pin to check for pass-through input (for state changes)
#define LED_PIN 6           // Pin connected to the NeoPixel LED strip
#define NUM_LEDS 5          // Number of LEDs in the NeoPixel strip

// Define the maximum speed and acceleration for the stepper motors
#define MAX_SPEED 10000.0   // Maximum speed in steps per second
#define ACCELERATION 10000.0  // Maximum acceleration in steps per second^2

#define HISTORY_SIZE 10     // History size (not used in the current code)

#include <Adafruit_MPU6050.h>    // Library for handling the MPU6050 IMU (accelerometer/gyroscope)
#include <Adafruit_Sensor.h>     // Sensor library for Adafruit sensors
#include <Wire.h>                // I2C communication library

// Create an instance of the NeoPixel strip
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Create an instance of the MPU6050 IMU
Adafruit_MPU6050 mpu;

// Variables to store pitch and roll from the IMU
float pitch = 0;
float roll = 0;
float pitch_1 = 0;
float roll_1 = 0;
float init_start;           // Not used in the current code, placeholder for future use
int init_end = 0;           // Flag to indicate if initialization is complete
int step_size;              // Step size for motor movement
float MOTION_MULT = 100.0;  // Multiplier for motion calculation

// Create instances of AccelStepper for the X and Y axes
AccelStepper x_stepper(AccelStepper::DRIVER, XSTEP_PIN, XDIR_PIN);
AccelStepper y_stepper(AccelStepper::DRIVER, YSTEP_PIN, YDIR_PIN);

// Variables to track target position and motor movement status
int state = 0;             // Variable to track the current state
bool x_isMoving = false;   // Flag indicating if the X motor is moving
bool y_isMoving = false;   // Flag indicating if the Y motor is moving

bool x_init = false;       // Flag indicating if the X motor has been initialized
bool y_init = false;       // Flag indicating if the Y motor has been initialized

// Initial positions for X and Y axes (could be offsets from home position)
int x_zero = 0;
int y_zero = -4;

// Current position for X and Y axes
int x_pos = 0;
int y_pos = 0;
int steps_per_rev = 1600;   // Number of steps per motor revolution (not used in the current code)

int roll_raw;              // Raw roll value from IMU (not used)
int pitch_raw;             // Raw pitch value from IMU (not used)
int roll_scaled;           // Scaled roll value (not used)
int pitch_scaled;          // Scaled pitch value (not used)
int x_pot_loc;             // X position from potentiometer (not used)
int y_pot_loc;             // Y position from potentiometer (not used)

int minPos = -8000;        // Minimum position limit
int maxPos = 8000;         // Maximum position limit
long t_1 = 0;              // Timer for controlling loop timing
int dt = 10;               // Delay time (ms) between loop iterations
byte i = 0;                // Loop counter (not used)

int LED_ANI_COUNT = 0;     // LED animation counter
float num1 = 0.0;          // Variable to store parsed value for X position
float num2 = 0.0;          // Variable to store parsed value for Y position
String inputString = "";   // String to hold incoming serial data
boolean stringComplete = false;  // Flag to indicate if a complete string has been received

// Buffer to hold incoming serial data
float receivedData[3];     // Array to store incoming data (X, Y, Z from serial)
char incomingBuffer[50];   // Buffer to hold incoming data
int bufferIndex = 0;       // Index for tracking position in the incoming buffer

void setup() {
  // Set pin modes for the stepper motor control pins
  pinMode(XDIR_PIN, OUTPUT);
  pinMode(YDIR_PIN, OUTPUT);
  pinMode(XSTEP_PIN, OUTPUT);
  pinMode(YSTEP_PIN, OUTPUT);
  pinMode(PASSTHROUGH_PIN, INPUT_PULLUP);  // Initialize the pass-through pin as an input with pull-up resistor

  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for the serial port to connect (needed for native USB port)
  }

  // Initialize the MPU6050 sensor (IMU)
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10); // Infinite loop if MPU6050 is not found
    }
  }

  // Set MPU6050 sensor parameters
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  // Set accelerometer range to ±8g
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);       // Set gyroscope range to ±500°/sec
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Set filter bandwidth

  // Set the maximum speed and acceleration for the stepper motors
  x_stepper.setMaxSpeed(MAX_SPEED);
  y_stepper.setMaxSpeed(MAX_SPEED);
  x_stepper.setAcceleration(ACCELERATION);
  y_stepper.setAcceleration(ACCELERATION);

  // Set the initial position of the stepper motors to 0
  x_stepper.setCurrentPosition(0);
  y_stepper.setCurrentPosition(0);

  // Set the speed to half of the maximum speed
  x_stepper.setSpeed(MAX_SPEED / 2);
  y_stepper.setSpeed(MAX_SPEED / 2);

  // Initialize the second serial port (Serial2) for additional communication
  Serial2.begin(9600, SERIAL_8N1);

  // Initialize the NeoPixel LED strip
  strip.begin();
  strip.show();  // Turn off all LEDs initially
}

void loop() {
  // Main loop of the program, runs continuously

  if ((millis() - t_1) > dt) {
    t_1 = millis();  // Update timer

    // Get the data from the IMU (pitch and roll)
    getIMUData();

    // Update stepper motor positions
    x_stepper.run();
    y_stepper.run();

    // State machine logic to control the movement
    if (state == 0) {  // Initialization phase
      // Check if both pitch and roll are close to their "zero" positions
      if ((abs(roll - x_zero) < 0.05) && (abs(pitch - y_zero) < 0.05)) {
        init_end = 1;  // Mark initialization as complete
      }

      // Handle X-axis movement during initialization
      if ((init_end == 0) && (x_init == false)) {
        step_size = abs(roll - x_zero) * 10;
        step_size = min(step_size, 10);  // Limit the step size to 10
        if ((roll - x_zero) > 0 && (x_isMoving == false)) {
          x_pos -= step_size;  // Move X motor left
        }
        if ((roll - x_zero) < 0 && (x_isMoving == false)) {
          x_pos += step_size;  // Move X motor right
        }
        moveToPosition(x_pos, y_pos);
        x_stepper.run();
        y_stepper.run();
        updateMovement();
      } else {
        x_init = true;  // Mark X motor as initialized
        x_stepper.setCurrentPosition(0);  // Set current position to 0
        x_pos = 0;
      }

      // Handle Y-axis movement during initialization
      if ((init_end == 0) && (y_init == false)) {
        step_size = abs(pitch - y_zero) * 10;
        step_size = min(step_size, 10);  // Limit the step size to 10
        if ((pitch - y_zero) > 0 && (y_isMoving == false)) {
          y_pos += step_size;  // Move Y motor up
        }
        if ((pitch - y_zero) < 0 && (y_isMoving == false)) {
          y_pos -= step_size;  // Move Y motor down
        }
        moveToPosition(x_pos, y_pos);
        x_stepper.run();
        y_stepper.run();
        updateMovement();
      } else {
        y_init = true;  // Mark Y motor as initialized
        y_stepper.setCurrentPosition(0);  // Set current position to 0
        y_pos = 0;
      }

      // Handle LED animation and state transition
      if (LED_ANI_COUNT > NUM_LEDS - 1) {
        LED_ANI_COUNT = 0;  // Reset LED animation counter
      }
      for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0));  // Turn off all LEDs
      }
      strip.setPixelColor(LED_ANI_COUNT, strip.Color(255, 255, 0));  // Turn on one LED in yellow
      strip.show();
      LED_ANI_COUNT++;

      // Once both motors are initialized, change the state
      if ((x_init == true) && (y_init == true)) {
        state = 2;  // Transition to state 2 (active mode)
      }
      
      // Print diagnostic information
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

    }
    else if (state == 1) {  // State 1 (waiting for external data)
      // Set LED color to blue
      for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 255));  // Blue color for all LEDs
      }
      strip.show();

      // Check if there's incoming data from Serial2 and process it
      while (Serial2.available()) {
        char incomingByte = Serial2.read();

        // Check for end-of-message marker
        if (incomingByte == '\n') {
          incomingBuffer[bufferIndex] = '\0';  // Null-terminate the string
          bufferIndex = 0;

          // Parse the incoming data (X and Y positions)
          sscanf(incomingBuffer, "%f,%f,%f", &receivedData[0], &receivedData[1], &receivedData[2]);

          // Update positions based on received data
          x_pos = map(receivedData[0], 1000, 2000, minPos, maxPos);
          y_pos = map(receivedData[1], 2000, 1000, minPos, maxPos);
        } else {
          // Append the byte to the buffer if it's not the end-of-message marker
          if (bufferIndex < sizeof(incomingBuffer) - 1) {
            incomingBuffer[bufferIndex++] = incomingByte;
          }
        }
      }

      // Check the state of the pass-through pin to potentially change state
      if (digitalRead(PASSTHROUGH_PIN) == 1) {
        state = 2;  // Transition to state 2 if the pass-through pin is high
      }
    }
    else if (state == 2) {  // State 2 (active mode)
      // Set LED color to green
      for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(0, 255, 0));  // Green color for all LEDs
      }
      strip.show();

      // Check the state of the pass-through pin to potentially change state
      if (digitalRead(PASSTHROUGH_PIN) == 0) {
        state = 1;  // Transition back to state 1 if the pass-through pin is low
      }

      // Handle string input from the main serial port (for remote control)
      if (stringComplete) {
        parseVector(inputString);  // Parse the input string to extract positions
        inputString = "";  // Clear the input string
        stringComplete = false;  // Reset the flag
        x_pos = num1 * MOTION_MULT;  // Apply motion multiplier
        y_pos = num2 * MOTION_MULT;
      }

      // Read incoming serial data and build the input string
      while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;

        // Check if the input string is complete (ends with newline)
        if (inChar == '\n') {
          stringComplete = true;  // Mark the string as complete
          break;
        }
      }
    }
    
    // Print the current state for debugging purposes
    Serial.println(state);
  }

  // Move the stepper motors to the new position
  moveToPosition(x_pos, y_pos);
  x_stepper.run();
  y_stepper.run();

  // Update the movement status of the motors
  updateMovement();
}

// Function to move the stepper motors to the specified position
void moveToPosition(long xposition, long yposition) {
  x_stepper.moveTo(xposition);  // Move the X motor
  y_stepper.moveTo(yposition);  // Move the Y motor
  x_isMoving = true;  // Mark X motor as moving
  y_isMoving = true;  // Mark Y motor as moving
}

// Function to update the movement status of the motors
void updateMovement() {
  // Check if the X motor has reached its target position
  if (x_isMoving && x_stepper.distanceToGo() == 0) {
    x_isMoving = false;  // Mark X motor as stopped
  }
  // Check if the Y motor has reached its target position
  if (y_isMoving && y_stepper.distanceToGo() == 0) {
    y_isMoving = false;  // Mark Y motor as stopped
  }
}

// Function to parse the input string for X and Y coordinates
void parseVector(String input) {
  input.trim();  // Remove leading and trailing whitespace

  // Find the comma separating the two values
  int commaIndex = input.indexOf(',');

  if (commaIndex > 0) {
    // Extract the two numbers from the string
    String num1Str = input.substring(0, commaIndex);
    String num2Str = input.substring(commaIndex + 1);

    // Convert the strings to float values
    num1 = num1Str.toFloat();
    num2 = num2Str.toFloat();

    // Print the parsed values for debugging
    Serial.print("Received: ");
    Serial.print(num1);
    Serial.print(", ");
    Serial.println(num2);
  }
}

// Function to read the IMU (accelerometer/gyroscope) data
void getIMUData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  // Get accelerometer and gyroscope data

  float ax = a.acceleration.x;  // X acceleration
  float ay = a.acceleration.y;  // Y acceleration
  float az = a.acceleration.z;  // Z acceleration

  // Calculate pitch and roll from the accelerometer data
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll = atan2(ay, az) * 180.0 / PI;

  // Apply smoothing filter to pitch and roll
  pitch = pitch * 0.1 + pitch_1 * 0.9;
  roll = roll * 0.1 + roll_1 * 0.9;

  // Store the current values for the next iteration
  pitch_1 = pitch;
  roll_1 = roll;
}
