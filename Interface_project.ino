#include <Arduino.h>
#include <driver/ledc.h>
#include <Wire.h>
#include <MPU6050.h>

// --------------------------
// Motor Control Pin Definitions
// --------------------------
// Motor 1 (Left Motor)
#define MOTOR1_IN1 26
#define MOTOR1_IN2 27
#define MOTOR1_ENA 25  // PWM-capable GPIO pin

// Motor 2 (Right Motor)
#define MOTOR2_IN3 14
#define MOTOR2_IN4 12
#define MOTOR2_ENA 13  // PWM-capable GPIO pin

// --------------------------
// Encoder Pin Definitions
// --------------------------
// Encoder for Motor 1
#define ENCODER1_A 32
#define ENCODER1_B 33

// Encoder for Motor 2
#define ENCODER2_A 18
#define ENCODER2_B 19


MPU6050 mpu;

// Variables for yaw calculation
float yaw = 0;          // Yaw angle
unsigned long lastTime = 0; // For time tracking

// pid paramteres 
float Kp = 1.63;
float Ki = 0.01;
float Kd = 0.2;


// Variables for PID Control
volatile int encoder_left_count = 0; 
volatile int encoder_right_count = 0; 
float error, prev_error = 0, integral = 0, derivative, correction;
int Base_PWM = 150; 
float desiredYawAngle = 0;
float gyroBiasZ = 0;

// wheels specifications
const float wheel_diameter = 4.4; // Diameter of the wheel in cm
const int PPR = 210;              
const float wheel_circumference = 3.14159 * wheel_diameter; // circumference in cm

// --------------------------
// LEDC (PWM) Configuration Function
// --------------------------
void configureLEDC(uint8_t channel, uint8_t pin, uint32_t freq, uint8_t resolution) {
  // Configure LEDC timer (using the same timer for all channels)
  ledc_timer_config_t ledcTimer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = (ledc_timer_bit_t)resolution,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = freq,
    .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&ledcTimer);

  // Configure LEDC channel with proper channel casting
  ledc_channel_config_t ledcConfig = {
    .gpio_num = pin,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = (ledc_channel_t)channel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
  };
  ledc_channel_config(&ledcConfig);
}

// --------------------------
// Motor Speed Control Function
// --------------------------
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Constrain the speeds to the range -255 to 255
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Calculate absolute PWM duty values
  int leftDuty = abs(leftSpeed);
  int rightDuty = abs(rightSpeed);

  // Set PWM duty cycles (casting channel numbers to ledc_channel_t)
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)0, leftDuty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)1, rightDuty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)1);

}

void updateEncoderLeft() {
  encoder_left_count++;
}

void updateEncoderRight() {
  encoder_right_count++;
}
void initialize_motors(){
  // Set motors to move forward initially
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);
  setMotorSpeed(Base_PWM, Base_PWM);
}

void moveForward(float target_distance) {

  initialize_motors();

  // Calculate the target number of encoder pulses
  int target_pulses = (target_distance / wheel_circumference) * PPR;

  // Reset encoder counts
  encoder_left_count = 0;
  encoder_right_count = 0;

  while ((encoder_left_count + encoder_right_count) / 2 < target_pulses) {
   
    error = encoder_left_count - encoder_right_count;

    // PID calculations
    integral += error;                      
    derivative = error - prev_error;      
    correction =  Kp * error + Ki * integral + Kd * derivative;

    // Adjust motor PWM values
    int PWM_left = Base_PWM - correction;
    int PWM_right = Base_PWM + correction;

    // Constrain PWM values to range (0-255)
    PWM_left = constrain(PWM_left, 0, 255);
    PWM_right = constrain(PWM_right, 0, 255);

    // Set motor speeds
    setMotorSpeed(PWM_left, PWM_right);

    // Update previous error
    prev_error = error;

    // Debugging information
    Serial.print("Left Encoder: "); Serial.print(encoder_left_count);
    Serial.print(" | Right Encoder: "); Serial.print(encoder_right_count);
    Serial.print(" | Target Pulses: "); Serial.print(target_pulses);
    Serial.print(" | PWM Left: "); Serial.print(PWM_left);
    Serial.print(" | PWM Right: "); Serial.println(PWM_right);

    // Small delay for loop execution
    delay(10);
  }

  // Stop motors after reaching the target distance
  setMotorSpeed(0, 0);
}

void moveForward_MPU(float target_distance) {
  // Calculate the target number of encoder pulses
  initialize_motors();
  int target_pulses = (target_distance / wheel_circumference) * PPR;

  // Reset encoder counts
  encoder_left_count = 0;
  encoder_right_count = 0;

  // PID variables for Z-axis angle
  float error, integral = 0, derivative, prev_error = 0;
  float angle_correction;


  while ((encoder_left_count + encoder_right_count) / 2 < target_pulses) {
    // Measure the Z-axis tilt
    float currentYaw = readYaw();

    error = currentYaw - desiredYawAngle;
    if ( error <= 1 && error >= -1){
      error = 0;
    }
    
    // PID calculations for angle correction
    integral += error;
    derivative = error - prev_error;
    angle_correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Set motor speeds based on angle correction
    int PWM_left = Base_PWM + angle_correction;
    int PWM_right = Base_PWM - angle_correction;

    // Constrain PWM values to range (0-255)
    PWM_left = constrain(PWM_left, 0, 255);
    PWM_right = constrain(PWM_right, 0, 255);

    // Set motor speeds
    setMotorSpeed(PWM_left, PWM_right);

    // Update previous error
    prev_error = error;

    
    // Small delay for loop execution
    delay(10);
  }

  // Stop motors after reaching the target distance
  setMotorSpeed(0, 0);
}


void initializeMPU6050() {
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    return;
  }
  Serial.println("MPU6050 initialized successfully!");
}

float readYaw() {
  // Get raw gyroscope data
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Convert raw gyroscope data to degrees per second (°/s)
  float gyroZ = gz / 131.0; // 131 LSB/(°/s) for ±250 °/s range

  // Calculate time elapsed since last reading
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastTime) / 1000000.0; // Convert to seconds
  lastTime = currentTime;

  // Integrate gyroscope data to get yaw
  yaw += gyroZ * deltaTime; // gyroZ is in °/s, deltaTime is in seconds

  return yaw;
}


void stopMotors() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  setMotorSpeed(0, 0);
}

void turnRight(float targetAngle) {
    // Read the initial yaw angle
    float initialYaw = readYaw();
    float targetYaw = initialYaw - targetAngle; // Calculate the target angle
    float currentYaw = initialYaw;

    // Parameters for dynamic speed adjustment
    float correctionFactor = 0.1; // Adjust this value for better control
    int baseSpeed = 130; // Base speed for the motors
    float errorMargin = 3.0; // Margin of error for stopping

    while (abs(currentYaw - targetYaw) > errorMargin) {
        currentYaw = readYaw(); // Get the current yaw angle

        // Calculate the error and correction
        float error = targetYaw - currentYaw;
        float correction = error * correctionFactor;

        // Adjust motor speeds dynamically
        int leftSpeed = baseSpeed + correction;
        int rightSpeed = baseSpeed - correction;

        // Ensure speeds are within valid PWM range (0-255)
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);

        // Turn right by rotating the left motor forward and the right motor backward
        digitalWrite(MOTOR1_IN1, HIGH);
        digitalWrite(MOTOR1_IN2, LOW);
        digitalWrite(MOTOR2_IN3, LOW);
        digitalWrite(MOTOR2_IN4, HIGH);

        // Apply the adjusted speeds
        setMotorSpeed(leftSpeed, -rightSpeed);
    }

    // Stop the motors once the target angle is reached
    stopMotors();

    // Update the desired yaw angle for future turns
    desiredYawAngle -= currentYaw;
}

void turnLeft(float targetAngle) {
    // Read the initial yaw angle
    float initialYaw = readYaw();
    float targetYaw = initialYaw + targetAngle; // Calculate the target angle
    float currentYaw = initialYaw;

    // Parameters for dynamic speed adjustment
    float correctionFactor = 0.1; // Adjust this value for better control
    int baseSpeed = 130; // Base speed for the motors
    float errorMargin = 3.0; // Margin of error for stopping

    while (abs(currentYaw - targetYaw) > errorMargin) {
        currentYaw = readYaw(); // Get the current yaw angle


        // Calculate the error and correction
        float error = targetYaw - currentYaw;
        float correction = error * correctionFactor;

        // Adjust motor speeds dynamically
        int leftSpeed = baseSpeed + correction;
        int rightSpeed = baseSpeed - correction;

        // Ensure speeds are within valid PWM range (0-255)
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);

        // Turn right by rotating the left motor forward and the right motor backward
        digitalWrite(MOTOR1_IN1, LOW);
        digitalWrite(MOTOR1_IN2, HIGH);
        digitalWrite(MOTOR2_IN3, HIGH);
        digitalWrite(MOTOR2_IN4, LOW);

        // Apply the adjusted speeds
        setMotorSpeed(-leftSpeed, rightSpeed);
    }

    // Stop the motors once the target angle is reached
    stopMotors();

    // Update the desired yaw angle for future turns
    desiredYawAngle += currentYaw;
}

// --------------------------
// Setup
// --------------------------
void setup() {
  Serial.begin(9600);

  // Set motor control pins as outputs
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);

  // Configure LEDC channels for motor enable pins
  configureLEDC(0, MOTOR1_ENA, 5000, 8);  // Channel 0 for left motor
  configureLEDC(1, MOTOR2_ENA, 5000, 8);  // Channel 1 for right motor

  // Initialize encoder pins with internal pull-ups
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), updateEncoderRight, RISING);

  initializeMPU6050();

  delay(100);

  // simple simulation test  
  moveForward(60);
  delay(1000);
  turnRight(85);
  delay(1000);
  moveForward(20);
  delay(1000);
  turnRight(175);
  delay(1000);
  moveForward(20);
  delay(1000);
  turnLeft(85);
  delay(1000);
  moveForward(60);

}

void loop() {

}
