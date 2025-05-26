#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // Default address

#define SERVO_CHANNEL_LEFT 8       // right Servo connected to channel 8 on PCA9685
#define SERVO_CHANNEL_RIGHT 9      // left Servo connected to channel 9 on PCA9685

#define SERVO_RIGHT_CENTER 1430
#define SERVO_LEFT_CENTER 1500

// #define SERVO_OFFSET 300

// #define SERVO_LEFT_POS  (SERVO_LEFT_CENTER + SERVO_OFFSET)   // e.g., 1800
// #define SERVO_RIGHT_POS (SERVO_RIGHT_CENTER - SERVO_OFFSET)  // e.g., 1200

#define BUTTON_PIN 32

// IR sensor and tray sensor
#define IR_SENSOR_PIN 34
#define TRAY_IR_SENSOR_PIN 35

#define SERVO_RIGHT_TARGET 1030
#define SERVO_LEFT_TARGET 1750

#define STEP_DELAY 400
#define STEP_SIZE 170

#define OFFSET 60               // Vibration range (microseconds)
#define STEP_DELAY_VIBE 15      // Delay between steps (ms)
#define STEP_SIZE_VIBE 35       // Smoothness (smaller = smoother)
#define VIBRATION_DURATION 5000  // Vibration duration in ms

#define WAIT_TIME_AFTER_FALL 10000

int currentLeft = SERVO_LEFT_CENTER;
int currentRight = SERVO_RIGHT_CENTER;
unsigned long startTime = 0;

bool isRight = false;
bool isFallen = false;
bool isPhoneWas = false;
bool prevButtonState = HIGH;

// VIBRATION FUNCTION
void vibrateSmoothly(int duration_ms) {
  unsigned long start_time = millis();
  while (millis() - start_time < duration_ms) {
    for (int offset = -OFFSET; offset <= OFFSET; offset += STEP_SIZE_VIBE) {
      moveServoMicroseconds(SERVO_CHANNEL_LEFT,  SERVO_LEFT_CENTER + offset);
      moveServoMicroseconds(SERVO_CHANNEL_RIGHT, SERVO_RIGHT_CENTER - offset);
      delay(STEP_DELAY_VIBE);
    }
    for (int offset = OFFSET; offset >= -OFFSET; offset -= STEP_SIZE_VIBE) {
      moveServoMicroseconds(SERVO_CHANNEL_LEFT,  SERVO_LEFT_CENTER + offset);
      moveServoMicroseconds(SERVO_CHANNEL_RIGHT, SERVO_RIGHT_CENTER - offset);
      delay(STEP_DELAY_VIBE);
    }
  }
  moveServoMicroseconds(SERVO_CHANNEL_LEFT, SERVO_LEFT_CENTER);
  moveServoMicroseconds(SERVO_CHANNEL_RIGHT, SERVO_RIGHT_CENTER);
}


////LAST WORKING
void moveServoSmoothly() {
  int currentLeft = SERVO_LEFT_CENTER;
  int currentRight = SERVO_RIGHT_CENTER;
  int stepSizeRight = abs(SERVO_RIGHT_CENTER-SERVO_RIGHT_TARGET) / 2;
  int stepSizeLeft = abs(SERVO_LEFT_CENTER-SERVO_LEFT_TARGET) / 2;

  while (currentLeft != SERVO_LEFT_TARGET || currentRight != SERVO_RIGHT_TARGET) {

    // ---- Right Servo Step ----
    if (currentRight != SERVO_RIGHT_TARGET) {
      if (currentRight < SERVO_RIGHT_TARGET) {
        currentRight += stepSizeRight;
        if (currentRight > SERVO_RIGHT_TARGET) currentRight = SERVO_RIGHT_TARGET;
      } else {
        currentRight -= stepSizeRight;
        if (currentRight < SERVO_RIGHT_TARGET) currentRight = SERVO_RIGHT_TARGET;
      }
      moveServoMicroseconds(SERVO_CHANNEL_RIGHT, currentRight);
      delay(STEP_DELAY);
    }

    // ---- Left Servo Step ----
    if (currentLeft != SERVO_LEFT_TARGET) {
      if (currentLeft < SERVO_LEFT_TARGET) {
        currentLeft += stepSizeLeft;
        if (currentLeft > SERVO_LEFT_TARGET) currentLeft = SERVO_LEFT_TARGET;
      } else {
        currentLeft -= stepSizeLeft;
        if (currentLeft < SERVO_LEFT_TARGET) currentLeft = SERVO_LEFT_TARGET;
      }
      moveServoMicroseconds(SERVO_CHANNEL_LEFT, currentLeft);
      delay(STEP_DELAY);
    }
  }
  
  // Final correction to ensure both reach exact target
  moveServoMicroseconds(SERVO_CHANNEL_LEFT, SERVO_LEFT_TARGET);
  moveServoMicroseconds(SERVO_CHANNEL_RIGHT, SERVO_RIGHT_TARGET);
}

// MOVEMENT FUNCTION
//void moveServoSmoothly() {
//  int currentLeft = SERVO_LEFT_CENTER;
//  int currentRight = SERVO_RIGHT_CENTER;
//  while (currentLeft != SERVO_LEFT_TARGET || currentRight != SERVO_RIGHT_TARGET) {
//    // Move left servo toward target
//    if (currentLeft < SERVO_LEFT_TARGET) {
//      currentLeft += STEP_SIZE;
//      if (currentLeft > SERVO_LEFT_TARGET) currentLeft = SERVO_LEFT_TARGET;
//    } else if (currentLeft > SERVO_LEFT_TARGET) {
//      currentLeft -= STEP_SIZE;
//      if (currentLeft < SERVO_LEFT_TARGET) currentLeft = SERVO_LEFT_TARGET;
//    }
//
//    // Move right servo toward target
//    if (currentRight < SERVO_RIGHT_TARGET) {
//      currentRight += STEP_SIZE;
//      if (currentRight > SERVO_RIGHT_TARGET) currentRight = SERVO_RIGHT_TARGET;
//    } else if (currentRight > SERVO_RIGHT_TARGET) {
//      currentRight -= STEP_SIZE;
//      if (currentRight < SERVO_RIGHT_TARGET) currentRight = SERVO_RIGHT_TARGET;
//    }
//
//    // Apply servo movement
//    moveServoMicroseconds(SERVO_CHANNEL_LEFT, currentLeft);
//    moveServoMicroseconds(SERVO_CHANNEL_RIGHT, currentRight);
//    delay(STEP_DELAY);
//
//    // Reverse direction slightly for soft effect
//    if (currentLeft < SERVO_LEFT_TARGET) {
//      currentLeft -= STEP_SIZE - 100;
//      if (currentLeft > SERVO_LEFT_TARGET) currentLeft = SERVO_LEFT_TARGET;
//    } else if (currentLeft > SERVO_LEFT_TARGET) {
//      currentLeft += STEP_SIZE - 100;
//      if (currentLeft < SERVO_LEFT_TARGET) currentLeft = SERVO_LEFT_TARGET;
//    }
//
//    if (currentRight < SERVO_RIGHT_TARGET) {
//      currentRight -= STEP_SIZE - 100;
//      if (currentRight > SERVO_RIGHT_TARGET) currentRight = SERVO_RIGHT_TARGET;
//    } else if (currentRight > SERVO_RIGHT_TARGET) {
//      currentRight += STEP_SIZE - 100;
//      if (currentRight < SERVO_RIGHT_TARGET) currentRight = SERVO_RIGHT_TARGET;
//    }
//
//    moveServoMicroseconds(SERVO_CHANNEL_LEFT, currentLeft);
//    moveServoMicroseconds(SERVO_CHANNEL_RIGHT, currentRight);
//    delay(STEP_DELAY);
//  }
//}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Setup starting...");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(TRAY_IR_SENSOR_PIN, INPUT);

  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  moveServoMicroseconds(SERVO_CHANNEL_LEFT, SERVO_LEFT_CENTER);
  moveServoMicroseconds(SERVO_CHANNEL_RIGHT, SERVO_RIGHT_CENTER);
  Serial.println("Servo initialized at center.");
}

void loop() {
  // ---- Joystick Button Logic ----
  int buttonState = digitalRead(BUTTON_PIN);
  int traySensorValue = analogRead(TRAY_IR_SENSOR_PIN);
  float trayVoltage = traySensorValue * (3.3 / 4095.0);
  if (prevButtonState == HIGH && buttonState == LOW) {
    isRight = !isRight;
    if (isRight) {
      moveServoSmoothly();
      Serial.println("Servo moved RIGHT");
    } else {
      moveServoMicroseconds(SERVO_CHANNEL_LEFT, SERVO_LEFT_CENTER);
      moveServoMicroseconds(SERVO_CHANNEL_RIGHT, SERVO_RIGHT_CENTER);
      Serial.println("Servo moved CENTER");
    }
    delay(200);
  }
  prevButtonState = buttonState;

  // ---- IR Sensor & Vibration Motor Logic ----
  int sensorValue = analogRead(IR_SENSOR_PIN);
  float voltage = sensorValue * (3.3 / 4095.0);  // Convert to volts

  Serial.print("TRAY IR Sensor Voltage: ");
  Serial.print(trayVoltage);
  Serial.print(" V - ");

  if (voltage > 1.5) {
    Serial.println("Hand detected → Vibrating with servos...");
    vibrateSmoothly(VIBRATION_DURATION);
  } else {
    Serial.println("No hand detected.");
  }

  delay(100); // Smooth reading

  // !isRight means that the robot is up
  if (trayVoltage > 1 && !isRight) {
    Serial.println("detected phone on tray");
    isPhoneWas = true;
  }

  if (isPhoneWas && !isRight && trayVoltage < 0.5) {
    Serial.println("phone is off");
    isRight = !isRight;
    moveServoSmoothly();
    startTime = millis();
    isFallen = true;
    isPhoneWas = false;
  }

  if (isFallen && millis() - startTime > WAIT_TIME_AFTER_FALL) {
    isRight = !isRight;
    moveServoMicroseconds(SERVO_CHANNEL_LEFT, SERVO_LEFT_CENTER);
    moveServoMicroseconds(SERVO_CHANNEL_RIGHT, SERVO_RIGHT_CENTER);
    isFallen = false;
  }
  
  delay(100);
}

// Convert microseconds to PWM signal for PCA9685
void moveServoMicroseconds(uint8_t channel, int pulse_us) {
  int pwm_val = (pulse_us * 4096) / 20000;
  pwm.setPWM(channel, 0, pwm_val);

  Serial.print("Servo Channel ");
  Serial.print(channel);
  Serial.print(" → ");
  Serial.print(pulse_us);
  Serial.print("us → PWM: ");
  Serial.println(pwm_val);
}
