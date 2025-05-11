#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // Default address

#define SERVO_CHANNEL_LEFT 8       // right Servo connected to channel 8 on PCA9685
#define SERVO_CHANNEL_RIGHT 9       // left Servo connected to channel 9 on PCA9685
#define SERVO_CENTER  1500    // Custom center pulse

#define SERVO_RIGHT 1200    // Custom center pulse
#define SERVO_LEFT 1800    // Custom center pulse

#define SERVO_RIGHT_CENTER 1500    // Custom center pulse
#define SERVO_LEFT_CENTER 1500    // Custom center pulse

#define SERVO_OFFSET 300

#define SERVO_LEFT_POS  (SERVO_LEFT_CENTER + SERVO_OFFSET)   // e.g., 1800
#define SERVO_RIGHT_POS (SERVO_RIGHT_CENTER - SERVO_OFFSET)   // e.g., 1200


#define BUTTON_PIN 32         // Joystick button

// IR sensor and vibration motor
#define IR_SENSOR_PIN 34
#define TRAY_IR_SENSOR_PIN 35

#define VIBRATION_PIN 19

#define SERVO_RIGHT_TARGET 1000
#define SERVO_LEFT_TARGET 2000

#define STEP_DELAY 150        // delay between each step in ms
#define STEP_SIZE 200         // microseconds to move per step

int currentLeft = 1500;      // starting point for left servo
int currentRight = 1500;     // starting point for right servo
unsigned long startTime = 0;

// Servo state
bool isRight = false;
bool isFallen = false;
bool isPhoneWas = false;
bool prevButtonState = HIGH;

void moveServoSmoothly() {
  int currentLeft = 1500;      // starting point for left servo
  int currentRight = 1500;
  while (currentLeft != SERVO_LEFT_TARGET || currentRight != SERVO_RIGHT_TARGET) {
    // Move left servo toward target
    if (currentLeft < SERVO_LEFT_TARGET) {
      currentLeft += STEP_SIZE;
      if (currentLeft > SERVO_LEFT_TARGET) currentLeft = SERVO_LEFT_TARGET;
    } else if (currentLeft > SERVO_LEFT_TARGET) {
      currentLeft -= STEP_SIZE;
      if (currentLeft < SERVO_LEFT_TARGET) currentLeft = SERVO_LEFT_TARGET;
    }

    // Move right servo toward target
    if (currentRight < SERVO_RIGHT_TARGET) {
      currentRight += STEP_SIZE;
      if (currentRight > SERVO_RIGHT_TARGET) currentRight = SERVO_RIGHT_TARGET;
    } else if (currentRight > SERVO_RIGHT_TARGET) {
      currentRight -= STEP_SIZE;
      if (currentRight < SERVO_RIGHT_TARGET) currentRight = SERVO_RIGHT_TARGET;
    }

    // Apply servo movement
    moveServoMicroseconds(SERVO_CHANNEL_LEFT, currentLeft);
    moveServoMicroseconds(SERVO_CHANNEL_RIGHT, currentRight);
    delay(STEP_DELAY);

    // Move left servo toward target
    if (currentLeft < SERVO_LEFT_TARGET) {
      currentLeft -= STEP_SIZE - 100;
      if (currentLeft > SERVO_LEFT_TARGET) currentLeft = SERVO_LEFT_TARGET;
    } else if (currentLeft > SERVO_LEFT_TARGET) {
      currentLeft += STEP_SIZE - 100;
      if (currentLeft < SERVO_LEFT_TARGET) currentLeft = SERVO_LEFT_TARGET;
    }

    // Move right servo toward target
    if (currentRight < SERVO_RIGHT_TARGET) {
      currentRight -= STEP_SIZE - 100;
      if (currentRight > SERVO_RIGHT_TARGET) currentRight = SERVO_RIGHT_TARGET;
    } else if (currentRight > SERVO_RIGHT_TARGET) {
      currentRight += STEP_SIZE - 100;
      if (currentRight < SERVO_RIGHT_TARGET) currentRight = SERVO_RIGHT_TARGET;
    }

    // Apply servo movement
    moveServoMicroseconds(SERVO_CHANNEL_LEFT, currentLeft);
    moveServoMicroseconds(SERVO_CHANNEL_RIGHT, currentRight);
    delay(STEP_DELAY);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Setup starting...");

  // Button setup
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // IR sensor and vibration motor
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(TRAY_IR_SENSOR_PIN, INPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);  // Start with vibration off

  // I2C and PCA9685 setup
  Wire.begin(21, 22);  // ESP32 SDA, SCL
  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz for servos
  delay(10);

  // Initialize servo position
  moveServoMicroseconds(SERVO_CHANNEL_LEFT, SERVO_CENTER);
  moveServoMicroseconds(SERVO_CHANNEL_RIGHT, SERVO_CENTER);
  Serial.println("Servo initialized at center.");
}

void loop() {
  // ---- Joystick Button Logic ----
  int buttonState = digitalRead(BUTTON_PIN);
  int traySensorValue = analogRead(TRAY_IR_SENSOR_PIN);
  float trayVoltage = traySensorValue * (3.3 / 4095.0);  // Convert to volts
  if (prevButtonState == HIGH && buttonState == LOW) {
    isRight = !isRight;
    if (isRight) {
      //moveServoMicroseconds(SERVO_CHANNEL_LEFT, SERVO_LEFT);
      //moveServoMicroseconds(SERVO_CHANNEL_RIGHT, SERVO_RIGHT);
      moveServoSmoothly();
      Serial.println("Servo moved RIGHT");
    } else {
      moveServoMicroseconds(SERVO_CHANNEL_LEFT, SERVO_CENTER);
      moveServoMicroseconds(SERVO_CHANNEL_RIGHT, SERVO_CENTER);
      Serial.println("Servo moved CENTER");
    }
    delay(200); // Debounce
  }
  prevButtonState = buttonState;

  // ---- IR Sensor & Vibration Motor Logic ----
  int sensorValue = analogRead(IR_SENSOR_PIN);
  float voltage = sensorValue * (3.3 / 4095.0);  // Convert to volts

  Serial.print("TRAY IR Sensor Voltage: ");
  Serial.print(trayVoltage);
  Serial.print(" V - ");

  if (voltage > 1.0) {
    digitalWrite(VIBRATION_PIN, HIGH);
    Serial.println("Hand detected - Vibration ON");
  } else {
    digitalWrite(VIBRATION_PIN, LOW);
    Serial.println("No hand - Vibration OFF");
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
   
  if(isFallen && millis() - startTime > 30000) {
    isRight = !isRight;
    moveServoMicroseconds(SERVO_CHANNEL_LEFT, SERVO_CENTER);
    moveServoMicroseconds(SERVO_CHANNEL_RIGHT, SERVO_CENTER);
    isFallen = false;
  }
  delay(100); // Smooth reading
}

// Convert microseconds to PWM signal for PCA9685
void moveServoMicroseconds(uint8_t channel, int pulse_us) {
  int pwm_val = (pulse_us * 4096) / 20000;  // Convert to 12-bit value
  pwm.setPWM(channel, 0, pwm_val);

  Serial.print("Servo Channel ");
  Serial.print(channel);
  Serial.print(" → ");
  Serial.print(pulse_us);
  Serial.print("us → PWM: ");
  Serial.println(pwm_val);
}
