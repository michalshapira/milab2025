#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // Default address

#define SERVO_CHANNEL 8       // Servo connected to channel 8 on PCA9685
#define SERVO_CENTER  1200    // Custom center pulse
#define SERVO_RIGHT   1800    // Custom right pulse

#define BUTTON_PIN 32         // Joystick button

// IR sensor and vibration motor
#define IR_SENSOR_PIN 34
#define VIBRATION_PIN 19

// Servo state
bool isRight = false;
bool prevButtonState = HIGH;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Setup starting...");

  // Button setup
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // IR sensor and vibration motor
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);  // Start with vibration off

  // I2C and PCA9685 setup
  Wire.begin(21, 22);  // ESP32 SDA, SCL
  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz for servos
  delay(10);

  // Initialize servo position
  moveServoMicroseconds(SERVO_CHANNEL, SERVO_CENTER);
  Serial.println("Servo initialized at center.");
}

void loop() {
  // ---- Joystick Button Logic ----
  int buttonState = digitalRead(BUTTON_PIN);
  if (prevButtonState == HIGH && buttonState == LOW) {
    isRight = !isRight;
    if (isRight) {
      moveServoMicroseconds(SERVO_CHANNEL, SERVO_RIGHT);
      Serial.println("Servo moved RIGHT");
    } else {
      moveServoMicroseconds(SERVO_CHANNEL, SERVO_CENTER);
      Serial.println("Servo moved CENTER");
    }
    delay(200); // Debounce
  }
  prevButtonState = buttonState;

  // ---- IR Sensor & Vibration Motor Logic ----
  int sensorValue = analogRead(IR_SENSOR_PIN);
  float voltage = sensorValue * (3.3 / 4095.0);  // Convert to volts

  Serial.print("IR Sensor Voltage: ");
  Serial.print(voltage);
  Serial.print(" V - ");

  if (voltage > 1.5) {
    digitalWrite(VIBRATION_PIN, HIGH);
    Serial.println("Hand detected - Vibration ON");
  } else {
    digitalWrite(VIBRATION_PIN, LOW);
    Serial.println("No hand - Vibration OFF");
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
