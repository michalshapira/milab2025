#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create servo driver instance (default I2C address 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Channel the servo is connected to
#define SERVO_CHANNEL 8

// Pulse lengths in microseconds for servo control
#define SERVO_CENTER  1200    // move right number of degrees
#define SERVO_RIGHT   1800    // move left number of degrees 

#define BUTTON_PIN 32         // Joystick button on GPIO 32


bool isRight = false;
bool prevButtonState = HIGH;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Setup starting...");

  // Button setup
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // I2C and servo driver init
  Wire.begin(21, 22);   // ESP32 SDA = 21, SCL = 22
  pwm.begin();
  pwm.setPWMFreq(50);   // 50Hz for servos
  delay(10);

  // Move to center at start
  moveServoMicroseconds(SERVO_CHANNEL, SERVO_CENTER);
  Serial.println("Servo initialized at center.");
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);

  // Detect falling edge (press)
  if (prevButtonState == HIGH && buttonState == LOW) {
    isRight = !isRight;  // Toggle position

    if (isRight) {
      moveServoMicroseconds(SERVO_CHANNEL, SERVO_RIGHT);
      Serial.println("Moved RIGHT");
    } else {
      moveServoMicroseconds(SERVO_CHANNEL, SERVO_CENTER);
      Serial.println("Moved CENTER");
    }

    delay(200); // debounce
  }

  prevButtonState = buttonState;
}

// Send pulse to PCA9685 (in microseconds)
void moveServoMicroseconds(uint8_t channel, int pulse_us) {
  int pwm_val = (pulse_us * 4096) / 20000;
  pwm.setPWM(channel, 0, pwm_val);

  Serial.print("Pulse ");
  Serial.print(pulse_us);
  Serial.print("us → PWM ");
  Serial.println(pwm_val);
}
