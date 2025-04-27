#include <ESP32Servo.h>

#define JOYSTICK_SW_PIN 32
#define SERVO_PIN 23

Servo myservo;
bool isVibrating = false;
bool prevButtonState = HIGH;

void setup() {
  Serial.begin(115200);
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);

  // Allocate PWM timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  myservo.setPeriodHertz(50); // 50Hz
  myservo.attach(SERVO_PIN, 500, 2500); // Control pulse range
}

void loop() {
  int buttonState = digitalRead(JOYSTICK_SW_PIN);

  if (prevButtonState == HIGH && buttonState == LOW) {
    isVibrating = !isVibrating;  // Toggle on press
    Serial.println(isVibrating ? "Vibration ON" : "Vibration OFF");
    delay(200); // debounce
  }

  if (isVibrating) {
    myservo.write(60);  // Slightly rotate one direction
    delay(100);
    myservo.write(120); // Then the other
    delay(100);
  } else {
    myservo.write(90); // Stop movement
  }

  prevButtonState = buttonState;
}
