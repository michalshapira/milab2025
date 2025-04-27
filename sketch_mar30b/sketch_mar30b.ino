#define IR_SENSOR_PIN 34
#define VIBRATION_PIN 19
#define UP 17
#define DOWN 16

void setup() {
  Serial.begin(115200);

  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW); // motor OFF
}

void loop() {
  int sensorValue = analogRead(IR_SENSOR_PIN);
  float voltage = sensorValue * (3.3 / 4095.0);  // convert to volts

  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print(" V - ");

  if (voltage > 1.5) {  // 👋 Hand detected
    digitalWrite(VIBRATION_PIN, HIGH);
    Serial.println("Hand detected - Motor ON");
  } else {
    digitalWrite(VIBRATION_PIN, LOW);
    Serial.println("No hand - Motor OFF");
  }

  delay(100);  // smooth reading
}
