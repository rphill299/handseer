const int emgPin = A0;  // Analog pin connected to ENV
int emgValue = 0;
unsigned long ttime;

void setup() {
  Serial.begin(115200);
  pinMode(emgPin, INPUT);
}

void loop() {
  static unsigned long prev = millis();
  if (millis() - prev >= 10) {
    prev += 10;
    emgValue = analogRead(emgPin);
    Serial.println(emgValue);
  }
}
