#define RE_DE_PIN 4

HardwareSerial RS485Serial(2); // UART2

void setup() {
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // Receive mode

  Serial.begin(115200);

  // RX=16, TX=17
  RS485Serial.begin(9600, SERIAL_8N1, 16, 17);
}

void loop() {

  // Read distance command
  uint8_t request[] = {
    0x01, 0x03, 0x00, 0x3B,
    0x00, 0x02, 0xB5, 0xC6
  };

  // Transmit
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(200);

  RS485Serial.write(request, sizeof(request));
  RS485Serial.flush();

  delayMicroseconds(200);
  digitalWrite(RE_DE_PIN, LOW);

  // Receive
  uint8_t response[9];
  int bytesRead = 0;

  unsigned long startTime = millis();

  while ((millis() - startTime) < 50 && bytesRead < 9) {
    if (RS485Serial.available()) {
      response[bytesRead++] = RS485Serial.read();
    }
  }

  // Only process complete packets
  if (bytesRead == 9 &&
      response[0] == 0x01 &&
      response[1] == 0x03 &&
      response[2] == 0x04) {

    uint32_t distance_um =
        ((uint32_t)response[3] << 24) |
        ((uint32_t)response[4] << 16) |
        ((uint32_t)response[5] << 8)  |
        response[6];

    float distance_mm = distance_um / 1000.0; // SDL-100/200/400: raw unit is 1 µm

    Serial.print("Distance: ");
    Serial.print(distance_mm, 3);
    Serial.println(" mm");
  }

  delay(5);
}