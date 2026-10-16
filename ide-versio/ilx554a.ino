#define CLK_PIN 16
#define ROG_PIN 17
#define ADC_PIN 26

#define NUM_PIXELS 2048

uint16_t frameBuffer[NUM_PIXELS];

void setup() {
  // Use highest practical baud rate (Pico USB serial runs at native USB speeds)
  Serial.begin(2000000); 
  
  pinMode(CLK_PIN, OUTPUT);
  pinMode(ROG_PIN, OUTPUT);
  
  // Pico has a 12-bit ADC (0-4095)
  analogReadResolution(12);
  
  digitalWrite(CLK_PIN, LOW);
  digitalWrite(ROG_PIN, HIGH);
}

void loop() {
  // 1. Electronic Shutter / Transfer (ROG Pulse)
  digitalWrite(CLK_PIN, HIGH);
  digitalWrite(ROG_PIN, LOW);
  delayMicroseconds(5); // Minimum ROG pulse width
  digitalWrite(ROG_PIN, HIGH);
  digitalWrite(CLK_PIN, LOW);
  
  // 2. Clock out optical black & dummy pixels (approx 33 clocks)
  for(int i = 0; i < 33; i++) {
    digitalWrite(CLK_PIN, HIGH);
    digitalWrite(CLK_PIN, LOW);
  }

  // 3. Read effective pixels
  for(int i = 0; i < NUM_PIXELS; i++) {
    digitalWrite(CLK_PIN, HIGH);
    digitalWrite(CLK_PIN, LOW);
    
    // Invert reading so higher values = brighter light
    frameBuffer[i] = 4095 - analogRead(ADC_PIN);
  }

  // 4. Send binary payload to PC: [S][Y][N][C][4096 bytes data]
  Serial.write("SYNC", 4);
  Serial.write((uint8_t*)frameBuffer, sizeof(frameBuffer));
  
  // 5. Integration time (increase delay to increase sensor light sensitivity)
  delay(10); 
}