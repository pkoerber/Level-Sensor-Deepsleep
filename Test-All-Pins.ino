// D1 (GPIO5), D2 (GPIO4): default for I2C interface
// D0 (GPIO16): wake, extra LED on NodeMCU
// D4 (GPIO2): LED (flashes during programming, TXD1)

// special GPIO1 (TX), GPIO3 (RX), GPIO16 (D0)
// normal GPIO0 (D3), GPIO2 (D4), GPIO4 (D2), GPIO5 (D1)
// normal GPIO12 (D6), GPIO13 (D7), GPIO14 (D5), GPIO15 (D8)

// Connections adapter plate ESP-12F
// EN (CH-PD): pull-up
// GPIO15: pull-down

// Boot modes
// Programming: GPIO15: low, GPIO0: low, GPIO2: high 
// Run firmware from flash: GPIO15: low, GPIO0: high, GPIO2: high 

const int RA1=100;
const int RA2=470;


void setup() {
  Serial.begin(9600);
  pinMode(0, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
}

void loop() {
  digitalWrite(0, LOW);
  digitalWrite(2, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(14, LOW);
  digitalWrite(15, LOW);
  delay(5000);
  
  Serial.println("0 high");
  digitalWrite(0, HIGH);
  int analog=analogRead(A0);
  Serial.printf("Analog read: %d, voltage: %.2f\n", analog, analog/1023.0*(RA2+RA1)/RA1);
  delay(5000);
  digitalWrite(0, LOW);
  
  Serial.println("2 high, ESP-12F on-board LED");
  digitalWrite(2, HIGH);
  analog=analogRead(A0);
  Serial.printf("Analog read: %d, voltage: %.2f\n", analog, analog/1023.0*(RA2+RA1)/RA1);
  delay(5000);
  digitalWrite(2, LOW);
  
  Serial.println("4 high");
  digitalWrite(4, HIGH);
  analog=analogRead(A0);
  Serial.printf("Analog read: %d, voltage: %.2f\n", analog, analog/1023.0*(RA2+RA1)/RA1);
  delay(5000);
  digitalWrite(4, LOW);

  Serial.println("5 high");
  digitalWrite(5, HIGH);
  analog=analogRead(A0);
  Serial.printf("Analog read: %d, voltage: %.2f\n", analog, analog/1023.0*(RA2+RA1)/RA1);
  delay(5000);
  digitalWrite(5, LOW);

  Serial.println("12 high");
  digitalWrite(12, HIGH);
  analog=analogRead(A0);
  Serial.printf("Analog read: %d, voltage: %.2f\n", analog, analog/1023.0*(RA2+RA1)/RA1);
  delay(5000);
  digitalWrite(12, LOW);

  Serial.println("13 high");
  digitalWrite(13, HIGH);
  analog=analogRead(A0);
  Serial.printf("Analog read: %d, voltage: %.2f\n", analog, analog/1023.0*(RA2+RA1)/RA1);
  delay(5000);
  digitalWrite(13, LOW);
  
  Serial.println("14 high");
  digitalWrite(14, HIGH);
  analog=analogRead(A0);
  Serial.printf("Analog read: %d, voltage: %.2f\n", analog, analog/1023.0*(RA2+RA1)/RA1);
  delay(5000);
  digitalWrite(14, LOW);
  
  Serial.println("15 high");
  digitalWrite(15, HIGH);
  analog=analogRead(A0);
  Serial.printf("Analog read: %d, voltage: %.2f\n", analog, analog/1023.0*(RA2+RA1)/RA1);
  delay(5000);
  digitalWrite(15, LOW);
  
  // Blink before going to deep-sleep
  delay(250);
  digitalWrite(2, HIGH);
  delay(250);
  digitalWrite(2, LOW);
  delay(250);
  digitalWrite(2, HIGH);
  delay(250);
  
  Serial.println("Deep sleep");
  ESP.deepSleep(2e7);
}
