int incomingByte = 0;
int voltage;

void setup() {
  Serial.begin(9600);
  voltage = 0;
  while (!Serial){
    ;
  }
}

void loop() {
  voltage = analogRead(A0);
//  double out_voltage = double(voltage/200.4);
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == 1) {
      delay(5);
      // Serial.write((uint8_t*)&voltage, sizeof(int));
      Serial.println(voltage);
    }
  }    
}
