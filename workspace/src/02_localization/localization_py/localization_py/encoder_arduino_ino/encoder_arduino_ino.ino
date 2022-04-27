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
  voltage = analogRead(A5);
  incomingByte = Serial.read();
  if (incomingByte == 1) {
    delay(5);
    Serial.println(voltage);
  }    
}
