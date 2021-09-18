#define PIN7 7
void setup() {
  pinMode(PIN7, OUTPUT);
  digitalWrite(7, LOW);
  delay(1000);
}
void loop() {
  for (int i = 0; i < 5; i++){
    digitalWrite(PIN7, HIGH);
    delay(100);
    digitalWrite(PIN7, LOW);
    delay(100);
  }
  while(1){
    digitalWrite(7, HIGH);
  }
}
