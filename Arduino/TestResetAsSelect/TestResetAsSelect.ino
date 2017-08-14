
void setup() {
    pinMode(A0, INPUT);
    pinMode(4, OUTPUT);
}

void loop() {
    digitalWrite(4, HIGH);
    delay(analogRead(A0));
    digitalWrite(4, LOW);
    delay(analogRead(A0));
}
