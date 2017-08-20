#define ENCODER_PIN  2

volatile unsigned long clicks = 0;
unsigned long last_clicks = 0;

void setup() {
    Serial.begin(9600);
    pinMode(ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoder_isr, CHANGE);
    Serial.println("Interrupt attached");
}

void loop() {
    if (clicks != last_clicks) {
        Serial.print("Clicks: ");
        Serial.println(clicks);
        last_clicks = clicks;
    }
    delay(1000);
}

void encoder_isr() {
    clicks++;
}