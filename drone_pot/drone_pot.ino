int pot_pin = A0;
int pot_val;
int dist_mm;
int max_dist = 305; // Adjust this based on the potentiomter used

void setup() {
  Serial.begin(115200);
  pinMode(pot_pin, INPUT);
  Serial.print("begin");
}

void loop() {
  pot_val = analogRead(pot_pin);
  dist_mm = map(pot_val, 0, 1023, 0, max_dist);

  Serial.println(dist_mm);
  delay(40);
}
