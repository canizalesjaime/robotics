const int pirPin = 7;     // PIR output pin
const int ledPin = 5;    

int pirState = LOW;       // Current state
int val = 0;              // Sensor value

void setup() {
  //pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);
  Serial.println("PIR Sensor initializing...");

  // Give sensor time to stabilize
  //delay(30000); // HC-SR501 often needs 30-60 sec warm-up
}

void loop() {
  //val = digitalRead(pirPin);
  digitalWrite(ledPin,LOW);
  if (val == HIGH) {
    digitalWrite(ledPin, HIGH);

    if (pirState == LOW) {
      Serial.println("Motion detected!");
      pirState = HIGH;
    }
  } 
  else {
    digitalWrite(ledPin, LOW);

    if (pirState == HIGH) {
      Serial.println("Motion ended.");
      pirState = LOW;
    }
  }

  delay(100);
}