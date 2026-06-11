#include <Servo.h>

Servo myServo;

// Potentiometer connected to A0
const int potPin = A0;
const int ledPin=5;

void setup() 
{
  Serial.begin(9600);
  pinMode(ledPin,OUTPUT);
  myServo.attach(9);
}

void loop() 
{
  int potValue = analogRead(potPin);

  Serial.print("Potentiometer Value: ");
  Serial.println(potValue);

  int brightness = map(potValue, 0, 1023, 0, 255);
  analogWrite(ledPin, brightness);

  int angle = map(potValue, 0, 1023, 0, 180);
  myServo.write(angle);

  delay(100);
}