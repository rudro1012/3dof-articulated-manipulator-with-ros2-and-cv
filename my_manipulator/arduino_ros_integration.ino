#include <Servo.h>

Servo myservo;

int pin=9;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(9);
  Serial.begin(9600);
  myservo.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available()>0){
    String anglestr=Serial.readStringUntil('\n');
    float angle= anglestr.toFloat();

    Serial.println("ACK");
    
    int minPulse = 500;  // try 500 or 600
    int maxPulse = 2500; // try 2300–2500
    int pulse = minPulse + (angle / 180.0) * (maxPulse - minPulse);
    myservo.writeMicroseconds(pulse);
  }

}
