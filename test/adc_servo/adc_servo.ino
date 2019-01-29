#include <Servo.h>

Servo servo;
int pot = 0;
int ms = 1500;
int pos = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A0, INPUT);
  servo.attach(9);

}

void loop() {
  // put your main code here, to run repeatedly:
  pot = analogRead(A0);
  Serial.print(pot);

  ms = map(pot, 0, 1023, 1000, 2000);
  Serial.print("->");
  Serial.print(ms);
  
  if (servo.attached()){
    servo.writeMicroseconds(ms);
    Serial.print("->");
    Serial.println(servo.read());
  } else {
    Serial.println("servo error");
  }
  
  
}
