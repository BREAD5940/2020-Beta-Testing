#include <Servo.h>

int proxInput = A4;
int wristInput = A2;

//int proxPWM = 10;
//int wristPWM = 11; // avoid 5 and 6

Servo prox;
Servo wrist;

double proxIn = 0;
double wristIn = 0;

double _min = 2000.0;
double _max = -1.0;

void setup() {
//  pinMode(proxPWM, OUTPUT);
//  pinMode(wristPWM, OUTPUT);
  Serial.begin(9600);
  prox.attach(10);
  wrist.attach(11);
}

void loop() {
  proxIn = (analogRead(proxInput) / 1023.0 * 360.0); // 0 to 1
  wristIn = (analogRead(wristInput) / 1023.0 * 360.0); // 0 to 1

  proxIn = proxIn;
  wristIn = wristIn;

  if(proxIn > _max) {
    _max = proxIn;
  }
  if(proxIn < _min) {
    _min = proxIn;
  }
  

  Serial.print("prox ");
  Serial.print(proxIn);
  Serial.print(" wrist ");
  Serial.print(wristIn);
  Serial.println(" ");

  prox.write(proxIn / 2.0);
  wrist.write(wristIn / 2.0);
 
//  analogWrite(proxPWM, 0.8);
//  analogWrite(wristPWM, wristIn);
}
