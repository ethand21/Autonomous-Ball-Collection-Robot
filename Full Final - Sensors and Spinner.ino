#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

Servo myservo;

#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define trigPin1 6
#define echoPin1 7
#define trigPin2 8
#define echoPin2 9
#define EN1 10
#define EN2 11
#define trigPin3 12
#define echoPin3 13
#define sensor A1
#define trigPinIR A2
#define echoPinIR A3

float reading;

float pingTime1, pingTime2, pingTime3, pingTimeIR;
float distance1, distance2, distance3, distanceIR;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // put your setup code here, to run once:

  lcd.init();       // initialize the LCD
  lcd.clear();      // clear the LCD display
  lcd.backlight();  // Make sure backlight is on

  pinMode(IN1, OUTPUT);  // Right Wheel Backward
  pinMode(IN2, OUTPUT);  // Right Wheel Forward
  pinMode(IN3, OUTPUT);  // Left Wheels Backward
  pinMode(IN4, OUTPUT);  // Left Wheels Forward
  pinMode(EN1, OUTPUT);  // Right Wheel Speed
  pinMode(EN2, OUTPUT);  // Left Wheel Speed

  pinMode(echoPin1, INPUT);  // Front sensor
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin2, INPUT);  // Right sensor
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin3, INPUT);  // Left sensor
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPinIR, INPUT);  // Left sensor
  pinMode(trigPinIR, OUTPUT);

  pinMode(sensor, INPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  pingTime1 = pulseIn(echoPin1, HIGH);    // Ping time in microsec
  distance1 = pingTime1 * 0.0134992 / 2;  // Distance in/sec

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  pingTime2 = pulseIn(echoPin2, HIGH);    // Ping time in microsec
  distance2 = pingTime2 * 0.0134992 / 2;  // Distance in/sec

  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);

  pingTime3 = pulseIn(echoPin3, HIGH);    // Ping time in microsec
  distance3 = pingTime3 * 0.0134992 / 2;  // Distance in/sec

  digitalWrite(trigPinIR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinIR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinIR, LOW);

  pingTimeIR = pulseIn(echoPinIR, HIGH);    // Ping time in microsec
  distanceIR = pingTimeIR * 0.0134992 / 2;  // Distance in/sec
  // if (distance1 < 9) {
  //   distance1 = 16;
  // }
  if (distance1 > 16) {
  //   // if (distance2 > 11 && distance2 < 20) {
  //   //   right();
  //   // } else if (distance2 < 3) {
  //   //   left();
  //   // } else {
      forward();
    }
  if (distance1 < 16) {
    stop();
    delay(1000);
    if (distance2 > 16) {
      right();
      delay(735);
      stop();
      delay(1000);
    } else if (distance3 > 16) {
      left();
      delay(785);
      stop();
      delay(1000);
    }
  }

  if ((digitalRead(sensor) == HIGH) && (distanceIR < 4)) {
    lcd.clear();
    lcd.setCursor(1, 0);  //Set cursor to character 2 on line 0
    lcd.print("Ball Detected:");
    lcd.setCursor(6, 1);  //Set cursor to character 2 on line 0
    lcd.print("Black");
    delay(250);
  } else if ((digitalRead(sensor) == LOW) && (distanceIR < 4)) {
    lcd.clear();
    lcd.setCursor(1, 0);  //Set cursor to character 2 on line 0
    lcd.print("Ball Detected:");
    pinMode(echoPin3, INPUT);  // Left sensor
    lcd.setCursor(6, 1);       //Set cursor to character 2 on line 0
    lcd.print("White");
    delay(250);
  } else {
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("Scanning");
  }
  reading = analogRead(distanceIR);

  Serial.println(distance1);
}
void forward() {
  digitalWrite(IN1, LOW);  // LW Forward
  digitalWrite(IN2, HIGH);  // RW forward
  digitalWrite(IN3, LOW);   // LW Back
  digitalWrite(IN4, HIGH);   // RW back

  analogWrite(EN1, 122);  // right
  analogWrite(EN2, 117);  // left
}
void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(EN1, 100);  // right
  analogWrite(EN2, 100);  // left
}
void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(EN1, 100);  // right
  analogWrite(EN2, 100);  // left
}
void reverse() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(EN1, 122);
  analogWrite(EN2, 122);
}
void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(EN1, 122);
  analogWrite(EN2, 122);
}