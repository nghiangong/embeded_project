#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <ESP32Servo.h>

#define SCALE 0.0914
#define SCOPE 700
#define DISPLAY_ANGLE 60;

#define servoPin 18

#define SPEED_OF_SOUND 343.0
#define SR04_TRIG_PIN 4    // Vị trí chân GPIO của ESP32 được nối với Trig của SR04
#define SR04_ECHO_PIN 2   // Vị trí chân GPIO của ESP32 được nối với Echo của SR04

#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   // QT-PY / XIAO

Servo myServo;
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int direction = 1;
float a[180];
int xO = 63;
int yO = 63;

void setup() {
  Serial.begin(115200);

  myServo.attach(servoPin);

  pinMode(SR04_TRIG_PIN, OUTPUT);
  pinMode(SR04_ECHO_PIN, INPUT);

  delay(250); // wait for the OLED to power up
  display.begin(i2c_Address, true); // Address 0x3C default

  display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
  display.fillCircle(63, 63, 63, SH110X_BLACK);
  display.display();
  delay(2000);

}
void reset() {
  display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
  display.fillCircle(63, 63, 63, SH110X_BLACK);
  display.display();
}

void show(float distance, float angle) {
  float radAngle1 = (angle + direction) * PI / 180.0;
  float radAngle = angle * PI / 180.0;
  float radAngle_60 = (angle - 60*direction) * PI / 180.0;

  int xA = 63*cos(radAngle1) + 63;
  int yA = - 63*sin(radAngle1) + 63;
  display.drawLine(xO, yO, xA, yA, SH110X_WHITE);

  int xB = 63*cos(radAngle_60) +63;
  int yB = - 63*sin(radAngle_60) + 63;
  display.drawLine(xO, yO, xB, yB, SH110X_BLACK);


  if (distance < SCOPE) 
    for (int i = 0; i <= 2; i++) {
      float angle_rad = (angle - direction*i*0.5) * PI / 180.0;

      int x0 = distance*SCALE*cos(angle_rad) + 63;
      int y0 = - distance*SCALE*sin(angle_rad) + 63;
      int x1 = 63*cos(angle_rad) + 63;
      int y1 = - 63*sin(angle_rad) + 63;
      display.drawLine(x0, y0, x1, y1, SH110X_BLACK);
    };

  display.drawCircle(63, 63, 64, SH110X_WHITE);
}
int getDistance() {
  digitalWrite(SR04_TRIG_PIN, LOW);  // Đưa chân Trig xuống mức thấp trong 2uS
  delayMicroseconds(2);
  digitalWrite(SR04_TRIG_PIN, HIGH);  //Gửi luồng siêu âm kéo dài 10uS
  delayMicroseconds(10);
  digitalWrite(SR04_TRIG_PIN, LOW);   //Tắt luồng siêu âm
  unsigned int duration = pulseIn(SR04_ECHO_PIN, HIGH); // Đợi cho tới khi có phản hồi
  return (duration * SPEED_OF_SOUND) / 2000.0;
}

unsigned int distance;
int pos = 15;
void run() {
  myServo.write(pos);
  delay(10);
  distance = getDistance();
  show(distance, pos);
  Serial.println(distance);
}

void loop() {
  while (15 <= pos && pos <= 165) {
    run();
    delay(10);
    pos += direction;
    display.display();
  }
  reset();
  display.display();
  delay(10);
  pos -= direction;
  direction = -direction;
}
