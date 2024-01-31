#include <Servo.h>
#include<Wire.h>

Servo servoX;
Servo servoY;

int angle_x[8] = {90, 90, 90, 90, 90, 90, 90, 90};
int angle_y[8] = {90, 90, 90, 90, 90, 90, 90, 90};

#define Addr 0x53

void setup() {
  servoX.attach(9);
  servoY.attach(11);
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();
  Wire.beginTransmission(Addr);
  Wire.write(0x2C);
  Wire.write(0x0A);
  Wire.endTransmission();
  Wire.beginTransmission(Addr);
  Wire.write(0x2D);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(Addr);
  Wire.write(0x31);
  Wire.write(0x08);
  Wire.endTransmission();
  // https://www.instructables.com/Measurement-of-Acceleration-Using-ADXL345-and-Ardu/
}

int pos = 0;
bool debugLedOn = false;




void loop() {

  digitalWrite(LED_BUILTIN, debugLedOn);
  debugLedOn = !debugLedOn;

  //Serial.print("Bruh " + String(debugLedOn) + "\n");
  // myservo.write(pos);
  // delay(15);
  // pos += 2;
  // if (pos > 179)
  //   pos = 0;


  unsigned int data[6];
  for (int i = 0; i < 6; i++) {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write((50 + i));
    // Stop I2C transmission
    Wire.endTransmission();
    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 10-bits
  int xAccl = (((data[1] & 0x03) * 256) + data[0]);
  if (xAccl > 511)
    xAccl -= 1024;

  int yAccl = (((data[3] & 0x03) * 256) + data[2]);
  if (yAccl > 511)
    yAccl -= 1024;

  int zAccl = (((data[5] & 0x03) * 256) + data[4]);
  if (zAccl > 511)
    zAccl -= 1024;

  //Serial.print("Xa: " + String(xAccl) + "  Ya: " + String(yAccl) + "  Za: " + String(zAccl) + "\n");


  // Räkna ut roll och vinklar
  float pitch = atan(float(xAccl) / float(zAccl));
  float roll = atan(float(yAccl) / float(zAccl));

  // Konvertera radianer till grader

  pitch = pitch * 180.0 / PI;
  roll = roll * 180.0 / PI;


  for (int i = 1; i <= 7; i++) {
    angle_x[i]=angle_x[i-1];
    angle_y[i]=angle_y[i-1];
  }

  angle_x[0] = pitch;
  angle_y[0] = roll;

  int styrvinkel_x = (angle_x[0] + angle_x[1] + angle_x[2] + angle_x[3] + angle_x[4] + angle_x[5] + angle_x[6] + angle_x[7]) / 8;
  int styrvinkel_y = (angle_y[0] + angle_y[1] + angle_y[2] + angle_y[3] + angle_y[4] + angle_y[5] + angle_y[6] + angle_y[7]) / 8;
  Serial.println("Pitch: " + String(pitch) + "  Roll: " + roll + "\n");

  //Accelerometern ger i en range som är -90 till 90, men servon behöver 0 till 180, därav +90// gammal kommentar
  servoX.write(map(styrvinkel_x, -90, 90, 0, 180));
  servoY.write(map(styrvinkel_y, -90, 90, 0, 180));


  delay(5);
}


int* getAccelerations() {

}
