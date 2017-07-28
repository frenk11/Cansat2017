#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <string.h>
#include <Adafruit_BMP085.h>

#define L3G4200D_CTRL_REG1 0x20
#define L3G4200D_OUT_X_L 0x28
#define GYR_ADDRESS (0xD2 >> 1)
#define Register_2D 0x2D
#define Register_X0 0x32
#define Register_X1 0x33
#define Register_Y0 0x34
#define Register_Y1 0x35
#define Register_Z0 0x36
#define Register_Z1 0x37
#define address 0x1E
#define BMP085_ADDRESS 0x77
const unsigned int SET_PIN = 5; // Пин для конфигурации радиомодуля
const unsigned int CS_PIN = 6; //Пин включение радиомодуля

class GY801 {
public:    //датчик барометр, гироскоп, акселерометр GY801 
  void setup() {
    l3g4200dSetup();
    adxl345Setup();
    hmc5883lSetup();
    bmp085Setup();
  }

  void loop() {
    l3g4200dLoop();   
    adxl345Loop();
    hmc5883lLoop();
    bmp085Loop();
  }

  float speed_x, speed_y, speed_z;

  void l3g4200dSetup() {
    l3g4200dEnableDefault();
  }

  void l3g4200dLoop() {
    l3g4200dRead();
  }

  void l3g4200dEnableDefault() {
    // 0x0F = 0b00001111
    // Normal power mode, all axes enabled
    l3g4200dWriteReg(L3G4200D_CTRL_REG1, 0x0F);
  }

  void l3g4200dWriteReg(byte reg, byte value) {
    Wire.beginTransmission(GYR_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

  void l3g4200dRead() {
    Wire.beginTransmission(GYR_ADDRESS);
    Wire.write(L3G4200D_OUT_X_L | (1 << 7));
    Wire.endTransmission();
    Wire.requestFrom(GYR_ADDRESS, 6);
    while (Wire.available() < 6);
    uint8_t xla = Wire.read(), xha = Wire.read(), yla = Wire.read(), yha = Wire.read(), zla = Wire.read(), zha = Wire.read();
    speed_x = xha << 8 | xla;
    speed_y = yha << 8 | yla;
    speed_z = zha << 8 | zla;
  }


  int ADXAddress = 0xA7 >> 1, reading = 0, val = 0, X0, X1, X_out, Y0, Y1, Y_out, Z1, Z0, Z_out;
  double accel_x, accel_y, accel_z;

  void adxl345Setup() {
    Wire.beginTransmission(ADXAddress);
    Wire.write(Register_2D);
    Wire.write(8);
    Wire.endTransmission();
  }

  void adxl345Loop() {
    Wire.beginTransmission(ADXAddress);
    Wire.write(Register_X0);
    Wire.write(Register_X1);
    Wire.endTransmission();
    Wire.requestFrom(ADXAddress, 2);
    if (Wire.available() <= 2) {
      X0 = Wire.read();
      X1 = Wire.read();
      X1 = X1 << 8;
      X_out = X0 + X1;
    }
    Wire.beginTransmission(ADXAddress);
    Wire.write(Register_Y0);
    Wire.write(Register_Y1);
    Wire.endTransmission();
    Wire.requestFrom(ADXAddress, 2);
    if (Wire.available() <= 2) {
      Y0 = Wire.read();
      Y1 = Wire.read();
      Y1 = Y1 << 8;
      Y_out = Y0 + Y1;
    }
    Wire.beginTransmission(ADXAddress);
    Wire.write(Register_Z0);
    Wire.write(Register_Z1);
    Wire.endTransmission();
    Wire.requestFrom(ADXAddress, 2);
    if (Wire.available() <= 2) {
      Z0 = Wire.read();
      Z1 = Wire.read();
      Z1 = Z1 << 8;
      Z_out = Z0 + Z1;
    }
    accel_x = X_out / 256.0;
    accel_y = Y_out / 256.0;
    accel_z = Z_out / 256.0;
  }

  int magnet_x, magnet_y, magnet_z;

  void hmc5883lSetup() {
    Wire.beginTransmission(address);
    Wire.write(0x02);
    Wire.write(0x00);
    Wire.endTransmission();
  }

  void hmc5883lLoop() {
    Wire.beginTransmission(address);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(address, 6);
    if (6 <= Wire.available()) {
      magnet_x = Wire.read() << 8;
      magnet_x |= Wire.read();
      magnet_z = Wire.read() << 8;
      magnet_z |= Wire.read();
      magnet_y = Wire.read() << 8;
      magnet_y |= Wire.read();
    }
  }

  const unsigned char OSS = 0;
  int ac1, ac2, ac3, b1, b2, mb, mc, md;
  unsigned int ac4, ac5, ac6;
  float temp;
  long pres, b5;

  void bmp085Setup() {
    bmp085Calibration();
  }

  void bmp085Loop() {
    temp = bmp085GetTemperature(bmp085ReadUT()) * 0.1;
    pres = bmp085GetPressure(bmp085ReadUP());
  }

  void bmp085Calibration() {
    ac1 = bmp085ReadInt(0xAA);
    ac2 = bmp085ReadInt(0xAC);
    ac3 = bmp085ReadInt(0xAE);
    ac4 = bmp085ReadInt(0xB0);
    ac5 = bmp085ReadInt(0xB2);
    ac6 = bmp085ReadInt(0xB4);
    b1 = bmp085ReadInt(0xB6);
    b2 = bmp085ReadInt(0xB8);
    mb = bmp085ReadInt(0xBA);
    mc = bmp085ReadInt(0xBC);
    md = bmp085ReadInt(0xBE);
  }

  short bmp085GetTemperature(unsigned int ut) {
    long x1, x2;
    x1 = (((long)ut - (long)ac6) * (long)ac5) >> 15;
    x2 = ((long)mc << 11) / (x1 + md);
    b5 = x1 + x2;
    return ((b5 + 8) >> 4);
  }

  long bmp085GetPressure(unsigned long up) {
    long x1, x2, x3, b3, b6, p;
    unsigned long b4, b7;
    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6) >> 12) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((long)ac1) * 4 + x3) << OSS) + 2) >> 2;
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;
    b7 = ((unsigned long)(up - b3) * (50000 >> OSS));
    if (b7 < 0x80000000)
      p = (b7 << 1) / b4;
    else
      p = (b7 / b4) << 1;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;
    return p;
  }

  int bmp085ReadInt(unsigned char a)
  {
    unsigned char msb, lsb;
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(a);
    Wire.endTransmission();
    Wire.requestFrom(BMP085_ADDRESS, 2);
    while (Wire.available() < 2);
    msb = Wire.read();
    lsb = Wire.read();
    return (int)msb << 8 | lsb;
  }

  unsigned int bmp085ReadUT() {
    unsigned int ut;
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(0xF4);
    Wire.write(0x2E);
    Wire.endTransmission();
    delay(5);
    ut = bmp085ReadInt(0xF6);
    return ut;
  }

  unsigned long bmp085ReadUP() {
    unsigned char msb, lsb, xlsb;
    unsigned long up = 0;
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(0xF4);
    Wire.write(0x34 + (OSS << 6));
    Wire.endTransmission();
    delay(2 + (3 << OSS));
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(0xF6);
    Wire.endTransmission();
    Wire.requestFrom(BMP085_ADDRESS, 3);
    while (Wire.available() < 3);
    msb = Wire.read();
    lsb = Wire.read();
    xlsb = Wire.read();
    up = (((unsigned long)msb << 16) | ((unsigned long)lsb << 8) | (unsigned long)xlsb) >> (8 - OSS);
    return up;
  }

} gy801;

class help {
public:
  double alt, vl;
  int c = 0;
  bool d = false;
  void calcAltitude(long sp = 98776) {
    if (d)
      alt = 44330 * (1 - pow((double)gy801.pres / (double)sp, 0.19)) - c;
    else {
      d = true;
      //-//
      calcAltitude(sp);
      c = alt;
      alt = 0;
    }
  }

  void calcVl() {
    vl = sqrt(pow(gy801.accel_x, 2) + pow(gy801.accel_y, 2) + pow(gy801.accel_z, 2));
  }
} help;

//SERVO & POINTS//
class points {
public:
  int pin = 3; //пин для фоторезистора (аналоговый)
  int value;
  bool launch = false;
  bool separate = false;
  bool recovery = false;
  bool landing = false;
  Servo servo;

  void setup() {
    servo.attach(10);
  }

  void loop() {
    value = analogRead(pin);
    if (help.alt >= 2.0D && help.vl >= 1.9D) {
      launch = true;
    }
    if (value >= 1000) {
      digitalWrite(7, HIGH);
      tone(8, 500);
      separate = true;
      delay(2000);
      servo.write(60);
      recovery = true;
      delay(2000);
      servo.detach();
    }
    if (gy801.accel_x <= 0.7D && gy801.accel_y <= 0.7D && gy801.accel_z <= 0.7D && launch == true) {
      landing = true;
    }
  }
}
points;

class sd {
public:
  File file;

  void setup() {
    SD.begin(4);
  }

  void returnDataSD() {
    file.print("10;"); file.print(millis() + String(";")); file.print(help.alt + String(";"));
    file.print(gy801.accel_x + String(";")); file.print(gy801.accel_y + String(";")); file.print(gy801.accel_z + String(";"));
    file.print(gy801.speed_x + String(";")); file.print(gy801.speed_y + String(";")); file.print(gy801.speed_z + String(";"));
    file.print(gy801.magnet_x + String(";")); file.print(gy801.magnet_y + String(";")); file.print(gy801.magnet_z + String(";"));
    file.print(gy801.pres + String(";")); file.print(gy801.temp + String(";"));
    //launch
    file.print((int)points.launch + String(";"));
    //separate & recovery
    file.print((int)points.separate + String(";"));
    file.print((int)points.recovery + String(";"));
    //landing
    file.print((int)points.landing + String(";"));
    file.println();
  }



  void loop() {
    file = SD.open("data_fly.txt", FILE_WRITE);
    if (file) {
      returnDataSD();
      file.close();
    }
  }
} sd;

class wireless {
public:   
  void setup() {
    Serial1.begin(9600);
    pinMode(SET_PIN, OUTPUT);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    delay(100);
    configure_transmitter();
    }
  void loop() {
    mess();
    delay(100);
    }
  void configure_transmitter() {
    byte config[17] = { 0xAA,0xFA,0x03,0x13,0x01,0x03,0x07,0x03,0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00 };

    //Перевод пина конфигурации в режим настройки
    digitalWrite(SET_PIN, LOW);
    delay(100);

    //Передача массива с настройкой
    Serial1.write(config, 17);
    delay(50);

    // Перевод пина конфигурации в режим передачи
    digitalWrite(SET_PIN, HIGH);
  }

  void mess() {
    String telemetry = "10;"
                      + String(millis()) + ";"
                      + String(help.alt) + ";"
                      + String(help.vl) + ";"
                      + String(points.launch) + ";"
                      + String(points.separate) + ";"
                      + String(points.recovery) + ";"
                      + String(points.landing) + ";";
    Serial1.println(telemetry);
    delay(200);
  }
} wireless;

void setup() {
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(7, HIGH);
  tone(8, 500);
  delay(1000);
  digitalWrite(7, LOW);
  tone(8, 0);
  delay(1000);
  digitalWrite(7, HIGH);
  tone(8, 500);
  delay(1000);
  digitalWrite(7, LOW);
  tone(8, 0);
  delay(1000);
  digitalWrite(7, HIGH);
  tone(8, 500);
  delay(1000);
  digitalWrite(7, LOW);
  tone(8, 0);
  Wire.begin();
  gy801.setup();
  points.setup();
  sd.setup();
  wireless.setup();
}

void loop() {
  help.calcVl();
  help.calcAltitude();
  gy801.loop();
  points.loop();
  sd.loop();
  wireless.loop();
  delay(200);
}
