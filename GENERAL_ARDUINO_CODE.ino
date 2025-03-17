#include <AsyncStream.h>
#include "Parser.h"
AsyncStream<2> serial(&Serial, '\n');
//(,pres,3452135,)
#include <MechaQMC5883.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <TinyGPS++.h> // Для работы с GPS
#include <SoftwareSerial.h>
#include <Wire.h> // Для работы с I2C
#include <I2Cdev.h> // Для работы с MPU-6050
#include <GyverBME280.h> // Для работы с BME280

// Пины для управления двигателем
const int IN1 = 2;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 7;
const int ENA = 3;
const int ENB = 6;

// Пины для GPS
const int GPS_RX = 8; // RX модуля GPS
const int GPS_TX = 9; // TX модуля GPS

// Создаем объект для работы с BME280
GyverBME280 bme;

// Создаем объект для работы с MPU-6050
MPU6050 mpu;

// Создаем объект для работы с GPS
TinyGPSPlus gps;

// Создаем объект SoftwareSerial для GPS
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// Создаем объект для работы с QMC5883L

MechaQMC5883 compass;

// Переменные для работы с DMP
uint8_t fifoBuffer[64]; // Буфер FIFO

// Массив для хранения данных с датчиков
//float sensorData[19]; // Температура, влажность, давление, ускорение X, Y, Z, угол X, угол Y, широта, долгота, количество спутников, высота, время, дата, курс, магнитная индукция, азимут

void setup() {
  serial.setEOL(',');
  compass.setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_256);
  // Настройка пинов как выходов
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Инициализация Serial для отладки
  Serial.begin(9600);

  // Инициализация GPS
  gpsSerial.begin(9600);

  // Инициализация датчика BME280
  if (!bme.begin()) {
    
    while (1);
  }
  

  // Инициализация MPU-6050 и DMP
  Wire.begin();
  mpu.initialize();
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    
  } else {
    
    while (1);
  }

  // Инициализация QMC5883L
  compass.init();
  
}

void loop() {
  static uint32_t tmr;
  if (millis() - tmr >= 11) { // Таймер на 11 мс
    // Чтение данных с BME280
    Serial.print("(,");
    Serial.print("tmp,");
    Serial.print(bme.readTemperature()); // Температура
    Serial.println(",)");

    Serial.print("(,");
    Serial.print("hum,");
    Serial.print(bme.readHumidity()); // Влажность
    Serial.println(",)");

    Serial.print("(,");
    Serial.print("pre,");
    Serial.print(bme.readPressure() / 100.0F); // Давление в гПа
    Serial.println(",)");

    // Чтение данных с MPU-6050
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // Переменные для расчёта
      Quaternion q;
      VectorFloat gravity;
      float ypr[3]; // Углы: yaw, pitch, roll

      // Расчёты
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // Ускорение по осям
      VectorInt16 aa; // Вектор ускорения
      mpu.dmpGetAccel(&aa, fifoBuffer);
      Serial.print("(,");
      Serial.print("axx,");
      Serial.print(aa.x / 16384.0); // Ускорение по X
      Serial.println(",)");

      Serial.print("(,");
      Serial.print("axy,");
      Serial.print(aa.y / 16384.0); // Ускорение по Y
      Serial.println(",)");

      Serial.print("(,");
      Serial.print("axz,");
      Serial.print(aa.z / 16384.0); // Ускорение по Z
      Serial.println(",)");


      // Углы наклона (pitch и roll)
      Serial.print("(,");
      Serial.print("pit,");
      Serial.print(ypr[1] * 180 / M_PI); // Угол наклона по X (pitch)
      Serial.println(",)");

      Serial.print("(,");
      Serial.print("rol,");
      Serial.print(ypr[2] * 180 / M_PI); // Угол наклона по Y (roll)
      Serial.println(",)");
    }

    // Чтение данных с GPS
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    // Запись данных GPS в массив
    if (gps.location.isValid()) {
      Serial.print("(,");
      Serial.print("lat,");
      Serial.print(gps.location.lat()); // Широта
      Serial.println(",)");

      Serial.print("(,");
      Serial.print("lng,");
      Serial.print(gps.location.lng()); // Долгота
      Serial.println(",)");

    } else {
      Serial.print("(,");
      Serial.print("lat,");
      Serial.print(0.0); 
      Serial.println(",)");

      Serial.print("(,");
      Serial.print("lng,");
      Serial.print(0.0); 
      Serial.println(",)"); // Нет данных
    }

    Serial.print("(,");
    Serial.print("sat,");
    Serial.print(gps.satellites.value()); // спутники
    Serial.println(",)");

    Serial.print("(,");
    Serial.print("alt,");
    Serial.print(gps.altitude.meters()); // высота над уровнем моря
    Serial.println(",)");


    // Время и дата
    if (gps.time.isValid()) {
      Serial.print("(,");
      Serial.print("time,");
      Serial.print(gps.time.hour() * 10000 + gps.time.minute() * 100 + gps.time.second()); // Время (HHMMSS)
      Serial.println(",)");
    } else {
      Serial.print("(,");
      Serial.print("time,");
      Serial.print(0.0); 
      Serial.println(",)"); // Нет данных
    }

    if (gps.date.isValid()) {
      Serial.print("(,");
      Serial.print("date,");
      Serial.print(gps.date.year() * 10000 + gps.date.month() * 100 + gps.date.day()); // Дата (YYYYMMDD)
      Serial.println(",)");
    } else {
      Serial.print("(,");
      Serial.print("date,");
      Serial.print(0.0); 
      Serial.println(",)"); // Нет данных
    }

    // Курс (направление движения)
    if (gps.course.isValid()) {
      Serial.print("(,");
      Serial.print("courseg,");
      Serial.print(gps.course.deg()); // Курс в градусах
      Serial.println(",)");
    } else {
      Serial.print("(,");
      Serial.print("courseg,");
      Serial.print(0.0); 
      Serial.println(",)"); // Нет данных
    }

    // Чтение данных с QMC5883L
    int x, y, z;
    int azimuth;
    //float azimuth; //is supporting float too
    compass.read(&x, &y, &z,&azimuth);
    Serial.print("(,");
    Serial.print("x,");
    Serial.print(x);
    Serial.println(",)");

    Serial.print("(,");
    Serial.print("y,");
    Serial.print(y);
    Serial.println(",)");
    
    Serial.print("(,");
    Serial.print("z,");
    Serial.print(z);
    Serial.println(",)");

    Serial.print("(,");
    Serial.print("az,");
    Serial.print(azimuth); // Азимут (угол относительно магнитного севера)
    Serial.println(",)");

    

    tmr = millis(); // Сброс таймера
  }
  
  // Обработка команд Bluetooth
  if (serial.available()) {
    Parser data(serial.buf, ',');  // отдаём парсеру
    int ints[2];           // массив для численных данных
    data.parseInts(ints);

    // Управление двигателями в зависимости от команды
    switch (ints[0]) {
      case 0: // Вперед
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, ints[1]);
        analogWrite(ENB, ints[1]);
        break;
      case 1: // Назад
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENA, ints[1]);
        analogWrite(ENB, ints[1]);
        break;
      case 2: // Вправо
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENA, ints[1]);
        analogWrite(ENB, ints[1]);
        break;
      case 3: // Влево
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, ints[1]);
        analogWrite(ENB, ints[1]);
        break;
      case 4: // Стоп
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
        break;
        
    }
  } 
}