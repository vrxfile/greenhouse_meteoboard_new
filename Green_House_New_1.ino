#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <pcf8574_esp.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_CCS811.h>
#include <Adafruit_BME280.h>
#include <BH1750FVI.h>
#include <VL53L0X.h>
#include <VEML6075.h>

// Точка доступа Wi-Fi
char ssid[] = "IOTIK";
char pass[] = "Terminator812";

// Расширитель портов
PCF857x pcf8574(0x20, &Wire);

// Датчик освещенности
BH1750FVI bh1750;

// Датчик ускорения/магнитного поля
Adafruit_LSM303_Mag_Unified mag303 = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel303 = Adafruit_LSM303_Accel_Unified(54321);

// Датчик температуры/влажности и атмосферного давления
Adafruit_BME280 bme280_1;
Adafruit_BME280 bme280_2;

// Аналогово-цифровой преобразователь
Adafruit_ADS1115 ads1115_1(0x48);
Adafruit_ADS1115 ads1115_2(0x49);

// Датчик ЛОВ и eCO2
Adafruit_CCS811 ccs811;

// Датчик УФ излучения Солнца
VEML6075 veml6075;

// Лазерный датчик расстояния
VL53L0X vl53l0x;

// Напряжение аккумулятора
#define ACC_VOLTAGE_PIN A0

// Датчик влажности и температуры почвы емкостной
const float air_value    = 83900.0;
const float water_value  = 45000.0;
const float moisture_0   = 0.0;
const float moisture_100 = 100.0;

// API key для Blynk
char auth[] = "08b6358f69b342c6840504fa054ccad3";
IPAddress blynk_ip(139, 59, 206, 133);

// Периоды для таймеров
#define BME280_UPDATE_TIME     5100
#define BH1750_UPDATE_TIME     5200
#define LSM303_UPDATE_TIME     5300
#define CCS811_UPDATE_TIME     5400
#define VEML6075_UPDATE_TIME   5500
#define VL53L0X_UPDATE_TIME    5600
#define ADS1115_UPDATE_TIME    5700
#define VOLTAGE_UPDATE_TIME    5800
#define CONTROL_UPDATE_TIME    60000

// Таймеры
BlynkTimer timer_bme280;
BlynkTimer timer_bh1750;
BlynkTimer timer_lsm303;
BlynkTimer timer_ccs811;
BlynkTimer timer_veml6075;
BlynkTimer timer_vl53l0x;
BlynkTimer timer_ads1115;
BlynkTimer timer_voltage;
BlynkTimer timer_control;

// Состояния управляющих устройств
int in_water_valve  = 0;
int out_water_valve = 0;
int light_control   = 0;

// Параметры сенсоров для IoT сервера
#define sensorCount 31
char* sensorNames[] = {"soil_temp_1", "soil_temp_2", "soil_temp_3", "soil_temp_4",
                       "soil_hum_1", "soil_hum_2", "soil_hum_3", "soil_hum_4",
                       "air_temp_1", "air_temp_2", "air_hum_1", "air_hum_2",
                       "air_press_1", "air_press_2",
                       "sun_light", "sun_uva", "sun_uvb",
                       "tvoc_conc", "eco2_conc", "water_level",
                       "acc_x", "acc_y", "acc_z", "mag_x", "mag_y", "mag_z",
                       "in_water_timer", "out_water_timer", "light_timer",
                       "acc_voltage", "working_time"
                      };
float sensorValues[sensorCount];
// Номера датчиков
#define soil_temp_1       0
#define soil_temp_2       1
#define soil_temp_3       2
#define soil_temp_4       3
#define soil_hum_1        4
#define soil_hum_2        5
#define soil_hum_3        6
#define soil_hum_4        7
#define air_temp_1        8
#define air_temp_2        9
#define air_hum_1         10
#define air_hum_2         11
#define air_press_1       12
#define air_press_2       13
#define sun_light         14
#define sun_uva           15
#define sun_uvb           16
#define tvoc_conc         17
#define eco2_conc         18
#define water_level       19
#define acc_x             20
#define acc_y             21
#define acc_z             22
#define mag_x             23
#define mag_y             24
#define mag_z             25
#define in_water_timer    26
#define out_water_timer   27
#define light_timer       28
#define acc_voltage       29
#define working_time      30

void setup()
{
  // Инициализация последовательного порта
  Serial.begin(115200);
  delay(512);

  // Инициализация Blynk и Wi-Fi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  Blynk.begin(auth, ssid, pass, blynk_ip, 8442);
  delay(1024);

  // Инициализация интерфейса I2C
  Serial.println("Init I2C");
  Wire.begin(4, 5);
  Wire.setClock(10000L);
  delay(1024);

  // Инициализация расширителя портов
  Serial.println("Init PCF8574");
  pcf8574.begin();
  pcf8574.write(0, LOW);
  pcf8574.write(1, LOW);
  pcf8574.write(2, LOW);
  pcf8574.write(3, LOW);
  pcf8574.write(4, LOW);
  pcf8574.write(5, LOW);
  pcf8574.write(6, LOW);
  pcf8574.write(7, LOW);
  delay(1024);

  //Инициализация АЦП
  Serial.println("Init ADS1115");
  ads1115_1.setGain(GAIN_TWOTHIRDS);
  ads1115_2.setGain(GAIN_TWOTHIRDS);
  ads1115_1.begin();
  ads1115_2.begin();
  delay(1024);

  // Инициализация датчика BH1750
  Serial.println("Init BH1750");
  bh1750.begin();
  bh1750.setMode(Continuously_High_Resolution_Mode);
  delay(1024);

  // Инициализация датчика LSM303
  Serial.println("Init LSM303");
  mag303.enableAutoRange(true);
  bool mag_stat = mag303.begin();
  if (!mag_stat)
    Serial.println("Could not find a valid LSM303 sensor, check wiring!");
  bool acc_stat = accel303.begin();
  if (!acc_stat)
    Serial.println("Could not find a valid LSM303 sensor, check wiring!");
  delay(1024);

  // Инициализация датчика CCS811
  Serial.println("Init CCS811");
  bool ccs_stat = ccs811.begin();
  if (!ccs_stat)
    Serial.println("Could not find a valid CCS811 sensor, check wiring!");
  while (!ccs811.available());
  float ccs_temp = ccs811.calculateTemperature();
  ccs811.setTempOffset(ccs_temp - 25.0);
  delay(1024);

  // Инициализация датчика VEML6075
  Serial.println("Init VEML6075");
  bool veml_stat = veml6075.begin();
  if (!veml_stat)
    Serial.println("Could not find a valid VEML6075 sensor, check wiring!");
  delay(1024);

  // Инициализация датчиков BME280
  Serial.println("Init BME280");
  bool bme_stat_1 = bme280_1.begin(0x77);
  if (!bme_stat_1)
    Serial.println("Could not find a valid BME280 sensor #1, check wiring!");
  bool bme_stat_2 = bme280_2.begin(0x76);
  if (!bme_stat_2)
    Serial.println("Could not find a valid BME280 sensor #2, check wiring!");
  delay(1024);

  // Инициализация датчика VL53L0X
  Serial.println("Init VL53L0X");
#define LONG_RANGE
  //#define HIGH_SPEED
  //#define HIGH_ACCURACY
  vl53l0x.init();
  vl53l0x.setTimeout(512);
#if defined LONG_RANGE
  vl53l0x.setSignalRateLimit(0.1);
  vl53l0x.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  vl53l0x.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED
  vl53l0x.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  vl53l0x.setMeasurementTimingBudget(200000);
#endif
  delay(1024);

  // Однократный опрос датчиков
  Serial.println("Read BME280"); readSensorBME280();
  Serial.println("Read BH1750"); readSensorBH1750();
  Serial.println("Read LSM303"); readSensorLSM303();
  Serial.println("Read CCS811"); readSensorCCS811();
  Serial.println("Read VEML6075"); readSensorVEML6075();
  Serial.println("Read VL53L0X"); readSensorVL53L0X();
  Serial.println("Read ADS1115"); readSensorADS1115();
  Serial.println("Read VOLTAGE"); readAccVOLTAGE();

  // Вывод в терминал данных с датчиков
  printAllSensors();

  // Инициализация таймеров
  timer_bme280.setInterval(BME280_UPDATE_TIME, readSensorBME280);
  timer_bh1750.setInterval(BH1750_UPDATE_TIME, readSensorBH1750);
  timer_lsm303.setInterval(LSM303_UPDATE_TIME, readSensorLSM303);
  timer_ccs811.setInterval(CCS811_UPDATE_TIME, readSensorCCS811);
  timer_veml6075.setInterval(VEML6075_UPDATE_TIME, readSensorVEML6075);
  timer_vl53l0x.setInterval(VL53L0X_UPDATE_TIME, readSensorVL53L0X);
  timer_ads1115.setInterval(ADS1115_UPDATE_TIME, readSensorADS1115);
  timer_voltage.setInterval(VOLTAGE_UPDATE_TIME, readAccVOLTAGE);
  timer_control.setInterval(CONTROL_UPDATE_TIME, doControlTIMER);
}

void loop()
{
  Blynk.run();
  timer_bme280.run();
  timer_bh1750.run();
  timer_lsm303.run();
  timer_ccs811.run();
  timer_veml6075.run();
  timer_vl53l0x.run();
  timer_ads1115.run();
  timer_voltage.run();
  timer_control.run();
}

// Чтение датчика BME280
void readSensorBME280()
{
  sensorValues[air_temp_1] = bme280_1.readTemperature();
  sensorValues[air_hum_1] = bme280_1.readHumidity();
  sensorValues[air_press_1] = bme280_1.readPressure() * 7.5006 / 1000.0;
  sensorValues[air_temp_2] = bme280_2.readTemperature();
  sensorValues[air_hum_2] = bme280_2.readHumidity();
  sensorValues[air_press_2] = bme280_2.readPressure() * 7.5006 / 1000.0;
  Blynk.virtualWrite(V8, sensorValues[air_temp_1]); delay(25);
  Blynk.virtualWrite(V9, sensorValues[air_hum_1]); delay(25);
  Blynk.virtualWrite(V10, sensorValues[air_press_1]); delay(25);
  Blynk.virtualWrite(V11, sensorValues[air_temp_2]); delay(25);
  Blynk.virtualWrite(V12, sensorValues[air_hum_2]); delay(25);
  Blynk.virtualWrite(V13, sensorValues[air_press_2]); delay(25);
}

// Чтение датчика BH1750
void readSensorBH1750()
{
  sensorValues[sun_light] = bh1750.getAmbientLight();
  Blynk.virtualWrite(V14, sensorValues[sun_light]); delay(25);
}

// Чтение датчика LSM303
void readSensorLSM303()
{
  sensors_event_t acc_event;
  accel303.getEvent(&acc_event);
  sensors_event_t mag_event;
  mag303.getEvent(&mag_event);
  sensorValues[acc_x] = acc_event.acceleration.x;
  sensorValues[acc_y] = acc_event.acceleration.y;
  sensorValues[acc_z] = acc_event.acceleration.z;
  sensorValues[mag_x] = mag_event.magnetic.x;
  sensorValues[mag_y] = mag_event.magnetic.y;
  sensorValues[mag_z] = mag_event.magnetic.z;
  Blynk.virtualWrite(V20, sensorValues[acc_x]); delay(25);
  Blynk.virtualWrite(V21, sensorValues[acc_y]); delay(25);
  Blynk.virtualWrite(V22, sensorValues[acc_z]); delay(25);
  Blynk.virtualWrite(V23, sensorValues[mag_x]); delay(25);
  Blynk.virtualWrite(V24, sensorValues[mag_y]); delay(25);
  Blynk.virtualWrite(V25, sensorValues[mag_z]); delay(25);
}

// Чтение датчика CCS811
void readSensorCCS811()
{
  if (ccs811.available())
  {
    float t = ccs811.calculateTemperature();
    if (!ccs811.readData())
    {
      sensorValues[eco2_conc] = ccs811.geteCO2();
      sensorValues[tvoc_conc] = ccs811.getTVOC();
      Blynk.virtualWrite(V17, sensorValues[tvoc_conc]); delay(25);
      Blynk.virtualWrite(V18, sensorValues[eco2_conc]); delay(25);
    }
  }
}

// Чтение датчика VEML6075
void readSensorVEML6075()
{
  veml6075.poll();
  sensorValues[sun_uva] = veml6075.getUVA();
  sensorValues[sun_uvb] = veml6075.getUVB();
  Blynk.virtualWrite(V15, sensorValues[sun_uva]); delay(25);
  Blynk.virtualWrite(V16, sensorValues[sun_uvb]); delay(25);
}

// Чтение датчика VL53L0X
void readSensorVL53L0X()
{
  sensorValues[water_level] = vl53l0x.readRangeSingleMillimeters() / 1000.0;
  Blynk.virtualWrite(V19, sensorValues[water_level]); delay(25);
}

// Чтение аналоговых датчиков через АЦП
void readSensorADS1115()
{
  float adc1_1 = (float)ads1115_1.readADC_SingleEnded(0) * 6.144;
  float adc1_2 = (float)ads1115_1.readADC_SingleEnded(1) * 6.144;
  float adc1_3 = (float)ads1115_1.readADC_SingleEnded(2) * 6.144;
  float adc1_4 = (float)ads1115_1.readADC_SingleEnded(3) * 6.144;
  float adc2_1 = (float)ads1115_2.readADC_SingleEnded(0) * 6.144;
  float adc2_2 = (float)ads1115_2.readADC_SingleEnded(1) * 6.144;
  float adc2_3 = (float)ads1115_2.readADC_SingleEnded(2) * 6.144;
  float adc2_4 = (float)ads1115_2.readADC_SingleEnded(3) * 6.144;
  sensorValues[soil_hum_1] = map(adc1_1, air_value, water_value, moisture_0, moisture_100);
  sensorValues[soil_temp_1] = adc1_2 / 1000.0;
  sensorValues[soil_hum_2] = map(adc1_3, air_value, water_value, moisture_0, moisture_100);
  sensorValues[soil_temp_2] = adc1_4 / 1000.0;
  sensorValues[soil_hum_3] = map(adc2_1, air_value, water_value, moisture_0, moisture_100);
  sensorValues[soil_temp_3] = adc2_2 / 1000.0;
  sensorValues[soil_hum_4] = map(adc2_3, air_value, water_value, moisture_0, moisture_100);
  sensorValues[soil_temp_4] = adc2_4 / 1000.0;
  Blynk.virtualWrite(V0, sensorValues[soil_temp_1]); delay(25);
  Blynk.virtualWrite(V1, sensorValues[soil_temp_2]); delay(25);
  Blynk.virtualWrite(V2, sensorValues[soil_temp_3]); delay(25);
  Blynk.virtualWrite(V3, sensorValues[soil_temp_4]); delay(25);
  Blynk.virtualWrite(V4, sensorValues[soil_hum_1]); delay(25);
  Blynk.virtualWrite(V5, sensorValues[soil_hum_2]); delay(25);
  Blynk.virtualWrite(V6, sensorValues[soil_hum_3]); delay(25);
  Blynk.virtualWrite(V7, sensorValues[soil_hum_4]); delay(25);
}

// Чтение встроенного АЦП и измерение напряжения аккумулятора
void readAccVOLTAGE()
{
  float acc_u = analogRead(A0);
  sensorValues[acc_voltage] =  acc_u / 1023.0 * 5.0;
  Blynk.virtualWrite(V27, sensorValues[acc_voltage]); delay(25);
}

// Чтение EEPROM и установка счетчика времени работы

// Обратный отсчет таймеров
void doControlTIMER()
{
  in_water_valve = in_water_valve - 1;
  out_water_valve = out_water_valve - 1;
  light_control = light_control - 1;
  if (in_water_valve <= 0)
  {
    in_water_valve = 0;
    Blynk.virtualWrite(V104, LOW);
    delay(25);
    pcf8574.write(0, LOW);
  }
  if (out_water_valve <= 0)
  {
    out_water_valve = 0;
    Blynk.virtualWrite(V103, LOW);
    delay(25);
    pcf8574.write(1, LOW);
  }
  if (light_control <= 0)
  {
    light_control = 0;
    Blynk.virtualWrite(V105, LOW);
    delay(25);
    pcf8574.write(2, LOW);
  }
  Blynk.virtualWrite(V101, in_water_valve); delay(25);
  Blynk.virtualWrite(V100, out_water_valve); delay(25);
  Blynk.virtualWrite(V102, light_control); delay(25);
}

// Управление тамером полива с Blynk
BLYNK_WRITE(V100)
{
  out_water_valve = param.asInt();
  Serial.println(out_water_valve);
}

// Управление таймером набора воды с Blynk
BLYNK_WRITE(V101)
{
  in_water_valve = param.asInt();
  Serial.println(in_water_valve);
}

// Управление таймером освещения с Blynk
BLYNK_WRITE(V102)
{
  light_control = param.asInt();
  Serial.println(light_control);
}

// Управление поливом с Blynk
BLYNK_WRITE(V103)
{
  int pwr = param.asInt();
  Serial.println(pwr);
  pcf8574.write(1, pwr);
}

// Управление набором воды с Blynk
BLYNK_WRITE(V104)
{
  int pwr = param.asInt();
  Serial.println(pwr);
  pcf8574.write(0, pwr);
}

// Управление освещением с Blynk
BLYNK_WRITE(V105)
{
  int pwr = param.asInt();
  Serial.println(pwr);
  pcf8574.write(2, pwr);
}

// Print sensors data to terminal
void printAllSensors()
{
  for (int i = 0; i < sensorCount; i++)
  {
    Serial.print(sensorNames[i]);
    Serial.print(" = ");
    Serial.println(sensorValues[i]);
  }
  Serial.println();
}

