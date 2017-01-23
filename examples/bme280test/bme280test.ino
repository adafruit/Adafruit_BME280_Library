/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

void print_temperature()
{
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");
}

void print_pressure()
{
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa"); 
}

void print_altitude()
{
    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
}

void print_humidity()
{
    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
}

// Weather station scenario
void weather_station()
{
  Serial.println(" -- Weather station scenario -- ");
  Serial.println(" Forced mode, freq 1Hz, no oversampling, filter off ");
  for (int i = 0; i < 10; ++i)
  {
    // Forced mode as recommended for weather station usage by Bosch (sec 5.5.1)
    // Settings have to be written to the sensor before every sampling
    bme.setTemperatureSampling(Adafruit_BME280::Sampling_x1);
    bme.setPressureSampling(Adafruit_BME280::Sampling_x1);
    bme.setHumiditySampling(Adafruit_BME280::Sampling_x1);
    bme.setFilter(Adafruit_BME280::Filter_Off);
    bme.setMode(Adafruit_BME280::Forced);
    bme.begin();

    print_temperature();
    print_pressure();
    print_altitude();
    print_humidity();
    Serial.println();
    
    // Actual suggested rate is 1/60Hz (1 min)
    delay(1e3); // 60e3
  }
}

// Humidity sensing scenario
void humidity_sensing()
{
  Serial.println(" -- Humidity sensing scenario -- ");
  Serial.println(" Forced mode, freq 1Hz, no pressure, no oversampling, filter off ");
  for (int i = 0; i < 10; ++i)
  {
    bme.setTemperatureSampling(Adafruit_BME280::Sampling_x1);
    bme.setPressureSampling(Adafruit_BME280::Sampling_None);
    bme.setHumiditySampling(Adafruit_BME280::Sampling_x1);
    bme.setFilter(Adafruit_BME280::Filter_Off);
    bme.setMode(Adafruit_BME280::Forced);
    bme.begin();

    print_temperature();
    // You can try and read the pressure but it will be garbage !
    //print_pressure();
    print_altitude();
    print_humidity();
    Serial.println();
    
    delay(1e3);
  }
}

// Indoor navigation scenario
void indoor_navigation()
{
  Serial.println(" -- Indoor navigation scenario -- ");
  Serial.println(" Normal mode, 25Hz, pressure x16, temp x2, humidity x1, filter x16 ");

  bme.setTemperatureSampling(Adafruit_BME280::Sampling_x2);
  bme.setPressureSampling(Adafruit_BME280::Sampling_x16);
  bme.setHumiditySampling(Adafruit_BME280::Sampling_x1);
  bme.setFilter(Adafruit_BME280::Filter_x16);
  bme.setStandbyDuration(Adafruit_BME280::Duration_500us);
  bme.setMode(Adafruit_BME280::Normal);
  bme.begin();
  
  for (int i = 0; i < 10; ++i)
  {
    print_temperature();
    print_pressure();
    print_altitude();
    print_humidity();
    Serial.println();

    // Suggested rate is 25Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
    // T_ovs = 2
    // P_ovs = 16
    // H_ovs = 1
    // = 40ms (25Hz)
    // with standby time that should really be 24.16913... Hz
    delay(41);
  }
}

// Gaming scenario
void gaming_scenario()
{
  Serial.println(" -- Gaming scenario -- ");
  Serial.println(" Normal mode, 83Hz, pressure x4, temp x1, humidity none, filter x16 ");

  bme.setTemperatureSampling(Adafruit_BME280::Sampling_x1);
  bme.setPressureSampling(Adafruit_BME280::Sampling_x4);
  bme.setHumiditySampling(Adafruit_BME280::Sampling_None);
  bme.setFilter(Adafruit_BME280::Filter_x16);
  bme.setStandbyDuration(Adafruit_BME280::Duration_500us);
  bme.setMode(Adafruit_BME280::Normal);
  bme.begin();
  
  for (int i = 0; i < 10; ++i)
  {
    print_temperature();
    print_pressure();
    print_altitude();
    print_humidity();
    Serial.println();

    // Suggested rate is 83Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5)
    // T_ovs = 1
    // P_ovs = 4
    // = 11.5ms + 0.5ms standby
    delay(12);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println(F("BME280 test"));

  weather_station();
  humidity_sensing();
  indoor_navigation();
  gaming_scenario();
}

void loop()
{

}
