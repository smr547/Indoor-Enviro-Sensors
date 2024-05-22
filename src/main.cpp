// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"

#define APP_NAME "Indoor Environment Sensors"
#define HOST_NAME "IndoorSensors"

using namespace sensesp;

// void setupGypsyParameter();

reactesp::ReactESP app;
Adafruit_BME280 bme;  // I2C

int report_interval_ms = 5000;

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  Wire.begin();
  unsigned status;

  // default settings
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println(
        "Could not find a valid BME280 sensor, check wiring, address, "
        "sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print(
        "        ID of 0xFF probably means a bad address, a BMP 180 or BMP "
        "085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname(HOST_NAME)
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

  /************************************************************************************
   * Atmospheric pressure in hPa *
   ************************************************************************************
   */
  auto* pressureSensor = new RepeatSensor<float>(
      report_interval_ms, []() { return bme.readPressure() / 100.; });

  // Connect the pressureSensor to Signal K output. This will publish the
  // pressure value to the Signal K on a regular basis
  pressureSensor->connect_to(new SKOutputFloat(
      "environment.indoor.pressure",  // Signal K path
      "/sensors/indoor/pressure",     // configuration path, used in the
                                      // web UI and for storing the
                                      // configuration
      new SKMetadata("hPa",           // Define output units
                     "Hectopascals")  // Value description
      ));

  /************************************************************************************
   * Temperature degrees Kelvin *
   ************************************************************************************
   */
  auto* tempSensor = new RepeatSensor<float>(
      report_interval_ms, []() { return bme.readTemperature() + 273.15; });

  // Connect the pressureSensor to Signal K output. This will publish the
  // pressure value to the Signal K on a regular basis
  tempSensor->connect_to(new SKOutputFloat(
      "environment.indoor.temp",        // Signal K path
      "/sensors/indoor/temp",           // configuration path, used in the
                                        // web UI and for storing the
                                        // configuration
      new SKMetadata("K",               // Define output units
                     "degrees Kelvin")  // Value description
      ));

  /************************************************************************************
   * Humidity Sensor *
   ************************************************************************************
   */
  auto* humiditySensor = new RepeatSensor<float>(
      report_interval_ms, []() { return bme.readHumidity() / 100.; });

  // Connect the pressureSensor to Signal K output. This will publish the
  // pressure value to the Signal K on a regular basis
  humiditySensor->connect_to(new SKOutputFloat(
      "environment.indoor.humidity",  // Signal K path
      "/sensors/indoor/humidity",     // configuration path, used in the
                                      // web UI and for storing the
                                      // configuration
      new SKMetadata("ratio",         // Define output units
                     "Humidity as a fraction of one")  // Value description
      ));

  // Add an observer that prints out the current value of the analog input
  // every time it changes.
  // pressureSensor->attach(
  //     []() { debugD("Pressure: %f", pressureSensor->get()); });

  // Start networking, SK server connections and other SensESP
  // internals

  sensesp_app->start();
}

void loop() { app.tick(); }
