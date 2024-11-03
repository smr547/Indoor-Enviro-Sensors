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
#include <sensesp/transforms/debounce.h>
#include <sensesp/transforms/lambda_transform.h>
#include <sensesp/transforms/linear.h>

#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/constant_sensor.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/integrator.h"
#include "sensesp_app_builder.h"

#define BME_REPORT_INTERVAL_MS 5000

#define APP_NAME "Indoor Environment Sensors"
#define HOST_NAME "IndoorSensors"
#define RAIN_PIN 12
#define RAIN_DEBOUNCE_MS 15
#define RAIN_REPORT_MS 300000
#define RAIN_REPORT_MS_PATH "/rain/report_period_ms"
#define RAIN_MM_PER_PULSE 0.18
#define RAIN_MM_PER_PULSE_PATH "/rain/mm_per_pulse"
#define SK_RAIN_PATH "environment.rain.5min"
#define SK_RAIN_CONFIG_PATH "/rain/sk_path"

#define ANEMOMETER_PIN 35
#define WIND_INSTANTANEOUS_SPEED_REPORT_MS 3000
#define WIND_DEBOUNCE_MS 15
#define SK_TRUE_WINDSPEED_PATH "environment.wind.speedTrue"
#define SK_TRUE_WINDSPEED_CONFIG_PATH "/wind/speedTrue/SK_path"

#define WIND_DIRECTION_PIN 33
#define WIND_DIRECTION_REPORT_MS 5000
#define SK_TRUE_WIND_DIRECTION_PATH "environment.wind.directionTrue"

#define AVG_WIND_REPORT_INTERVAL_MS 60 * 10 * 1000  // 10 mins
#define WIND_PPS_TO_M_PER_SEC 1.02F

#define GUST_MAX_REPORTS 4

using namespace sensesp;

// void setupGypsyParameter();

reactesp::ReactESP app;
Adafruit_BME280 bme;  // I2C

// variables for average wind computation
float sumDx = 0.0;
float sumDy = 0.0;
int dataPoints = 0;

// variables for gust computation

int maxGustCount = 0;
int gustPulseCount = 0;
int reports = 0;

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname(HOST_NAME)
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    ->set_wifi("Bertie", "Ookie1234")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

  UIOutput<String>* app_version_url_ = new UIOutput<String>(
      "Application source code",
      "https://github.com/smr547/Indoor-Enviro-Sensors", "Software", 1901);

  Wire.begin();
  unsigned status;

  // default settings
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  status = bme.begin(0x77, &Wire);
  if (!status) {
    Serial.println(
        "Could not find a valid BME280 sensor, check wiring, address, "
        "sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
  } else {
    Serial.print("Found BME280 sensor at: 0x");
    Serial.println(bme.sensorID(), 16);

    /************************************************************************************
     * Atmospheric pressure in hPa *
     ************************************************************************************
     */
    auto* pressureSensor = new RepeatSensor<float>(
        BME_REPORT_INTERVAL_MS, []() { return bme.readPressure() / 100.; });

    // Connect the pressureSensor to Signal K output. This will publish the
    // pressure value to the Signal K on a regular basis
    pressureSensor->connect_to(new SKOutputFloat(
        "environment.outside.pressure",  // Signal K path
        "/sensors/pressure",             // configuration path, used in the
                                         // web UI and for storing the
                                         // configuration
        new SKMetadata("hPa",            // Define output units
                       "Hectopascals")   // Value description
        ));

    /************************************************************************************
     * Temperature degrees Kelvin *
     ************************************************************************************
     */
    auto* tempSensor = new RepeatSensor<float>(BME_REPORT_INTERVAL_MS, []() {
      return bme.readTemperature() + 273.15;
    });

    // Connect the pressureSensor to Signal K output. This will publish the
    // pressure value to the Signal K on a regular basis
    tempSensor->connect_to(new SKOutputFloat(
        "environment.outside.temperature",  // Signal K path
        "/sensors/temp",                    // configuration path, used in the
                                            // web UI and for storing the
                                            // configuration
        new SKMetadata("K",                 // Define output units
                       "degrees Kelvin")    // Value description
        ));

    /************************************************************************************
     * Humidity Sensor *
     ************************************************************************************
     */
    auto* humiditySensor = new RepeatSensor<float>(
        BME_REPORT_INTERVAL_MS, []() { return bme.readHumidity() / 100.; });

    // Connect the pressureSensor to Signal K output. This will publish the
    // pressure value to the Signal K on a regular basis
    humiditySensor->connect_to(new SKOutputFloat(
        "environment.outside.relativeHumidity",  // Signal K path
        "/sensors/humidity",     // configuration path, used in the
                                 // web UI and for storing the
                                 // configuration
        new SKMetadata("ratio",  // Define output units
                       "Humidity as a fraction of one")  // Value description
        ));
  }

  /************************************************************************************
   * Rain Sensor *
   *************************************************************************************/

  auto* rainMetadata = new SKMetadata();
  rainMetadata->units_ = "mm";
  rainMetadata->description_ =
      "Amount of rain falling in last measure interval";
  rainMetadata->display_name_ = "Rain volume";
  rainMetadata->short_name_ = "Rain volume";

  auto* rainPulseCounter = new DigitalInputDebounceCounter(
      RAIN_PIN, INPUT_PULLUP, FALLING, RAIN_REPORT_MS, RAIN_DEBOUNCE_MS,
      RAIN_REPORT_MS_PATH);
  auto* rainMultiplier =
      new Linear(RAIN_MM_PER_PULSE, 0.0F, RAIN_MM_PER_PULSE_PATH);
  auto* rainReporter =
      new SKOutputFloat(SK_RAIN_PATH, SK_RAIN_CONFIG_PATH, rainMetadata);

  rainPulseCounter->connect_to(rainMultiplier)->connect_to(rainReporter);

  /************************************************************************************
   * Wind speed sensor (anemometer) *
   ************************************************************************************/

  // auto* rainMetadata = new SKMetadata();
  //  rainMetadata->units_ = "mm";
  //  rainMetadata->description_ =
  //      "Amount of rain falling in last measure interval";
  //  rainMetadata->display_name_ = "Rain volume";
  //  rainMetadata->short_name_ = "Rain volume";

  auto* windPulseCounter = new DigitalInputDebounceCounter(
      ANEMOMETER_PIN, INPUT_PULLUP, FALLING, WIND_INSTANTANEOUS_SPEED_REPORT_MS,
      WIND_DEBOUNCE_MS);
  auto* windSpeedScaler =
      new Linear((float)(WIND_PPS_TO_M_PER_SEC * 1000 /
                         WIND_INSTANTANEOUS_SPEED_REPORT_MS),
                 0.0F, "");
  auto* instataneousWindSpeedReporter =
      new SKOutputFloat(SK_TRUE_WINDSPEED_PATH, SK_TRUE_WINDSPEED_CONFIG_PATH);

  windPulseCounter->connect_to(windSpeedScaler)
      ->connect_to(instataneousWindSpeedReporter);

  /************************************************************************************
   * Wind direction sensor *
   ************************************************************************************/
  auto* windDirectionReporter = new SKOutputFloat(SK_TRUE_WIND_DIRECTION_PATH);

  // Wind direction comes to us via ADC. A count of zero implies due north,
  // full count is 359 degrees. Signal K expects this in radians, so we scale
  // it to -PI -> 0 -> +PI.
  auto* windDirectionSensor =
      new AnalogInput(WIND_DIRECTION_PIN, WIND_DIRECTION_REPORT_MS, "", 2 * PI);
  windDirectionSensor
      ->connect_to(new LambdaTransform<float, float>(
          [](float inRadians) {
            // Convert from true wind direction to apparent wind
            // direction with the boat heading true north.
            return inRadians < PI ? inRadians : (inRadians - 2 * PI);
          },
          "" /* no config */))
      ->connect_to(windDirectionReporter);

  /************************************************************************************
   * Average wind speed computation -- over 10 minutes *
   ************************************************************************************/

  auto* averageWindSpeedIntegrator = new IntegratorT<int, int>();
  windPulseCounter->connect_to(averageWindSpeedIntegrator);

  auto* averageWindSpeedSensor = new RepeatSensor<float>(
      AVG_WIND_REPORT_INTERVAL_MS, [averageWindSpeedIntegrator]() {
        int pulsesIn10Mins = averageWindSpeedIntegrator->get();
        averageWindSpeedIntegrator->reset();
        return (float)pulsesIn10Mins * WIND_PPS_TO_M_PER_SEC / 600.0F;
        // return (float)(pulsesIn10Mins * WIND_PPS_TO_M_PER_SEC * 1000 /
        //               AVG_WIND_REPORT_INTERVAL_MS);
      });

  SKMetadata* windSpeedMetadata =
      new SKMetadata("m/s", "Average wind speed over 10 minute interval",
                     "Avg Speed", "Avg Speed");

  auto* averageWindSpeedReporter =
      new SKOutputFloat("environment.wind.speedAverage", windSpeedMetadata);
  averageWindSpeedSensor->connect_to(averageWindSpeedReporter);

  /************************************************************************************
   * Average wind direction computation -- over 10 minutes *
   ************************************************************************************/
  /*
    float sumDx = 0.0;
    float sumDy = 0.0;
    int dataPoints = 0;
  */
  auto* directionAverager = new LambdaTransform<float, float>(
      [](float inRadians) {
        float dx = sin(inRadians);
        float dy = cos(inRadians);
        sumDx += dx;
        sumDy += dy;
        dataPoints += 1;
        // Convert from true wind direction to apparent wind
        // direction with the boat heading true north.
        return 0.0;
      },
      "" /* no config */);

  windDirectionReporter->connect_to(directionAverager);

  auto* averageWindDirectionSensor =
      new RepeatSensor<float>(AVG_WIND_REPORT_INTERVAL_MS, []() {
        float result = atan2(sumDx, sumDy);
        sumDx = 0.0;
        sumDy = 0.0;
        dataPoints = 0;

        return result;
      });

  SKMetadata* windAvgDirectionMetadata =
      new SKMetadata("rad", "Average wind direction over 10 minute interval",
                     "Avg direction", "Avg direction");

  auto* averageWindDirectionReporter = new SKOutputFloat(
      "environment.wind.directionAverage", windAvgDirectionMetadata);
  averageWindDirectionSensor->connect_to(averageWindDirectionReporter);

  /************************************************************************************
   * Wind Gust computation -- over 10 minutes *
   ************************************************************************************/

  // int maxGustCount = 0;
  // int gustPulseCount = 0;
  // int reports = 0;

  auto* gustDetector = new LambdaTransform<int, float>(
      //[&maxGustCount, &gustPulseCount, &reports](float count) {
      [](float count) {
        Serial.print("adding pulses: ");
        Serial.print(count);
        Serial.print(", reports=");
        Serial.println(reports);
        reports += 1;
        gustPulseCount += count;

        if (reports >= GUST_MAX_REPORTS) {
          if (gustPulseCount > maxGustCount) {
            maxGustCount = gustPulseCount;
          }
          gustPulseCount = 0;
          reports = 0;
        }
        // Convert from true wind direction to apparent wind
        // direction with the boat heading true north.
        return 0.0;
      },
      "" /* no config */);

  windPulseCounter->connect_to(gustDetector);

  auto* windGustSensor = new LambdaTransform<int, float>(
      [](float count) {
        float result = maxGustCount * WIND_PPS_TO_M_PER_SEC / 12.0;
        maxGustCount = 0;
        return result;
      },
      "" /* no config */);

  SKMetadata* windGustMetadata =
      new SKMetadata("m/s", "Maximum wind gusst over 10 minute interval",
                     "Wind gust", "Wind gust");

  auto* windGustReporter = new SKOutputFloat(
      "environment.wind.gust", "/wind/gust/SK_path", windGustMetadata);
  averageWindDirectionSensor->connect_to(windGustSensor)
      ->connect_to(windGustReporter);

  // debug info for gusts

  auto* gustCountSensor =
      new RepeatSensor<int>(3000, []() { return gustPulseCount; });
  gustCountSensor->connect_to(new SKOutputInt("environment.wind.gustPulseCount",
                                              "/wind/gustPulseCount/SK_path"));

  // Add an observer that prints out the current value of the analog input
  // every time it changes.
  // pressureSensor->attach(
  //     []() { debugD("Pressure: %f", pressureSensor->get()); });

  // Start networking, SK server connections and other SensESP
  //
  /*
    auto* cs = new StringConstantSensor(
        "https://github.com/smr547/Indoor-Enviro-Sensors/tree/main",1000
        "/sensors/source_code_path");

  */

  sensesp_app->start();
}

void loop() { app.tick(); }
