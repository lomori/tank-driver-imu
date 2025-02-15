#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA219.h>

#include <SerialProtocol.h>

#include <Controller.h>

#include <IMU.h>

#define LED_PIN 2

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

static const int NUM_LINES = 8;
static const int LINE_SIZE_RAW = 32;
static const int LINE_SIZE = LINE_SIZE_RAW - 1;
char displayLines[NUM_LINES][LINE_SIZE_RAW];
int currentDisplayLine = 0;

static const char *tag = "Main";

Adafruit_SSD1306 *display;
Adafruit_INA219 ina219;
SerialProtocol serialProtocol(&Serial);

IMU imu;
Controller controller;

SemaphoreHandle_t i2cSemaphore;

bool first = true;
unsigned long lastDisplayUpdate;
unsigned long lastCheck;
unsigned long lastIMUPublished = 0;

int counterUndervoltage = 0;

void batteryCheck()
{
  float busVoltage = 0;

  for (int i = 0; i < 3; i++)
  {
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdPASS)
    {
      busVoltage = ina219.getBusVoltage_V();
      vTaskDelay(10);
      xSemaphoreGive(i2cSemaphore);
    }
    else
      esp_log_write(ESP_LOG_ERROR, tag, "Semaphore failed");

    if (busVoltage > 0.0)
      break;
    else
      vTaskDelay(0.01);
  }

  if (busVoltage < 12.0)
  {
    esp_log_write(ESP_LOG_ERROR, tag, "UNDERVOLTAGE !!!!!");
    if (++counterUndervoltage > 3)
    {
      digitalWrite(LED_PIN, LOW);
      if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdPASS)
      {
        float shuntVoltage = ina219.getShuntVoltage_mV();
        float current_mA = ina219.getCurrent_mA();
        float power_mW = ina219.getPower_mW();
        float loadVoltage = busVoltage + (shuntVoltage / 1000);
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println("UNDERVOLTAGE !!!!!");
        display->printf("Bus: %4.2fV", busVoltage);
        display->printf("Shunt: %4.2fV", shuntVoltage);
        display->printf("Load: %4.2fV", loadVoltage);
        display->printf("Current: %6.1fmA", current_mA);
        display->printf("Power: %6.1fmW", power_mW);
        display->display();
        xSemaphoreGive(i2cSemaphore);
      }
      else
        esp_log_write(ESP_LOG_ERROR, tag, "Semaphore failed");

      esp_deep_sleep_start();
    }
  }
  else
    counterUndervoltage = 0;
}

void displayI2CDevices(TwoWire &wire)
{
  byte error, address;
  int nDevices;
  esp_log_write(ESP_LOG_INFO, tag, "i2c devices");
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    if (address == SCREEN_ADDRESS)
      continue;

    error = 0;
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdPASS)
    {
      wire.beginTransmission(address);
      error = wire.endTransmission();
      xSemaphoreGive(i2cSemaphore);
    }

    if (error == 0)
    {
      esp_log_write(ESP_LOG_INFO, tag, "I2C 0x%02x", address);
      nDevices++;
    }
    else if (error == 4)
    {
      esp_log_write(ESP_LOG_ERROR, tag, "Error at 0x%02x", address);
    }
  }

  if (nDevices == 0)
    esp_log_write(ESP_LOG_ERROR, tag, "No I2C devices");
  else
    esp_log_write(ESP_LOG_INFO, tag, "Found %d devices", nDevices);
  vTaskDelay(5000);
}

void processCommand()
{
  std::string *msg = serialProtocol.receiveData();
  if (msg != nullptr)
  {
    esp_log_write(ESP_LOG_INFO, tag, "CMD:%d %d (%s)",
                  xPortInIsrContext(), xPortGetCoreID(), msg->c_str());
    if (msg->find("clear") != std::string::npos)
    {
      serialProtocol.clear();
      esp_log_write(ESP_LOG_INFO, tag, "TX queue cleared");
    }
    else if (msg->find("stop") != std::string::npos)
    {
      serialProtocol.clear();
      controller.stop();
      esp_log_write(ESP_LOG_INFO, tag, "RVR Stopped");
    }
    else if (msg->find("rotate") != std::string::npos)
    {
      int first = msg->find(',');
      unsigned long timeoutMS = atol(msg->substr(first + 1).c_str());
      controller.setDynamics(0.0, 0.3, timeoutMS);
    }
    else
    {
      int first = msg->find(',');
      int second = msg->find(',', first + 1);

      float linearX = atof(msg->substr(0, first).c_str());
      float angularZ = atof(msg->substr(first + 1, second - first - 1).c_str());
      unsigned long timeoutMS = atol(msg->substr(second + 1).c_str());

      esp_log_write(ESP_LOG_INFO, tag, "%7.2f %7.2f %2.1f",
                    linearX, angularZ, timeoutMS / 1000.0);

      controller.setDynamics(linearX, angularZ, timeoutMS);
    }
    delete msg;
  }
}

int localvprintf(const char *fmt, va_list ap)
{
  static const int PREFIX_SIZE = 5;
  char msg[128] = "LOG: ";

  vsnprintf(msg + PREFIX_SIZE, sizeof(msg) - PREFIX_SIZE, fmt, ap);
  serialProtocol.sendData(msg);

  if (display && xPortInIsrContext() == 0)
  {
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdPASS)
    {
      snprintf(displayLines[currentDisplayLine], LINE_SIZE, "%s", msg + PREFIX_SIZE);
      if (++currentDisplayLine >= NUM_LINES)
        currentDisplayLine = 0;

      display->clearDisplay();
      display->setCursor(0, 0);

      for (int l = 0; l < NUM_LINES; l++)
        display->println(displayLines[(currentDisplayLine + l) % NUM_LINES]);

      display->display();
      xSemaphoreGive(i2cSemaphore);
    }
    else
      // Oh, what should we do????
      Serial.printf("Failed to acquire semaphore");
  }
  return 0;
}

void publishIMU()
{
  char msg[128];
  IMU::IMU_VALUES readings;

  if (imu.isEnabled())
  {
    imu.getPublishData(readings);

    readings.velocities.data[0] = (float)controller.getActualLinearVelocity();
    readings.velocities.data[1] = 0.0f;
    readings.velocities.data[2] = 0.0f;

    snprintf(msg, sizeof(msg), "IMU,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f",
             readings.gyro.data[0], readings.gyro.data[1], readings.gyro.data[2],
             readings.angles.data[0], readings.angles.data[1], readings.angles.data[2],
             readings.accelerations.data[0], readings.accelerations.data[1], readings.accelerations.data[2],
             readings.velocities.data[0], readings.velocities.data[1], readings.velocities.data[2]);
    serialProtocol.sendData(msg);
  }
}

void setup()
{
  // Immediately stop engines if they are running
  pinMode(Constants::ENABLE_ENGINES, OUTPUT);
  digitalWrite(Constants::ENABLE_ENGINES, LOW);

  memset(displayLines, 0, sizeof(displayLines));
  i2cSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(i2cSemaphore);
  esp_log_set_vprintf(localvprintf);
  esp_log_level_set("*", ESP_LOG_DEBUG);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  vTaskDelay(10000);

  Wire.begin();

  /*
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire,
                                 OLED_RESET, 100000UL, 100000UL);
  // Initialising the UI will init the display too.
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, false, false))
  {
    Serial.printf("SSD1306 initialization failed.");
    for (;;)
      ; // Don't proceed, loop forever
  }
  else
    esp_log_write(ESP_LOG_INFO, tag, "SSD1306 initialized");

  display->display();
  vTaskDelay(2000);

  digitalWrite(LED_PIN, LOW);

  esp_log_write(ESP_LOG_INFO, tag, "First I2C bus.");
  displayI2CDevices(Wire);

  display->clearDisplay();
  display->setTextSize(1);              // Normal 1:1 pixel scale
  display->setTextColor(SSD1306_WHITE); // Draw white text
  display->setTextWrap(false);
  vTaskDelay(100);
*/
  // Initialize the INA219.
  while (!ina219.begin())
  {
    esp_log_write(ESP_LOG_ERROR, tag, "INA219 failed");
  }
  ina219.setCalibration_32V_2A(); // Max current and voltage

  batteryCheck();

  float busVoltage = 0.0, shuntVoltage = 0.0, current_mA = 0.0, power_mW = 0.0;
  if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdPASS)
  {
    busVoltage = ina219.getBusVoltage_V();
    shuntVoltage = ina219.getShuntVoltage_mV();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    xSemaphoreGive(i2cSemaphore);
  }
  else
    esp_log_write(ESP_LOG_ERROR, tag, "Semaphore failed");

  float loadVoltage = busVoltage + (shuntVoltage / 1000);
  esp_log_write(ESP_LOG_INFO, tag, "Bus: %4.2fV", busVoltage);
  esp_log_write(ESP_LOG_INFO, tag, "Shunt: %4.2fV", shuntVoltage);
  esp_log_write(ESP_LOG_INFO, tag, "Load: %4.2fV", loadVoltage);
  esp_log_write(ESP_LOG_INFO, tag, "Current: %6.1fmA", current_mA);
  esp_log_write(ESP_LOG_INFO, tag, "Power: %6.1fmW", power_mW);
  vTaskDelay(5000);

  imu.init(i2cSemaphore);

  controller.init(&imu);

  lastDisplayUpdate = millis();
  lastCheck = millis();

  serialProtocol.start();

  esp_log_write(ESP_LOG_INFO, tag, "Starting loop");
}

void loop()
{
  unsigned long now = millis();

  try
  {
    if (first)
    {
      esp_log_write(ESP_LOG_INFO, tag, "Loop: ISR %d Core %d",
                    xPortInIsrContext(), xPortGetCoreID());
      first = false;
    }

    controller.controlLoop();

    if ((now - lastIMUPublished) > 250)
    {
      publishIMU();
      lastIMUPublished = now;
    }

    if ((now - lastCheck) >= 80)
    {
      //batteryCheck();
      processCommand();
      lastCheck = now;
    }

    if ((now - lastDisplayUpdate) > 1000)
    {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      lastDisplayUpdate = now;
    }
  }
  catch (const std::exception &e)
  {
    esp_log_write(ESP_LOG_ERROR, tag, "Main:%s", e.what());
  }

  vTaskDelay(10);
}
