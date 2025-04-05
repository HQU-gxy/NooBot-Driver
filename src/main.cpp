
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <BMI088.h>
#include <STM32FreeRTOS.h>
#include <CMSIS_DSP.h>
#include <ulog.h>
#include <utility>

#include "SensorCollector.h"
#include "Magnetometer.h"
#include "IMU.h"
#include "Motor.hpp"
#include "UpLink.hpp"
#include "config.h"

/**
 * @brief Convert the linear and angular speed of the car to the speed of the left and right motors
 *
 * @param linear Linear speed of the car in m/s
 * @param angular Angular speed of the car in rad/s
 *
 * @return A pair of left and right motor speed in m/s
 */
std::pair<float, float> carSpeedToMotorSpeed(float linear, float angular)
{
  float leftSpeed = linear - angular * WHEEL_DISTANCE / 2;
  float rightSpeed = linear + angular * WHEEL_DISTANCE / 2;

  return {leftSpeed, rightSpeed};
}

/**
 * @brief Convert the speed of the left and right motors to the linear and angular speed of the car
 *
 * @param left Speed of the left motor in m/s
 * @param right Speed of the right motor in m/s
 *
 * @return A pair of linear and angular speed of the car in m/s and rad/s
 */
std::pair<float, float> motorSpeedToCarSpeed(float left, float right)
{
  float linear = (left + right) / 2;
  float angular = (right - left) / WHEEL_DISTANCE;

  return {linear, angular};
}

TFT_eSPI tft;
void app_main(void *)
{
  SensorCollector::begin();

  bool button1Pressed = false;
  attachInterrupt(BUTTON1_PIN, [&button1Pressed]()
                  { button1Pressed = true; }, FALLING);

  Motor leftMotor(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, MOTOR1_EN_PIN, MOTOR1_ENC_CFG, MOTOR1_INVERTED);
  Motor rightMotor(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, MOTOR2_EN_PIN, MOTOR2_ENC_CFG, MOTOR2_INVERTED);

#if defined(USE_LORA) && USE_LORA
  auto onUpLinkCommand = [&leftMotor, &rightMotor](float linear, float angular)
  {
    auto [leftSpeed, rightSpeed] = carSpeedToMotorSpeed(linear, angular);
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
  };

  auto getStatus = [&leftMotor, &rightMotor]() -> std::tuple<float, float, IMU::IMUData>
  {
    IMU::IMUData imuData;
    IMU::getData(&imuData);

    auto leftSpeed = leftMotor.getSpeed();
    auto rightSpeed = rightMotor.getSpeed();
    auto [linear, angular] = motorSpeedToCarSpeed(leftSpeed, rightSpeed);

    return {linear, angular, imuData};
  };

  enum : uint8_t
  {
    LORA,
    COMPUTER
  } activeUplink = LORA;

  UpLink loraLink(Serial1, UART1_TX_PIN, UART1_RX_PIN, 38400);
  loraLink.setGetStatusFunc(getStatus);
  loraLink.setOnCmdCallback([&activeUplink, &onUpLinkCommand](float linear, float angular)
                            { if (activeUplink == LORA)
                                onUpLinkCommand(linear, angular); });

  loraLink.setOnConnectCallback([&activeUplink]()
                                { activeUplink = LORA;
                                  ULOG_INFO("LoraLink connected"); });

  loraLink.setOnLinkLostCallback([&activeUplink, &leftMotor, &rightMotor]()
                                 { activeUplink = COMPUTER;
                                   leftMotor.setSpeed(0);
                                   rightMotor.setSpeed(0);
                                   ULOG_WARNING("LoraLink connection lost"); });

  UpLink computerLink(Serial3, UART3_TX_PIN, UART3_RX_PIN, 115200);
  computerLink.setGetStatusFunc(getStatus);
  computerLink.setOnCmdCallback([&activeUplink, &onUpLinkCommand](float linear, float angular)
                                { if (activeUplink == COMPUTER)
                                    onUpLinkCommand(linear, angular); });

  computerLink.setOnConnectCallback([]
                                    { ULOG_INFO("Computer connected"); });

  computerLink.setOnLinkLostCallback([&activeUplink, &leftMotor, &rightMotor]()
                                     {
                                      leftMotor.setSpeed(0);
                                      rightMotor.setSpeed(0);
                                      ULOG_WARNING("Computer connection lost"); });

#endif

  while (1)
  {

    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

#ifdef PID_TUNING
void tunePID(void *)
{
  Motor rightMotor(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, MOTOR2_EN_PIN, MOTOR2_ENC_CFG);
  ULOG_INFO("PID Tuning started");

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(50));
    struct __attribute__((packed)) PIDConfig
    {
      uint8_t header;
      float kp;
      float ki;
      float kd;
      char shit[4];
    };

    struct __attribute__((packed)) SpeedSet
    {
      uint8_t header;
      float speed;
      char shit[4];
    };

    static String inputStr;
    if (Serial3.available())
    {
      char c = Serial3.read();
      inputStr += c;
      if (inputStr.endsWith("shit"))
      {
        if (inputStr[0] == 0x96)
        {
          if (inputStr.length() != sizeof(PIDConfig))
            continue;
          auto parsed = reinterpret_cast<const PIDConfig *>(inputStr.c_str());
          ULOG_DEBUG("Received PID config");
          rightMotor.setPID(parsed->kp, parsed->ki, parsed->kd);
        }
        else if (inputStr[0] == 0x97)
        {
          if (inputStr.length() != sizeof(SpeedSet))
            continue;
          auto parsed = reinterpret_cast<const SpeedSet *>(inputStr.c_str());
          ULOG_DEBUG("Received speed set");
          rightMotor.setSpeed(parsed->speed);
        }
        inputStr = "";
      }
    }
  }
}
#endif // PID_TUNING

void setup(void)
{
  Serial2.setTx(UART2_TX_PIN);
  Serial2.setRx(UART2_RX_PIN);
  Serial2.begin(115200);

  ulog_init();
  ulog_subscribe([](ulog_level_t severity, char *msg)
                 { Serial2.printf("%d [%s]: %s\n", millis(), ulog_level_name(severity), msg); }, ULOG_DEBUG_LEVEL);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_PINK);

  ULOG_INFO("I'm fucked up!");
  ULOG_INFO("Checking the peripherals: ");

  tft.println("Checking the peripherals: ");

  if (!IMU::begin())
  {
    ULOG_ERROR("IMU Initialization Error");
    tft.printf("IMU Initialization Error");
    while (1)
      ;
  }
  tft.println("IMU Initialized");

  if (!Magneto::begin())
  {
    ULOG_ERROR("Compass Initialization Error");
    tft.printf("Compass Initialization Error");
    while (1)
      ;
  }
  tft.println("Compass Initialized");
  tft.fillScreen(TFT_CYAN);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);

  analogWriteFrequency(1e4);
  auto ledTimer = xTimerCreate("Alive LED", pdMS_TO_TICKS(1000), pdTRUE, nullptr, [](TimerHandle_t)
                               { digitalToggle(LED1_PIN); });
  xTimerStart(ledTimer, 0);

#ifdef PID_TUNING
  xTaskCreate(tunePID, "Tune PID", 1024, nullptr, osPriorityHigh, nullptr);

#else
  xTaskCreate(app_main, "Main App", 1024, nullptr, osPriorityHigh, nullptr);
#endif // PID_TUNING

  vTaskStartScheduler();
}

void loop() { /* Never fuck into this */ }