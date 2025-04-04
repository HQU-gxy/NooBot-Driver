#include <ulog.h>
#include <STM32FreeRTOS.h>
#include <stm32g431xx.h>

#include "Motor.h"

/**
 * @brief Construct a new Motor object
 * 
 * @param in1_pin In1 pin of the direction controlling
 * @param in2_pin In2 pin of the direction controlling
 * @param enable_pin Enable pin of the driving module
 * @param encoder The encoder configuration of the hardware timer
 * @param inverted Whether to invert the motor
 */
Motor::Motor(uint8_t in1_pin,
             uint8_t in2_pin,
             uint8_t enable_pin,
             EncoderCfg encoder,
             bool inverted)
    : in1Pin(in1_pin),
      in2Pin(in2_pin),
      enablePin(enable_pin),
      inverse(inverted)
{
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinmap_pinout(digitalPinToPinName(encoder.encA_Pin), PinMap_TIM);
  pinmap_pinout(digitalPinToPinName(encoder.encB_Pin), PinMap_TIM);

  encoderTimer = std::make_shared<HardwareTimer>(encoder.timer);
  encoderTimer->setPreloadEnable(false);

  LL_TIM_ENCODER_InitTypeDef encInitStruct{
      .EncoderMode = LL_TIM_ENCODERMODE_X4_TI12,
      .IC1Polarity = LL_TIM_IC_POLARITY_RISING,
      .IC1ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI,
      .IC1Prescaler = LL_TIM_ICPSC_DIV4,
      .IC1Filter = LL_TIM_IC_FILTER_FDIV1,
      .IC2Polarity = LL_TIM_IC_POLARITY_RISING,
      .IC2ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI,
      .IC2Prescaler = LL_TIM_ICPSC_DIV4,
      .IC2Filter = LL_TIM_IC_FILTER_FDIV1,
  };
  LL_TIM_ENCODER_Init(encoder.timer, &encInitStruct);
  LL_TIM_DisableMasterSlaveMode(encoder.timer);
  LL_TIM_SetTriggerOutput(encoder.timer, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(encoder.timer, LL_TIM_TRGO2_RESET);
  LL_TIM_SetCounter(encoder.timer, 0);
  LL_TIM_EnableCounter(encoder.timer);

  auto pidTimer = xTimerCreate("PID", pdMS_TO_TICKS(PID_PERIOD), pdTRUE, static_cast<void *>(this), fuckPID);
  xTimerStart(pidTimer, 0);
  ULOG_DEBUG("PID Timer started");
}

/**
 * @brief Set the target speed of the motor
 * 
 * @param speed the target speed in m/s
 */
void Motor::setSpeed(float speed)
{
  targetFreq = speed * SPEED_SCALE;
  if (abs(targetFreq) < MIN_TARGET_FREQ / 2)
    targetFreq = 0;
  else if (abs(targetFreq) < MIN_TARGET_FREQ) // Wether to set the motor to stop
    targetFreq = targetFreq > 0 ? MIN_TARGET_FREQ : -MIN_TARGET_FREQ;
}

Motor::~Motor()
{
}

/**
 * @brief Read motor angle from the timer and perform PID
 *
 * @param handle Timer handle with the Motor instance as ID
 * @note This function is called by the FreeRTOS timer
 */
void Motor::fuckPID(TimerHandle_t handle)
{
  auto motor = static_cast<Motor *>(pvTimerGetTimerID(handle));
  if (!motor)
  {
    ULOG_ERROR("Fuck PID is called without a Motor pointer passed in");
    return;
  }

  auto currentCounter = static_cast<int16_t>(LL_TIM_GetCounter(motor->encoderTimer->getHandle()->Instance) & 0xffff);
  motor->freqMeasured = motor->inverse ? (-currentCounter * (1000 / motor->PID_PERIOD))
                                       : (currentCounter * (1000 / motor->PID_PERIOD));

  LL_TIM_SetCounter(motor->encoderTimer->getHandle()->Instance, 0);

  // An incredibly shitty PID implementation

  int32_t currentError = motor->targetFreq - motor->freqMeasured;
  int32_t output = motor->lastOutput + motor->PID_KP * (currentError - motor->lastError) +
                   motor->PID_KI * currentError +
                   motor->PID_KD * (currentError - 2 * motor->lastError + motor->lastLastError);

  if (output < 0)
    motor->setDirection(1);
  else
    motor->setDirection(0);

  motor->setDuty(output < 0 ? -output : output);

  motor->lastOutput = output;
  motor->lastError = currentError;
  motor->lastLastError = motor->lastError;

#ifdef PID_TUNING
  struct __attribute__((packed))
  {
    uint8_t header = 0x69;
    int32_t freq;
    int32_t error;
    int32_t output;
    char shit[4] = {'s', 'h', 'i', 't'};
  } pidStatus{
      .freq = motor->freqMeasured,
      .error = currentError,
      .output = output,
  };
  Serial3.write(reinterpret_cast<char *>(&pidStatus), sizeof(pidStatus));
#endif
}