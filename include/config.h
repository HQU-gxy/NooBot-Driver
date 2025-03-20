#pragma once

#include <Arduino.h>

// LED pins
constexpr uint8_t LED1_PIN = PC2;
constexpr uint8_t LED2_PIN = PC3;

// Button pins
constexpr uint8_t BUTTON1_PIN = PA12;
constexpr uint8_t BUTTON2_PIN = PA11;
constexpr uint8_t BUTTON3_PIN = PA10;

// IMU pins
constexpr uint8_t ACCEL_CS_PIN = PB1;
constexpr uint8_t GYRO_CS_PIN = PB0;
constexpr uint8_t SPI1_SCK_PIN = PA5;
constexpr uint8_t SPI1_MISO_PIN = PA6;
constexpr uint8_t SPI1_MOSI_PIN = PA7;

// Compass pins
constexpr uint8_t I2C1_SDA_PIN = PB9;
constexpr uint8_t I2C1_SCL_PIN = PA15;

// Compass magnetic declination
constexpr float MAGNETIC_DECLINATION = -(4.0 + (38.0 / 60.0)) / (180 / PI);

// UART pins
// Reserve for future peripherals (like LoraLink)
constexpr uint8_t UART1_TX_PIN = PB6;
constexpr uint8_t UART1_RX_PIN = PB7;
// For debugging
constexpr uint8_t UART2_TX_PIN = PA2;
constexpr uint8_t UART2_RX_PIN = PA3;
// For uplink and tuning
constexpr uint8_t UART3_TX_PIN = PB10;
constexpr uint8_t UART3_RX_PIN = PB11;

// Power monitor pins
constexpr uint8_t ISENS_PIN = PA0;
constexpr uint8_t VSENS_PIN = PA1;

struct EncoderCfg
{
    TIM_TypeDef *timer;
    uint16_t encA_Pin; // Pay attention to the AF number
    uint16_t encB_Pin; // Pay attention to the AF number
};

// Motor configs
constexpr uint8_t MOTOR1_EN_PIN = PB4;
constexpr uint8_t MOTOR1_IN1_PIN = PC11;
constexpr uint8_t MOTOR1_IN2_PIN = PC12;
constexpr EncoderCfg MOTOR1_ENC_CFG = {
    .timer = TIM8,
    .encA_Pin = PC7_ALT1,
    .encB_Pin = PC6_ALT1,
};
constexpr bool MOTOR1_INVERTED = true; // Installed in the opposite direction

constexpr uint8_t MOTOR2_EN_PIN = PB5;
constexpr uint8_t MOTOR2_IN1_PIN = PD2;
constexpr uint8_t MOTOR2_IN2_PIN = PB3;
constexpr EncoderCfg MOTOR2_ENC_CFG = {
    .timer = TIM1,
    .encA_Pin = PC1,
    .encB_Pin = PC0,
};
constexpr bool MOTOR2_INVERTED = false;

constexpr uint16_t MCPWM_BEEP_DC = 20;
constexpr float WHEEL_DISTANCE = 0.35; // The distance between the two wheels in meters

// Optional modules
#define USE_LORA 1