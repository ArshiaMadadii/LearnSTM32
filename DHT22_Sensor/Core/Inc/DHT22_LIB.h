/*
 * DHT22_LIB.h
 *
 *  Created on: Dec 5, 2024
 *      Author: Arshia
 */
#ifndef DHT22_H
#define DHT22_H

#include "stm32f1xx_hal.h" // Include HAL library for STM32
#include <stdint.h>
#include <string.h>

// Pin and Port Definitions (Modify as Needed)
#define DHT22_Pin GPIO_PIN_9
#define DHT22_GPIO_Port GPIOB

// Function Declarations
/**
 * @brief Initializes the DHT22 sensor (sets up GPIO and Timer).
 */
void DHT22_Init(void);

/**
 * @brief Starts the communication signal with the DHT22 sensor.
 */
void DHT22_StartSignal(void);

/**
 * @brief Checks for the response signal from the DHT22 sensor.
 * @return 1 if the response is valid, 0 otherwise.
 */
uint8_t DHT22_CheckResponse(void);

/**
 * @brief Reads a single byte of data from the DHT22 sensor.
 * @return The byte read from the sensor.
 */
uint8_t DHT22_ReadByte(void);

/**
 * @brief Reads temperature and humidity data from the DHT22 sensor.
 * @param[out] temperature Pointer to store the temperature value.
 * @param[out] humidity Pointer to store the humidity value.
 * @return 1 if the read was successful, 0 otherwise.
 */
uint8_t DHT22_ReadData(float *temperature, float *humidity);

/**
 * @brief Sends a string via UART for debugging.
 * @param[in] string The string to send.
 */
void DHT22_SendDebug(char *string);

/**
 * @brief Microsecond delay implementation using TIM1.
 * @param[in] us Delay duration in microseconds.
 */
void DHT22_DelayUs(uint16_t us);

#endif // DHT22_H
