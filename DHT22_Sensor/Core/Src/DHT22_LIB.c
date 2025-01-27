/*
 * DHT22_LIB.c
 *
 *  Created on: Dec 5, 2024
 *      Author: Arshia
 */

#include "DHT22_LIB.h"

// External UART and Timer Handles (Declare in your main.c or HAL configuration file)
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;

/**
 * @brief Initializes the GPIO and Timer for the DHT22 sensor.
 */
void DHT22_Init(void) {
    // Initialize the Timer
    HAL_TIM_Base_Start(&htim1);
    DHT22_SendDebug("DHT22 Initialization Complete\n\r");
}

/**
 * @brief Sends the start signal to the DHT22 sensor.
 */
void DHT22_StartSignal(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure GPIO as Output
    GPIO_InitStruct.Pin = DHT22_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);

    // Send Start Signal
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);
    HAL_Delay(18); // Hold low for at least 18ms
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    DHT22_DelayUs(40); // Wait 40μs (recommended timing)

    // Reconfigure GPIO as Input (for receiving response)
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Checks for the response from the DHT22 sensor.
 * @return 1 if response is valid, 0 otherwise.
 */
uint8_t DHT22_CheckResponse(void) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    // Wait for LOW signal from sensor
    while (!HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin)) {
        if (__HAL_TIM_GET_COUNTER(&htim1) > 100) {
            return 0; // Timeout: No response
        }
    }

    __HAL_TIM_SET_COUNTER(&htim1, 0);

    // Wait for HIGH signal from sensor
    while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin)) {
        if (__HAL_TIM_GET_COUNTER(&htim1) > 100) {
            return 0; // Timeout: No response
        }
    }

    return 1; // Response is valid
}

/**
 * @brief Reads a single byte from the DHT22 sensor.
 * @return The byte read from the sensor.
 */
uint8_t DHT22_ReadByte(void) {
    uint8_t data = 0;

    for (uint8_t i = 0; i < 8; i++) {
        // Wait for LOW signal
        while (!HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin)) {}

        // Measure HIGH duration
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin)) {}

        // If HIGH duration > 40μs, it's a 1
        if (__HAL_TIM_GET_COUNTER(&htim1) > 40) {
            data |= (1 << (7 - i));
        }
    }

    return data;
}

/**
 * @brief Reads temperature and humidity from the DHT22 sensor.
 * @param[out] temperature Pointer to store the temperature value.
 * @param[out] humidity Pointer to store the humidity value.
 * @return 1 if successful, 0 otherwise.
 */
uint8_t DHT22_ReadData(float *temperature, float *humidity) {
    uint8_t rh_integral, rh_decimal;
    uint8_t temp_integral, temp_decimal;
    uint8_t checksum;

    // Start communication
    DHT22_StartSignal();
    if (!DHT22_CheckResponse()) {
        DHT22_SendDebug("DHT22: No response from sensor\n\r");
        return 0; // No response
    }

    // Read 5 bytes: RH integral, RH decimal, Temp integral, Temp decimal, Checksum
    rh_integral = DHT22_ReadByte();
    rh_decimal = DHT22_ReadByte();
    temp_integral = DHT22_ReadByte();
    temp_decimal = DHT22_ReadByte();
    checksum = DHT22_ReadByte();

    // Verify checksum
    uint8_t calculated_checksum = rh_integral + rh_decimal + temp_integral + temp_decimal;
    if (checksum != calculated_checksum) {
        DHT22_SendDebug("DHT22: Checksum mismatch\n\r");
        return 0; // Invalid data
    }

    // Calculate actual values
    *humidity = rh_integral + (rh_decimal / 10.0f);
    *temperature = temp_integral + (temp_decimal / 10.0f);
    if (temp_integral & 0x80) { // Negative temperature
        *temperature *= -1;
    }

    // Apply calibration for humidity (for example, adding an offset)
    float humidity_offset = -2.0f; // Example calibration offset (adjust as needed)
    *humidity += humidity_offset;

    return 1; // Success
}

/**
 * @brief Sends a string via UART for debugging.
 * @param[in] string The string to send.
 */
void DHT22_SendDebug(char *string) {
    HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), HAL_MAX_DELAY);
}

/**
 * @brief Microsecond delay implementation using TIM1.
 * @param[in] us Delay duration in microseconds.
 */
void DHT22_DelayUs(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us) {}
}
