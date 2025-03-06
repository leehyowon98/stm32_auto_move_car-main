#include "dht.h"

void DHT11_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOC clock
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Configure GPIO pin
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    HAL_Delay(1000);  // Wait for DHT11 to get stable
}

static void DHT11_SetPinOutput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

static void DHT11_SetPinInput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

static uint8_t DHT11_ReadByte(void) {
    uint8_t i, data = 0;

    for(i = 0; i < 8; i++) {
        while(!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));  // Wait for pin to go high
        delay_us(40);  // Wait for 40 microseconds

        if(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) {  // If pin is still high
            data |= (1 << (7-i));  // Then its a 1
            while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));  // Wait for pin to go low
        }
    }

    return data;
}

uint8_t DHT11_Read(uint8_t *temperature, uint8_t *humidity) {
    uint8_t data[5] = {0};

    // Start signal
    DHT11_SetPinOutput();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(18);  // At least 18ms delay
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(40);  // Wait for 40 microseconds

    DHT11_SetPinInput();

    if(!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) {
        while(!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));  // Wait for pin to go high
        while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));   // Wait for pin to go low

        // Read 5 bytes
        for(int i = 0; i < 5; i++) {
            data[i] = DHT11_ReadByte();
        }

        // Verify checksum
        if(data[4] == (data[0] + data[1] + data[2] + data[3])) {
            *humidity = data[0];
            *temperature = data[2];
            return 1;  // Success
        }
    }

    return 0;  // Error
}
