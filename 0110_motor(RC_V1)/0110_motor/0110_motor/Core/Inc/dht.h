#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "main.h"

#define DHT11_PORT GPIOC
#define DHT11_PIN GPIO_PIN_4

void DHT11_Init(void);
uint8_t DHT11_Read(uint8_t *temperature, uint8_t *humidity);

#endif /* INC_DHT11_H_ */
