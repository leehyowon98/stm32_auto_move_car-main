/*
 * dht.h
 *
 *  Created on: Jan 17, 2025
 *      Author: wlstn
 */

#ifndef INC_DHT_H_
#define INC_DHT_H_

#include "delay_us.h"
#include "stdio.h"
#include "main.h"
#include "tim.h"

enum
{
	INPUT,
	OUTPUT
};

typedef struct
{
	GPIO_TypeDef	*port;
	uint16_t		Pin;
	uint8_t			temperature;
	uint8_t			humidity;
}DHT11;


void dht11Init(DHT11 *dht, GPIO_TypeDef *port, uint16_t Pin);
void dht11GpioMode(DHT11 *dht, uint8_t mode);
uint8_t dht11Read(DHT11 *dht);



#endif /* INC_DHT_H_ */
