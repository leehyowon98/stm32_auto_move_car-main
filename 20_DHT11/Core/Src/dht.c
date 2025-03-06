/*
 * dht.c
 *
 *  Created on: Jan 17, 2025
 *      Author: wlstn
 */

#include "main.h"
#include "dht.h"
#include "tim.h"
#include <stdint.h>

void dht11Init(DHT11 *dht, GPIO_TypeDef *port, uint16_t Pin)
{
	dht->port = port; //포트 설정
	dht->Pin = Pin; //핀설정
}

//DHT11 GPIO Mode 함수 설정
void dht11GpioMode(DHT11 *dht, uint8_t mode) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 }; //GPio 구조체 변수 선언 및 초기화

	if (mode == OUTPUT)
	{
		//출력 모드(복사해온거임 gpio.c에서)
		GPIO_InitStruct.Pin = dht->Pin; //핀으로 넣어준다.
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(dht->port, &GPIO_InitStruct);

	} else if (mode == INPUT) {
		//입력 모드 설정
		GPIO_InitStruct.Pin = dht->Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(dht->port, &GPIO_InitStruct);
	}
}
uint8_t dht11Read(DHT11 *dht) {
	bool ret = true; //기본 반환 설정

	uint16_t timeTick = 0;	//시간 측정 변수 선언 및 초기화
	uint8_t pulse[40] = {0}; //40비트 데이터 저장
	//온/습도 데이터 변수 선언 및 초기화
	uint8_t humValue1 = 0, humValue2 = 0;
	uint8_t tempValue1 = 0, tempValue2 = 0;
	uint8_t parityValue = 0;

	//타어머 시작
	HAL_TIM_Base_Start(&htim11);

	//통신 시작 신호 전송
	dht11GpioMode(dht, OUTPUT);	//gpio를 출력으로 선언
	HAL_GPIO_WritePin(dht->port, dht->Pin, 0);  //로우
	HAL_Delay(20);		//적어도 18ms 이상 대기
	HAL_GPIO_WritePin(dht->port, dht->Pin, 1);	//하이
	delay_us(20);
	dht11GpioMode(dht, INPUT);		//input 모드로 전환

	//DHT11의 응답 신호 대기
	__HAL_TIM_SET_COUNTER(&htim11, 0);
	while (HAL_GPIO_ReadPin(dht->port, dht->Pin) == GPIO_PIN_RESET) {
		if (__HAL_TIM_GET_COUNTER(&htim11) > 100) //타임아웃 체크
				{
			printf("Low Signal Time out\n\r");
			break;
		}
	}

	__HAL_TIM_SET_COUNTER(&htim11, 0);
	while (HAL_GPIO_ReadPin(dht->port, dht->Pin) == GPIO_PIN_SET) {
		if (__HAL_TIM_GET_COUNTER(&htim11) > 100) //타임아웃 체크
				{
			printf("High Signal Time out\n\r");
			break;
		}
	}
	for (uint8_t i = 0; i < 40; i++) {
	    // LOW 상태 대기
	    while (HAL_GPIO_ReadPin(dht->port, dht->Pin) == GPIO_PIN_RESET);

	    // 타이머 카운터 초기화
	    __HAL_TIM_SET_COUNTER(&htim11, 0);

	    // HIGH 상태 대기
	    while (HAL_GPIO_ReadPin(dht->port, dht->Pin) == GPIO_PIN_SET) {
	        // 타이머 값 저장
	        timeTick = __HAL_TIM_GET_COUNTER(&htim11);

	        // 신호 길이 판별
	        if (timeTick > 20 && timeTick < 30) { // 26 ~ 28 us -> 0
	            pulse[i] = 0;
	        } else if (timeTick > 65 && timeTick < 85) { // 70 ~ 75 us -> 1
	            pulse[i] = 1;
	        }
	    }
	}

	HAL_TIM_Base_Stop(&htim11);

	//온습도 데이터 처리
	for (uint8_t i = 0; i < 8; i++) {humValue1 = (humValue1 << 1) + pulse[i];} // 습도 상위 8비트
	for (uint8_t i = 8; i < 16; i++) {humValue2 = (humValue2 << 1) + pulse[i];} // 습도 상위 8비트
	for (uint8_t i = 16; i < 24; i++) {tempValue1 = (tempValue1 << 1) + pulse[i];} // 온도 상위 8비트}
	for (uint8_t i = 24; i < 32; i++) {tempValue2 = (tempValue2 << 1) + pulse[i];} // 온도 하위 8비트}
	for (uint8_t i = 32; i < 40; i++) {parityValue = (parityValue << 1) + pulse[i];} // 체크섬 계산}

	//구조체 온습도 값 저장
	dht->temperature = tempValue1;
	dht->humidity = humValue1;

	//데이터 무결설 검증

	uint8_t checkSum = humValue1 + humValue2 + tempValue1 + tempValue2;
	if(checkSum != parityValue)
	{
		printf("checksum Error\n\r");
		ret = false;
	}
	return ret;
		}
