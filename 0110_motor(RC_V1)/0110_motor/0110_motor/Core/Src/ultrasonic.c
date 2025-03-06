#include "ultrasonic.h"

/* Private variables ---------------------------------------------------------*/
static uint16_t IC_ValueRising1 = 0;
static uint16_t IC_ValueFalling1 = 0;
static uint16_t captureTime1 = 0;
static uint8_t captureFlag1 = 0;
static uint16_t distance1 = 0;

static uint16_t IC_ValueRising2 = 0;
static uint16_t IC_ValueFalling2 = 0;
static uint16_t captureTime2 = 0;
static uint8_t captureFlag2 = 0;
static uint16_t distance2 = 0;

static uint16_t IC_ValueRising3 = 0;
static uint16_t IC_ValueFalling3 = 0;
static uint16_t captureTime3 = 0;
static uint8_t captureFlag3 = 0;
static uint16_t distance3 = 0;

static uint16_t previousDistance1 = 0;
static uint16_t previousDistance2 = 0;
static uint16_t previousDistance3 = 0;

#define MAX_ALLOWED_DIFF 20  // 허용 가능한 최대 변화량 (cm)
#define FILTER_SIZE 5           // 이동 평균 필터 크기

typedef struct {
    uint16_t buffer[FILTER_SIZE];
    uint16_t sum;
    uint8_t index;
    uint8_t count;
    uint16_t lastStableValue;
    uint8_t stableCounter;
} EnhancedFilter;

static EnhancedFilter filter1 = {0};  // FL 센서
static EnhancedFilter filter2 = {0};  // FR 센서
static EnhancedFilter filter3 = {0};  // B 센서

#define FL_FILTER_SIZE 7
#define FL_MAX_DIFF 15  // 허용 가능한 최대 변화량 (cm)

typedef struct {
    uint16_t buffer[FL_FILTER_SIZE];
    uint16_t sum;
    uint8_t index;
    uint8_t count;
    uint16_t lastStableValue;
    uint8_t consecutiveStableCount;
} FLSpecialFilter;

static FLSpecialFilter flFilter = {0};

static uint16_t prev_distance1 = 0;
static uint16_t stable_distance1 = 0;
static uint8_t unstable_count1 = 0;

#define MAX_UNSTABLE_THRESHOLD 3
#define MAX_DISTANCE_VARIATION 20  // 허용 가능한 최대 거리 변화 (cm)

uint16_t updateEnhancedFilter(EnhancedFilter* filter, uint16_t newValue) {
    // 0 값이나 비정상적인 값 제외
    if (newValue == 0 || newValue > 400) {
        return filter->lastStableValue;
    }

    // 마지막 안정값과의 차이 계산
    uint16_t diff = (newValue > filter->lastStableValue) 
        ? (newValue - filter->lastStableValue) 
        : (filter->lastStableValue - newValue);

    // 급격한 변화 감지
    if (diff > MAX_ALLOWED_DIFF) {
        // 안정값 유지
        filter->stableCounter++;
        
        // 연속 3회 이상 급격한 변화면 새 값 수용
        if (filter->stableCounter >= 3) {
            filter->lastStableValue = newValue;
            filter->stableCounter = 0;
        }
        
        return filter->lastStableValue;
    }

    // 안정된 값으로 인정
    filter->stableCounter = 0;
    filter->lastStableValue = newValue;

    // 이동 평균 필터 로직
    if (filter->count < FILTER_SIZE) {
        filter->buffer[filter->index] = newValue;
        filter->sum += newValue;
        filter->count++;
        filter->index = (filter->index + 1) % FILTER_SIZE;
        return filter->sum / filter->count;
    }

    // 오래된 값 제거 및 새 값 추가
    filter->sum = filter->sum - filter->buffer[filter->index] + newValue;
    filter->buffer[filter->index] = newValue;
    filter->index = (filter->index + 1) % FILTER_SIZE;

    return filter->sum / FILTER_SIZE;
}

uint16_t updateFLSpecialFilter(uint16_t newValue) {
    // 0 값이나 비정상적인 값 제외
    if (newValue == 0 || newValue > 300) {
        return flFilter.lastStableValue ? flFilter.lastStableValue : newValue;
    }

    // 첫 측정이면 그대로 반환
    if (flFilter.count == 0) {
        flFilter.lastStableValue = newValue;
        flFilter.buffer[0] = newValue;
        flFilter.sum = newValue;
        flFilter.count = 1;
        return newValue;
    }

    // 마지막 안정값과의 차이 계산
    uint16_t diff = (newValue > flFilter.lastStableValue) 
        ? (newValue - flFilter.lastStableValue) 
        : (flFilter.lastStableValue - newValue);

    // 급격한 변화 감지
    if (diff > FL_MAX_DIFF) {
        // 연속 3회 이상 안정값 유지
        flFilter.consecutiveStableCount++;
        
        // 3회 이상 연속되면 새 값 부분 수용
        if (flFilter.consecutiveStableCount >= 3) {
            flFilter.lastStableValue = (flFilter.lastStableValue + newValue) / 2;
            flFilter.consecutiveStableCount = 0;
        }
        
        return flFilter.lastStableValue;
    }

    // 안정된 값으로 인정
    flFilter.consecutiveStableCount = 0;
    flFilter.lastStableValue = newValue;

    // 순환 버퍼 및 이동 평균 로직
    if (flFilter.count < FL_FILTER_SIZE) {
        flFilter.buffer[flFilter.index] = newValue;
        flFilter.sum += newValue;
        flFilter.count++;
        flFilter.index = (flFilter.index + 1) % FL_FILTER_SIZE;
        return flFilter.sum / flFilter.count;
    }

    // 오래된 값 제거 및 새 값 추가
    flFilter.sum = flFilter.sum - flFilter.buffer[flFilter.index] + newValue;
    flFilter.buffer[flFilter.index] = newValue;
    flFilter.index = (flFilter.index + 1) % FL_FILTER_SIZE;

    return flFilter.sum / FL_FILTER_SIZE;
}

uint16_t Get_Distance1(void) {
    // 0 또는 비정상적인 값 제외
    if (distance1 == 0 || distance1 > 300) {
        return stable_distance1;
    }

    // 첫 측정이거나 이전 안정값이 없는 경우
    if (stable_distance1 == 0) {
        stable_distance1 = distance1;
        return stable_distance1;
    }

    // 거리 변화 계산
    int16_t distance_diff = abs((int16_t)distance1 - (int16_t)stable_distance1);

    // 거리 변화가 허용 범위 내인 경우
    if (distance_diff <= MAX_DISTANCE_VARIATION) {
        stable_distance1 = distance1;
        unstable_count1 = 0;
        return stable_distance1;
    }

    // 거리 변화가 큰 경우
    unstable_count1++;

    // 연속 3회 이상 불안정하면 값 업데이트
    if (unstable_count1 >= MAX_UNSTABLE_THRESHOLD) {
        stable_distance1 = distance1;
        unstable_count1 = 0;
    }

    return stable_distance1;
}

uint16_t Get_Distance2(void) {
    return updateEnhancedFilter(&filter2, distance2);
}

uint16_t Get_Distance3(void) {
    return updateEnhancedFilter(&filter3, distance3);
}

/* Functions ---------------------------------------------------------*/
void HCSR04_Init(void) {
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);   // 첫 번째 초음파 센서
    HAL_TIM_IC_Start_IT(&htim10, TIM_CHANNEL_1);  // 두 번째 초음파 센서
    HAL_TIM_IC_Start_IT(&htim11, TIM_CHANNEL_1);  // 세 번째 초음파 센서
}

void HCSR04_Read(void) {
    HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(TRIG_PORT2, TRIG_PIN2, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(TRIG_PORT2, TRIG_PIN2, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT2, TRIG_PIN2, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(TRIG_PORT3, TRIG_PIN3, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(TRIG_PORT3, TRIG_PIN3, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT3, TRIG_PIN3, GPIO_PIN_RESET);

    __HAL_TIM_ENABLE_IT(&htim5, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_IT(&htim10, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_IT(&htim11, TIM_CHANNEL_1);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (htim == &htim5) {  // 첫 번째 센서
            if (captureFlag1 == 0) {
                IC_ValueRising1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                captureFlag1 = 1;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            } else if (captureFlag1 == 1) {
                IC_ValueFalling1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                captureFlag1 = 0;
                if (IC_ValueFalling1 > IC_ValueRising1) {
                    captureTime1 = IC_ValueFalling1 - IC_ValueRising1;
                } else {
                    captureTime1 = (0xFFFF - IC_ValueRising1) + IC_ValueFalling1;
                }
                distance1 = captureTime1 / 58;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }
        else if (htim == &htim10) {  // 두 번째 센서
            if (captureFlag2 == 0) {
                IC_ValueRising2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                captureFlag2 = 1;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            } else if (captureFlag2 == 1) {
                IC_ValueFalling2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                captureFlag2 = 0;
                if (IC_ValueFalling2 > IC_ValueRising2) {
                    captureTime2 = IC_ValueFalling2 - IC_ValueRising2;
                } else {
                    captureTime2 = (0xFFFF - IC_ValueRising2) + IC_ValueFalling2;
                }
                distance2 = captureTime2 / 58;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }
        else if (htim == &htim11) {  // 세 번째 센서
            if (captureFlag3 == 0) {
                IC_ValueRising3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                captureFlag3 = 1;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            } else if (captureFlag3 == 1) {
                IC_ValueFalling3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                captureFlag3 = 0;
                if (IC_ValueFalling3 > IC_ValueRising3) {
                    captureTime3 = IC_ValueFalling3 - IC_ValueRising3;
                } else {
                    captureTime3 = (0xFFFF - IC_ValueRising3) + IC_ValueFalling3;
                }
                distance3 = captureTime3 / 58;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }
    }
}
