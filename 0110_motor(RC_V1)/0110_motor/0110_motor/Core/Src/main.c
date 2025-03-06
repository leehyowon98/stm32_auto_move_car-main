/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "ultrasonic.h"
#include "dht.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    FORWARD_DIR = 0,   // 정방향
    BACKWARD_DIR = 1,   // 역방향
    RIGHT_TURN = 2,
    LEFT_TURN = 3,
    BREAKING = 4
} Motor_Direction;

typedef enum {
  STOP_CAR = 0,   // RC 대기
  START_CAR = 1 // RC 시작
} CAR_Starting;

typedef enum {
    STOP_MODE = 0,
    MANUAL_MODE = 2, // 수동모드
    SELF_DRIVING_MODE = 3 // 자율주행 모드
} RC_Mode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 64

#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define CIRCLE 'C'
#define CROSS 'X'
#define TRIANGLE 'T'
#define SQUARE 'S'
#define START 'A'
#define PAUSE 'P'
//

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//uint8_t readData_in2;
//uint8_t readData_in3;

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_index = 0;

uint8_t rx_data[64];

uint16_t pwm_num;

uint8_t rc_start = 0; // 테스트 후 삭제
uint8_t rc_mode = 0;    // 테스트 후 삭제
uint8_t ultra_test = 0; // 테스트 후 삭제

uint16_t FL_Dist = 0;
uint16_t FR_Dist = 0;
uint16_t B_Dist = 0;

uint8_t current_state = 0;  // Call the function and store its result

uint8_t temperature = 0;  // 온도 변수 추가
uint8_t humidity = 0;     // 습도 변수 추가

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HC06_SendData(uint8_t *data, uint16_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void Motor_Control(void) {
  // 모터 1 정방향
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  // 모터 2 정방향
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  HAL_Delay(3000);  // 3초 대기

  // 모터 1 역방향
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  // 모터 2 역방향
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

  HAL_Delay(3000);  // 3초 대기

  // 모터 정지
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}

void Motor_PWM_Control(uint16_t pwm_num) {
  // 모터 1 (채널 1, PA6) 속도 설정 (0-1000)
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_num);

  // 모터 2 (채널 2, PA7) 속도 설정 (0-1000)
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_num);

  // PWM 시작 (CubeMX에서 이미 설정했다면 불필요)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void Motor_Direction_Control(Motor_Direction dir) {
//void Motor_Direction_Control() {
  // PB0, PB1: 모터 1 방향 제어
  // PB2, PB3: 모터 2 방향 제어
  switch(dir){
    case FORWARD_DIR:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // 정방향
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   // 정방향
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
      break;

    case BACKWARD_DIR:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
      break;

    case RIGHT_TURN:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
      break;

    case LEFT_TURN:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
      break;
  }

}

uint8_t Self_Driving_state(void){

  uint16_t Dist_Compare;
  Dist_Compare = FL_Dist - FR_Dist;

  if(FL_Dist >= 60 && FR_Dist >= 60){
    return FORWARD;
  }
  else if(FR_Dist >= FL_Dist && FR_Dist > 40){  // 오른쪽으로 치우침
    return RIGHT;
  }
  else if(Dist_Compare > 0 && FL_Dist > 40){  // 왼쪽으로 치우침
    return LEFT;
  }
  else if(B_Dist >= 50){
    return BACKWARD;
  }
  else{
    return BREAKING;
  }

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &ultra_test, 1);
  HAL_TIM_Base_Start(&htim10);  // Delay용

  HC06_StartReceive();
  HCSR04_Init();

  pwm_num = 430;
  uint8_t dht_cnt = 0;
//  uint8_t rc_start = 0;
//  uint8_t rc_mode = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Check if data is available from Bluetooth
    DHT11_Read(&temperature, &humidity);  // DHT11 센서 읽기
//    printf("Distance 1: %d cm, Distance 2: %d cm, Distance 3: %d cm\r\n",
//           Get_Distance1(), Get_Distance2(), Get_Distance3());
//    printf("Temperature: %d°C, Humidity: %d%%\r\n",  // 온습도 출력 추가
//           temperature, humidity);
    dht_cnt++;
    if(dht_cnt >= 40){
      char buffer[50];
      sprintf(buffer, "Temp: %d C, Hum: %d %%\r\n", temperature, humidity);
      HAL_UART_Transmit(&huart6, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
      dht_cnt = 0;
    }
    HCSR04_Read();

    FL_Dist = Get_Distance1() - 3;
    FR_Dist = Get_Distance2();
    B_Dist = Get_Distance3();

    if (HC06_DataAvailable())
    {

      uint16_t data_length = HC06_GetReceivedData(rx_data, sizeof(rx_data));

      if (data_length > 0)    // RC카 시작 / 대기
      {
        switch(rx_data[0]){
          case PAUSE:
            rc_start = STOP_CAR;
            rc_mode = STOP_MODE;
            Motor_PWM_Control(0);        // 브레이크
            break;
          case START:
            rc_start = START_CAR;
            rc_mode = STOP_MODE;
            Motor_PWM_Control(0);        // 브레이크
            break;
          default:
            break;
        }
      }

      if (rc_start == START_CAR && data_length > 0)
      {
        switch(rx_data[0]){
        case SQUARE:
          Motor_PWM_Control(0);        // 브레이크
          HAL_Delay(500);
          rc_mode = MANUAL_MODE;
          break;
        case CIRCLE:
          Motor_PWM_Control(0);        // 브레이크
          HAL_Delay(500);
          rc_mode = SELF_DRIVING_MODE;
          break;
        case TRIANGLE:      //속도 업
          // Perform action for moving forward
          pwm_num += 10;
          break;
        case CROSS:         //속도 다운
          // Perform action for moving backward
          pwm_num -= 10;
          break;
        default:
          break;
        }
      }

      if (rc_mode == MANUAL_MODE && data_length > 0)
      {
        switch(rx_data[0]){
          case FORWARD:        // 전진
            // Perform action for turning left
            Motor_Direction_Control(FORWARD_DIR);  // 방향 설정
            Motor_PWM_Control(pwm_num);        // 속도 제어
            break;
          case BACKWARD:       // 후진
            // Perform action for turning right
            Motor_Direction_Control(BACKWARD_DIR);  // 방향 설정
            Motor_PWM_Control(400);        // 속도 제어
            break;
          case RIGHT:        //우회전
            // Perform action for turning left
            Motor_Direction_Control(RIGHT_TURN);  // 방향 설정
            Motor_PWM_Control(pwm_num - 50);        // 속도 제어
            break;
          case LEFT:       // 좌회전
            // Perform action for turning right
            Motor_Direction_Control(LEFT_TURN);  // 방향 설정
            Motor_PWM_Control(pwm_num - 50);        // 속도 제어
            break;
          default:
            break;
        }
      }

      uint8_t hello_msg[] = "Completed\r\n";
      HC06_SendData(hello_msg, sizeof(hello_msg) - 1);

        // Set flag to prevent further messages
//        hello_sent = 1;
    }

    if (rc_mode == SELF_DRIVING_MODE )
    {
      current_state = Self_Driving_state();  // Call the function and store its result

      switch(current_state){
        case FORWARD:        // 전진
          // Perform action for turning left
          Motor_Direction_Control(FORWARD_DIR);  // 방향 설정
          Motor_PWM_Control(pwm_num);        // 속도 제어
          break;
        case BACKWARD:       // 후진
          // Perform action for turning right
          Motor_Direction_Control(BACKWARD_DIR);  // 방향 설정
          Motor_PWM_Control(pwm_num);        // 속도 제어
          break;
        case RIGHT:        //우회전
          // Perform action for turning left
          Motor_Direction_Control(RIGHT_TURN);  // 방향 설정
          Motor_PWM_Control(pwm_num - 50);        // 속도 제어
          break;
        case LEFT:       // 좌회전
          // Perform action for turning right
          Motor_Direction_Control(LEFT_TURN);  // 방향 설정
          Motor_PWM_Control(pwm_num - 50);        // 속도 제어
          break;
        case BREAKING:
          Motor_PWM_Control(0);        // 브레이크
          break;
        default:
          break;
      }
    }
  }

    HAL_Delay(100);  // Small delay to prevent blocking


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
