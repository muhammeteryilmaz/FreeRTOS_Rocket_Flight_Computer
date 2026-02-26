/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "stdio.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// -----------------MPU6050------------------
#define MPU6050_ADDRESS		0x68<<1

#define MPU6050_CONFIG		0x1A
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define ACCEL_MSB			0x3B	//	Read until 0x40 -> 48 bits for ACCEL, 6 bytes
#define TEMP_MSB			0x41	//	Read until 0x41 -> 16 bits for Temp
#define GYRO_MSB			0x43
#define RESET_ADDRESS		0x68
#define SMPRT_DIV			0x19
#define PWR_MGMT_1			0x6B

// -----------------BME280------------------
#define BME280_I2C_ADDRESS   0x76 << 1  // BME280 I2C address (SDO = GND => 0x76, SDO = VCC => 0x77)
#define BME280_REG_ID        0xD0
#define BME280_REG_RESET     0xE0
#define BME280_REG_CTRL_HUM  0xF2
#define BME280_REG_STATUS    0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5
#define BME280_REG_PRESS_MSB 0xF7
#define BME280_REG_TEMP_MSB  0xFA
#define BME280_REG_HUM_MSB   0xFD

#define BME280_CALIB_DATA_START  0x88
#define BME280_CALIB_DATA_END    0xA1
// Calibration Rgister Adresses
#define BME280_CALIB_DATA_START  0x88
#define BME280_CALIB_DATA_END    0xA1
#define BME280_CALIB_HUM_START   0xE1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

/* Definitions for defaultTask */
/*osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};*/
/* USER CODE BEGIN PV */

// -----------------MPU6050------------------
uint8_t config = 0x00;
uint8_t smprtDIV = 0x00;
uint8_t gyroConfig = 0x18;
uint8_t accelConfig = 0x18;

int16_t accelX = 0x00;
int16_t accelY = 0x00;
int16_t accelZ = 0x00;

int16_t gyroX = 0x00;
int16_t gyroY = 0x00;
int16_t gyroZ = 0x00;

uint8_t pwrMgnt1 = 0x00;

uint8_t reset = 0x07;
uint8_t address = 0;

typedef struct {
	float aX;
	float aY;
	float aZ;

	float gX;
	float gY;
	float gZ;
} IMUData_t;

float gyX;
float gyY;
float gyZ;

QueueHandle_t xIMUQueue;
int counter = 0;
// -----------------BME280------------------
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int16_t dig_H2, dig_H4, dig_H5;
int8_t dig_H6;



typedef struct {
	  float temperature, pressure;
	  uint32_t humidity;
} BMEData_t;

float temp, press;
uint32_t hum;

QueueHandle_t xBMEQueue;

uint8_t isQueueCreated;
uint8_t isControlCreated;
int bmeAddress;
HAL_StatusTypeDef status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
// -----------------MPU6050------------------
void MPU6050_Init(void);
void readAccelData(IMUData_t *data);
void readGyroData(IMUData_t *data);
void vIMUTask(void *pvParameters);

// -----------------BME280------------------
void BME280_Init();
void BME280_Read_Calibration();
void BME280_Read_Data(BMEData_t *bmeData);
int32_t BME280_Compensate_Temperature(int32_t adc_T);
uint32_t BME280_Compensate_Pressure(int32_t adc_P);
uint32_t BME280_Compensate_Humidity(int32_t adc_H);
void vBMETask(void *pvParameters);
void findBME(void);

// -----------------Motor------------------
void ServoInit(void);
void ServoStop(void);
void ServoRun(void);
// -----------------Control Task------------------
void vControlTask(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  findBME();
  HAL_Delay(100);
  BME280_Read_Calibration();
  BME280_Init();
  MPU6050_Init();
  ServoInit();

  xIMUQueue = xQueueCreate(5, sizeof(IMUData_t));
  xBMEQueue = xQueueCreate(5, sizeof(BMEData_t));


  if( xIMUQueue != NULL && xBMEQueue != NULL){
	  isQueueCreated = 1;
	  xTaskCreate(vIMUTask, "ImuTask", 1024, NULL, 3, NULL);
	  xTaskCreate(vBMETask, "BmeTask", 1024, NULL, 2, NULL);
	  xTaskCreate(vControlTask, "ControlTask", 1024, NULL, 4, NULL);

	  vTaskStartScheduler();
  }else{
	  isQueueCreated = 0;
  }


  for(;;);


  /* USER CODE END 2 */

  /* Init scheduler */
  //osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// -----------------MPU6050------------------

void MPU6050_Init(void){

	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, PWR_MGMT_1, 1, &pwrMgnt1, 1, HAL_MAX_DELAY);

	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, RESET_ADDRESS, 1, &reset, 1, HAL_MAX_DELAY);

	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, SMPRT_DIV, 1, &smprtDIV, 1, HAL_MAX_DELAY);

	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_CONFIG, 1, &config, 1, HAL_MAX_DELAY);

	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, ACCEL_CONFIG, 1, &accelConfig, 1, HAL_MAX_DELAY);

	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, GYRO_CONFIG, 1, &gyroConfig, 1, HAL_MAX_DELAY);


}

void readAccelData(IMUData_t *data){
	uint8_t accelData[6];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, ACCEL_MSB, 1, accelData, 6, HAL_MAX_DELAY);

	accelX = ((int16_t)accelData[0] << 8) | (int16_t)accelData[1];
	accelY = ((int16_t)accelData[2] << 8) | (int16_t)accelData[3];
	accelZ = ((int16_t)accelData[4] << 8) | (int16_t)accelData[5];

	data->aX = (float)accelX/2048.0;
	data->aY = (float)accelY/2048.0;
	data->aZ = (float)accelZ/2048.0;

}

void readGyroData(IMUData_t *data){
	uint8_t gyroData[6];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, GYRO_MSB, 1, gyroData, 6, HAL_MAX_DELAY);

	gyroX = ((int16_t)gyroData[0] << 8) | (int16_t)gyroData[1];
	gyroY = ((int16_t)gyroData[2] << 8) | (int16_t)gyroData[3];
	gyroZ = ((int16_t)gyroData[4] << 8) | (int16_t)gyroData[5];

	data->gX = (float)gyroX/16.4;
	data->gY = (float)gyroY/16.4;
	data->gZ = (float)gyroZ/16.4;

}

void vIMUTask(void *pvParameters){
	TickType_t lastWakeTime = xTaskGetTickCount();
	IMUData_t data;
	BaseType_t xStatus;

	for(;;){
		readAccelData(&data);
		readGyroData(&data);
		xStatus = xQueueSend(xIMUQueue, &data, 50);
		if(xStatus == pdPASS){
			// I turn on led to warning
			vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50));
		}
	}

}

// -----------------BME280------------------
void BME280_Init() {
    uint8_t id;
    HAL_I2C_Mem_Read(&hi2c2, BME280_I2C_ADDRESS, BME280_REG_ID, 1, &id, 1, HAL_MAX_DELAY);
    if (id != 0x60) {
        printf("BME280 Could not find!\n");
    }

    uint8_t reset_cmd = 0xB6;
    HAL_I2C_Mem_Write(&hi2c2, BME280_I2C_ADDRESS, BME280_REG_RESET, 1, &reset_cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(100);

    uint8_t filter = 0xA4;
    HAL_I2C_Mem_Write(&hi2c2, BME280_I2C_ADDRESS, BME280_REG_CONFIG, 1, &filter, 1, HAL_MAX_DELAY);

    BME280_Read_Calibration();
    HAL_Delay(100);
    uint8_t ctrl_hum = 0x01;
    HAL_I2C_Mem_Write(&hi2c2, BME280_I2C_ADDRESS, BME280_REG_CTRL_HUM, 1, &ctrl_hum, 1, HAL_MAX_DELAY);

    uint8_t ctrl_meas = 0x67;
    HAL_I2C_Mem_Write(&hi2c2, BME280_I2C_ADDRESS, BME280_REG_CTRL_MEAS, 1, &ctrl_meas, 1, HAL_MAX_DELAY);
}

void BME280_Read_Calibration() {
    uint8_t calib_data[26];
    status=HAL_I2C_Mem_Read(&hi2c2, BME280_I2C_ADDRESS, BME280_CALIB_DATA_START, 1, calib_data, 26, HAL_MAX_DELAY);

    if(status != HAL_OK){

        return;
    }
    dig_T1 = (calib_data[1] << 8) | calib_data[0];
    dig_T2 = (calib_data[3] << 8) | calib_data[2];
    dig_T3 = (calib_data[5] << 8) | calib_data[4];

    dig_P1 = (calib_data[7] << 8) | calib_data[6];
    dig_P2 = (calib_data[9] << 8) | calib_data[8];
    dig_P3 = (calib_data[11] << 8) | calib_data[10];
    dig_P4 = (calib_data[13] << 8) | calib_data[12];
    dig_P5 = (calib_data[15] << 8) | calib_data[14];
    dig_P6 = (calib_data[17] << 8) | calib_data[16];
    dig_P7 = (calib_data[19] << 8) | calib_data[18];
    dig_P8 = (calib_data[21] << 8) | calib_data[20];
    dig_P9 = (calib_data[23] << 8) | calib_data[22];

    uint8_t hum_data[7];
    HAL_I2C_Mem_Read(&hi2c2, BME280_I2C_ADDRESS, BME280_CALIB_HUM_START, 1, hum_data, 7, HAL_MAX_DELAY);

    dig_H1 = hum_data[0];
    dig_H2 = (hum_data[2] << 8) | hum_data[1];
    dig_H3 = hum_data[3];
    dig_H4 = (hum_data[4] << 4) | (hum_data[5] & 0x0F);
    dig_H5 = (hum_data[6] << 4) | (hum_data[5] >> 4);
    dig_H6 = hum_data[6];
}

void BME280_Read_Data(BMEData_t *bmeData) {
    uint8_t data[8];
    HAL_I2C_Mem_Read(&hi2c2, BME280_I2C_ADDRESS, BME280_REG_PRESS_MSB, 1, data, 8, HAL_MAX_DELAY);

    int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
    int32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4);
    int32_t adc_H = ((uint32_t)data[6] << 8) | (uint32_t)data[7];

    bmeData->temperature = (float)BME280_Compensate_Temperature(adc_T) / 100.0f;
    bmeData->pressure = (float)BME280_Compensate_Pressure(adc_P) / 256.0f / 100.0f;
    bmeData->humidity = BME280_Compensate_Humidity(adc_H);

}

int32_t BME280_Compensate_Temperature(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    T = (((var1 + var2) * 5) + 128) >> 8;
    return T;
}

uint32_t BME280_Compensate_Pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)adc_P) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)dig_P1)) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p;
}

uint32_t BME280_Compensate_Humidity(int32_t adc_H) {
    return adc_H;
}

void vBMETask(void *pvParameters){
	TickType_t lastWakeTime = xTaskGetTickCount();
	BMEData_t data;
	BaseType_t xStatus;

	for(;;){
		BME280_Read_Data(&data);
		xStatus = xQueueSend(xBMEQueue, &data, 0);

		if(xStatus == pdPASS){
			// I turn on led to warning
			vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(57));
		}
	}
}

void findBME(void){
    for(int i = 0; i <= 255; i++){
        if(HAL_I2C_IsDeviceReady(&hi2c2, i, 1, 10) == HAL_OK){
            bmeAddress = i;
            HAL_Delay(50);
        }
    }
}

// -----------------Motor-----------------

void ServoInit(void){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void ServoStop(void){
	TIM1->CCR1 = 1500;
}

void ServoRun(void){
	TIM1->CCR1 = 2000;
}


// -----------------Control Task------------------
void vControlTask(void *pvParameters){
	TickType_t lastWakeTime = xTaskGetTickCount();
	BaseType_t xImuStatus;
	BaseType_t xBmeStatus;
	IMUData_t imuData;
	BMEData_t bmeData;
	float prevPressure = 0;
	isControlCreated=0;

	for(;;){
		xImuStatus = xQueueReceive(xIMUQueue, &imuData, 100);
		xBmeStatus = xQueueReceive(xBMEQueue, &bmeData, 200);
		isControlCreated=2;
		if(xImuStatus && xBmeStatus){
			gyX = imuData.gX;
			gyY = imuData.gY;
			gyZ = imuData.gZ;

			temp = bmeData.temperature;
			press = bmeData.pressure;
			hum = bmeData.humidity;
			isControlCreated=1;
			if((press-prevPressure) <= -5.00){
				ServoRun();
				vTaskDelay(pdMS_TO_TICKS(1000));
				ServoStop();
			}

			prevPressure = press;
			vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
