/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "crc.h"
#include <stdio.h>
#include <string.h>
#include "main_cpp.hpp"
#include "BufferC.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
#define UART_BUF_LEN 255
unsigned char UART3_rxBuffer[UART_BUF_LEN];

#define ADC_BUF_LEN 4
uint32_t adc_buf[ADC_BUF_LEN];

#define ADC_IN1_PA0_POUT     0
#define ADC_IN11_PB0_PROX    1
#define ADC_IN6_PC0_I_MOT    2
#define ADC_IN7_PC1_S_MOT    3

#define SFM3219_ADDRESS     0x2E
#define SFM3219_START_AIR   0x3608

typedef enum{
    I2C_INIT = 0,
    I2C_READ = 1
}i2c_states;

int32_t offsetPout = -366;
int32_t offsetPprox = -366;

#define MEASURES_BUF_LEN 5
int32_t Measures[MEASURES_BUF_LEN];
#define MEAS_QOUT     0
#define MEAS_POUT     1
#define MEAS_PPROX    2
#define MEAS_I_MOT    3
#define MEAS_S_MOT    4

char const* data_names[] =
{
        "time",
        "qout",
        "pout",
        "pprox",
        "mot_speed",
        "mot_current"
};

uint32_t target_motor_speed     = 20000;    // 20000 rpm
uint32_t target_motor_qout      = 1000;     // 10 L/min

uint32_t cmd_motor              = 0; //100;  // 10%
uint32_t cmd_peep               = 0; //200;  // 20%
uint32_t cmd_valve              = 0;    // 0%

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void printFloats(float in[], int size)
{
    char buffer[80]="";

    for (int i = 0; i < size; i++)
    {
        char txt[10];
        sprintf(txt, "%.2f ", in[i]);
        strcat(buffer, txt);
    }
    strcat(buffer, "\n");
    //HAL_UART_Transmit(&huart3, buffer, strlen(buffer), 100);
    HAL_UART_Transmit_DMA(&huart3, buffer, strlen(buffer));
}


void printFloatsTelePlot(float in[], char const* names[], int size)
{
    char buffer[256]="";

    for (int i = 0; i < size; i++)
    {
        char txt[64];
        sprintf(txt, ">%s:%.2f ", names[i], in[i]);
        strcat(buffer, txt);
        strcat(buffer, "\r\n");
    }
    strcat(buffer, "\0");

    HAL_UART_Transmit_DMA(&huart3, buffer, strlen(buffer));
}

const int32_t MAX_CURRENT   = 300;
const int32_t MAX_SPEED     = 60000;
const int32_t MAX_RAW_ADC   = 4095;
const int32_t MAX_RAW_I2C   = 0xFFFF;

const int32_t PRESSURE_OFFSET   = 300;
const int32_t PRESSURE_GAIN     = 60000;

int32_t VoltageTo01mbar(int32_t volt, int32_t offset)
{
    return ( ( ( ( volt - 33 ) * 13790 ) / 264 ) - 6895) - offset;
}

int32_t RawADCToVolt(int32_t raw)
{
    return raw * 330 / MAX_RAW_ADC;
}

int32_t RawSFM3019ToLmin(int32_t raw, int32_t offset)
{
    return ( ( 100 * ( raw + 24576 ) ) / 170 ) - offset;
}

int32_t RawToCal(int32_t raw, int32_t max_cal, int32_t max_raw)
{
    return (raw * max_cal) / max_raw;
}

void UpdatePWM1(uint32_t per1000)
{
    TIM15->CCR1 = (per1000 * TIM15_PERIOD_TICKS) /1000;
}

void UpdatePWM2(uint32_t per1000)
{
    TIM15->CCR2 = (per1000 * TIM15_PERIOD_TICKS) /1000;
}

void UpdatePWM3(uint32_t per1000)
{
    TIM2->CCR1 = (per1000 * TIM2_PERIOD_TICKS) /1000;
}

int CMD_MOTOR_INSPI     = 2500;
int CMD_MOTOR_EXPI      = 500;

int INSPI_TIME          = 3000; //1000;
int EXPI_TIME           = 3000; //2000;


uint8_t cmd[3] = {0x36, 0x08, 0x00};
uint8_t crc = 0;
uint8_t i2c_state = 0;
uint8_t i2c_retry = 0;

int32_t rawQout = 0;
int32_t offsetQout = -57;

void InitQoutSensor()
{
    crc = SF04_CalcCrc (cmd, 2);
    cmd[2] = crc;

    HAL_GPIO_WritePin(CmdQout_GPIO_Port, CmdQout_Pin, GPIO_PIN_SET);
    i2c_state = I2C_INIT;
}


void ReadQoutSensor()
{
    static int tick_i2c = 0;
    rawQout = -24576;

    if (i2c_state == I2C_INIT)
    {
        if (tick_i2c <= 300)
        {
            HAL_GPIO_WritePin(CmdQout_GPIO_Port, CmdQout_Pin, GPIO_PIN_SET);
        }
        else if (tick_i2c <= 400)
        {
            HAL_GPIO_WritePin(CmdQout_GPIO_Port, CmdQout_Pin, GPIO_PIN_RESET);
        }
        else
        {
            uint8_t status = HAL_I2C_Master_Transmit(&hi2c1, SFM3219_ADDRESS<<1, cmd, 3, 1000);
            if (status == HAL_OK)
            {
                i2c_state = I2C_READ;
                i2c_retry = 0;
            }
            else
            {
                HAL_UART_Transmit(&huart3, "INIT_ERROR\n\0", strlen("INIT_ERROR\n\0"), 100);
            }
            tick_i2c = 0;
        }
        tick_i2c++;
    }
    else if (i2c_state == I2C_READ)
    {
        uint8_t i2c_rcv_buff[3] = {0x00, 0x00, 0x00};
        uint8_t status = HAL_I2C_Master_Receive(&hi2c1, SFM3219_ADDRESS<<1, i2c_rcv_buff, 3, 1000);
        if (status == HAL_OK)
        {
            if (SF04_CheckCrc (i2c_rcv_buff, 2, i2c_rcv_buff[2]) != CHECKSUM_ERROR)
            {
                rawQout = (int16_t)(((uint16_t)i2c_rcv_buff[0])<<8 | i2c_rcv_buff[0]);
            }
            else
            {
                HAL_UART_Transmit(&huart3, "CHECKSUM_ERROR\n\0", strlen("CHECKSUM_ERROR\n\0"), 100);
                i2c_retry++;
            }
        }
        else
        {
            HAL_UART_Transmit(&huart3, "READING_ERROR\n\0", strlen("READING_ERROR\n\0"), 100);
            i2c_retry++;
        }
        if (i2c_retry >= 10)
        {
            tick_i2c = 0;
            i2c_state = I2C_INIT;
        }
    }
}


void UpdateMeasurements()
{
    Measures[MEAS_QOUT]   = RawSFM3019ToLmin(rawQout, offsetQout);
    Measures[MEAS_POUT]   = VoltageTo01mbar( RawADCToVolt( adc_buf[ADC_IN1_PA0_POUT] ), offsetPout );
    Measures[MEAS_PPROX]  = VoltageTo01mbar( RawADCToVolt( adc_buf[ADC_IN11_PB0_PROX] ), offsetPprox );
    Measures[MEAS_I_MOT]  = RawToCal(adc_buf[ADC_IN6_PC0_I_MOT], MAX_CURRENT, MAX_RAW_ADC);
    Measures[MEAS_S_MOT]  = RawToCal(adc_buf[ADC_IN7_PC1_S_MOT], MAX_SPEED, MAX_RAW_ADC);
}

void PrintMeasurements()
{
    float tick = HAL_GetTick();
    tick /= 1000;

    float measures[6];
    measures[0] = tick;
    measures[1] = ((float)Measures[MEAS_QOUT])/100;
    measures[2] = ((float)Measures[MEAS_POUT])/100;
    measures[3] = ((float)Measures[MEAS_PPROX])/100;
    measures[4] = ((float)Measures[MEAS_S_MOT]);
    measures[5] = ((float)Measures[MEAS_I_MOT])/100;


    //printFloats(measures, 6);
    printFloatsTelePlot(measures, data_names, 6);
}


void Tick_1ms()
{
    static int time = 0;
    static int time_ini = 0;
    static int time_duration = 0;

    ReadQoutSensor();
    UpdateMeasurements();

    if (time%10 == 0)
    {
        PrintMeasurements();
    }

    time += 1;

    cmd_peep = 500;
    if ( (time - time_ini) > time_duration )
    {
        if (cmd_motor == CMD_MOTOR_INSPI)
        {
            cmd_motor = 4*CMD_MOTOR_EXPI;
            cmd_valve = 0;
            time_duration = EXPI_TIME;
        }
        else
        {
            cmd_motor = CMD_MOTOR_INSPI;
            cmd_valve = 999;
            time_duration = INSPI_TIME;
        }
        time_ini = time;
    }

    // cmd_motor
    // Calculate new target motor
    // 3.3V - 4095 -> 45'000 rpm
    //int error = target_motor_qout - Measures[MEAS_QOUT];
    //int cmd_target_tmp = (int)cmd_motor + (error / 200);
    //if (cmd_target_tmp < 0) cmd_target_tmp = 0;
    //if (cmd_target_tmp > 4095) cmd_target_tmp = 4095;
    //cmd_motor = (uint32_t)cmd_target_tmp;

    DAC1->DHR12R1 = cmd_motor%4096;

    UpdatePWM1(cmd_valve);
    UpdatePWM2(cmd_peep);

    UpdatePWM3(cmd_valve);

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

  // TODO: !!! DMA to be initialised before ADC and other peripherals !!! ////////
  //MX_GPIO_Init();
  //MX_USART2_UART_Init();
  //MX_DMA_Init();
  //MX_ADC1_Init();
  //MX_USART3_UART_Init();
  //MX_DAC_Init();
  //MX_I2C1_Init();
  //MX_TIM15_Init();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  // ??? TODO: !!! Comment in MX_DMA_Init() : HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn); --> No interrupt required

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  UpdatePWM1(0);
  UpdatePWM2(0);
  TIM15->CCER |= TIM_CCER_CC1E;
  TIM15->CCER |= TIM_CCER_CC2E;
  HAL_TIMEx_PWMN_Start(&htim15, HAL_TIM_ACTIVE_CHANNEL_1);

  UpdatePWM3(0);
  TIM2->CCER |= TIM_CCER_CC1E;
  HAL_TIMEx_PWMN_Start(&htim2, HAL_TIM_ACTIVE_CHANNEL_1);

  // !!! Start UART before ADC  !!! ////////
  HAL_UART_Receive_IT(&huart3, UART3_rxBuffer, 1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

  InitQoutSensor();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //char buffer [50];
  while (1)
  {
	  HAL_Delay(10);
	  //(void) main_cpp();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM15
                              |RCC_PERIPHCLK_ADC1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_PLLCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  DAC1->DHR12R1 = 0;
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  /* USER CODE END DAC_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_PERIOD_TICKS;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = TIM15_PERIOD_TICKS;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim15, TIM_CHANNEL_1);
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim15, TIM_CHANNEL_2);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CmdQout_GPIO_Port, CmdQout_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CmdQout_Pin */
  GPIO_InitStruct.Pin = CmdQout_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CmdQout_GPIO_Port, &GPIO_InitStruct);

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
