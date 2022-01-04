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
#include "math.h"
#include "LIS3DSH.h"
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
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define MAX_LED 5
#define USE_BRIGHTNESS 0

uint8_t LED_Data[MAX_LED][4];

uint8_t LED_Mod[MAX_LED][4]; // for brightness 0 - 45

int datasentflag=0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag = 1;
}

void Set_LED(int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0]=LEDnum;
	LED_Data[LEDnum][1]=Green;
	LED_Data[LEDnum][2]=Red;
	LED_Data[LEDnum][3]=Blue;
}

#define PI 3.14159265

void Set_Brightness (int brightness) //
{
	#if USE_BRIGHTNESS

	if (brightness>45) brightness=45;
	for (int i =0; i<MAX_LED; i++)
		{
			LED_Mod[i][0] = LED_Data[i][0];
			for (int j=1; j<4; j++)
			{
				float angle = 90-brightness;
				angle = angle*PI/180;
				LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
			}

		}

	#endif

}

uint16_t pwmData[(24*MAX_LED)+50];

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 60;  // 2/3 of 90
			}

			else pwmData[indx] = 30;  // 1/3 of 90

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}

void SPICongig(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 |= SPI_CR1_CPOL;
	SPI1->CR1 |= SPI_CR1_CPHA;
	SPI1->CR1 |= SPI_CR1_MSTR;
	SPI1->CR1 |= SPI_CR1_BR_2;
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST; // msb first
	SPI1->CR1 |= SPI_CR1_SSI;
	SPI1->CR1 |= SPI_CR1_SSM;
	SPI1->CR1 &= ~SPI_CR1_RXONLY; // full duplex
	SPI1->CR1 &= ~SPI_CR1_DFF; // 8bit format

	SPI1->CR2 = 0;
}

void GPIOConfig(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable clock

	GPIOA->MODER |= GPIO_MODER_MODE7_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE7_0;

	GPIOA->MODER |= GPIO_MODER_MODE6_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE6_0;

	GPIOA->MODER |= GPIO_MODER_MODE5_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE5_0;

	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_0;

	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_0;

	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_0;

	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL7_3;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL7_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL7_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL7_0;

	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL6_3;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL6_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL6_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL6_0;

	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL5_3;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL5_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL5_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL5_0;


	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

	GPIOE->MODER &= ~GPIO_MODER_MODE3_1;
	GPIOE->MODER |= GPIO_MODER_MODE3_0;

	GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3_1;
	GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3_0;

}

void SPI_Enable (void)
{
	SPI1->CR1 |= SPI_CR1_SPE;
}

void SPI_Disable (void)
{
	SPI1->CR1 &= ~SPI_CR1_SPE;
}

void CS_Enable(void)
{
	GPIOE->BSRR |= GPIO_BSRR_BR3;
}

void CS_Disable(void)
{
	GPIOE->BSRR |= GPIO_BSRR_BS3;
}


void SPI_Transmit(uint8_t *data, int size)
{
	int i = 0;
	while (i<size)
	{
		while(!((SPI1->SR) & (1<<1))) {};
		SPI1->DR = data[i];
		i++;
	}

	while(!(SPI1->SR & SPI_SR_TXE)) {}; // wait for TXE bit to set -> This will indicate that the buffer is empty
	while(((SPI1->SR) & (1<<7))) {}; // wait for BSY bit to reset -> This will indicate that SPI is not busy in communication

	// Clearing the Overrun flg by reading DR and SR
	uint8_t temp = SPI1->DR;
		temp = SPI1->SR;

}

void SPI_Receive (uint8_t *data, int size)
{
	while (size)
	{
		while(((SPI1->SR) & (1<<7))) {}; // wait for BSY bit to reset -> This will indicate that SPI is not busy in communication
		SPI1->DR = 0;
		while(!(SPI1->SR & SPI_SR_TXE)) {}; //  Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
		Delay_us(500);
		*data++ = (SPI1->DR);
		size--;
	}
}

void LIS3DSH_Write (uint8_t *data, uint8_t addr, uint8_t NumberOfBytes)
{
	CS_Disable();
	if (NumberOfBytes>1)
	{
		addr |= 0x40;
	}
	CS_Enable();
	SPI_Transmit(&addr, 1);
	SPI_Transmit(data, NumberOfBytes);
	CS_Disable();
}

uint8_t RxData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Read
void LIS3DSH_Read (uint8_t *data, uint8_t addr, uint8_t NumberOfBytes )
{
	addr |= 0x80; //read operation

	if (NumberOfBytes>1)
	{
		addr|= 0x40; // multibyte read
	}

	CS_Enable();
	SPI_Transmit(&addr, 1);
	SPI_Receive (data, NumberOfBytes);
	CS_Disable();
}


double LIS3DSH_INT_Sensitivity = 0;
// Sensitivity = LIS3DSH_Sensitivity_2G;
//LIS3DSH_Filter_t Filter = LIS3DSH_Filter_800Hz;
void LIS3DSH_Initialise(LIS3DSH_Sensitivity_t Sensitivity, LIS3DSH_Filter_t Filter) {
	uint8_t tmpreg;
	uint16_t temp;

	/* Set data */
	temp = (uint16_t) (LIS3DSH_DATARATE_100 | LIS3DSH_XYZ_ENABLE);
	temp |= (uint16_t) (LIS3DSH_SERIALINTERFACE_4WIRE | LIS3DSH_SELFTEST_NORMAL);

	/* Set sensitivity */
	if (Sensitivity == LIS3DSH_Sensitivity_2G) {
		temp |= (uint16_t) (LIS3DSH_FULLSCALE_2);
		LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_06G;
	} else if (Sensitivity == LIS3DSH_Sensitivity_4G) {
		temp |= (uint16_t) (LIS3DSH_FULLSCALE_4);
		LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_12G;
	} else if (Sensitivity == LIS3DSH_Sensitivity_6G) {
		temp |= (uint16_t) (LIS3DSH_FULLSCALE_6);
		LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_18G;
	} else if (Sensitivity == LIS3DSH_Sensitivity_8G) {
		temp |= (uint16_t) (LIS3DSH_FULLSCALE_8);
		LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_24G;
	} else if (Sensitivity == LIS3DSH_Sensitivity_16G) {
		temp |= (uint16_t) (LIS3DSH_FULLSCALE_16);
		LIS3DSH_INT_Sensitivity = LIS3DSH_SENSITIVITY_0_73G;
	} else {
		return;
	}

	/* Set filter */
	if (Filter == LIS3DSH_Filter_800Hz) {
		temp |= (uint16_t) (LIS3DSH_FILTER_BW_800 << 8);
	} else if (Filter == LIS3DSH_Filter_400Hz) {
		temp |= (uint16_t) (LIS3DSH_FILTER_BW_400 << 8);
	} else if (Filter == LIS3DSH_Filter_200Hz) {
		temp |= (uint16_t) (LIS3DSH_FILTER_BW_200 << 8);
	} else if (Filter == LIS3DSH_Filter_50Hz) {
		temp |= (uint16_t) (LIS3DSH_FILTER_BW_50 << 8);
	} else {
		return;
	}

	/* Configure MEMS: power mode(ODR) and axes enable */
	tmpreg = (uint8_t) (temp);

	/* Write value to MEMS CTRL_REG4 register */
	LIS3DSH_Write(&tmpreg, LIS3DSH_CTRL_REG4_ADDR, 1);

	/* Configure MEMS: full scale and self test */
	tmpreg = (uint8_t) (temp >> 8);

	/* Write value to MEMS CTRL_REG5 register */
	LIS3DSH_Write(&tmpreg, LIS3DSH_CTRL_REG5_ADDR, 1);
}

LIS3DSH_t AxesData;

void TM_LIS3DSH_INT_ReadAxes(LIS3DSH_t *Axes_Data) {
	int8_t buffer[6];

	LIS3DSH_Read((uint8_t*)&buffer[0], LIS3DSH_OUT_X_L_ADDR, 1);
	LIS3DSH_Read((uint8_t*)&buffer[1], LIS3DSH_OUT_X_H_ADDR, 1);
	LIS3DSH_Read((uint8_t*)&buffer[2], LIS3DSH_OUT_Y_L_ADDR, 1);
	LIS3DSH_Read((uint8_t*)&buffer[3], LIS3DSH_OUT_Y_H_ADDR, 1);
	LIS3DSH_Read((uint8_t*)&buffer[4], LIS3DSH_OUT_Z_L_ADDR, 1);
	LIS3DSH_Read((uint8_t*)&buffer[5], LIS3DSH_OUT_Z_H_ADDR, 1);

	/* Set axes */
	Axes_Data->X = (int16_t)((buffer[1] << 8) + buffer[0])*LIS3DSH_INT_Sensitivity;
	Axes_Data->Y = (int16_t)((buffer[3] << 8) + buffer[2])*LIS3DSH_INT_Sensitivity;
	Axes_Data->Z = (int16_t)((buffer[5] << 8) + buffer[4])*LIS3DSH_INT_Sensitivity;
}

void LedConfig()
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER|=GPIO_MODER_MODE12_0;
	GPIOD->MODER|=GPIO_MODER_MODE13_0;
	GPIOD->MODER|=GPIO_MODER_MODE14_0;
	GPIOD->MODER|=GPIO_MODER_MODE15_0;


}

void LedsOff()
{
	GPIOD->BSRR|=GPIO_BSRR_BR12;
	GPIOD->BSRR|=GPIO_BSRR_BR13;
	GPIOD->BSRR|=GPIO_BSRR_BR14;
	GPIOD->BSRR|=GPIO_BSRR_BR15;
}

void TIMconfig (void)
{
// TIM 6 config
	RCC->APB1ENR |= (1<<4); // enable timer6 clock
	TIM6->PSC = 84-1; // 84MHz/(83+1)=1MHz = 1uS delay
	TIM6->ARR = 0xffff; // MAX ARR value
	TIM6->CR1 |= (1<<0); // enable the counter and wait for the update flag to set
	while (!(TIM6->SR & (1<<0))); // UIF: This bit is set by hardware when the registers are updated
}

void Delay_us (uint16_t us)
{
	TIM6->CNT = 0; // Reset the counter
	while (TIM6->CNT < us); // Wait the counter to reach entered value. Each count is 1us delay
}

void Delay_ms (uint16_t ms)
{
	for  (uint16_t i = 0; i<ms; i++)
	{
		Delay_us(1000); // delay of 1ms;
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
  MX_DMA_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  GPIOConfig();
  LedConfig();
  TIMconfig();
  SPICongig();
  CS_Disable();
  SPI_Enable();

  LIS3DSH_Initialise(LIS3DSH_Sensitivity_2G, LIS3DSH_Filter_50Hz);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {




	  TM_LIS3DSH_INT_ReadAxes(&AxesData);

	  if (AxesData.Y>-150 && AxesData.Y<150 )
	 	  	 {
	 		  	Set_LED(0, 0, 0, 0);
	 		    Set_LED(1, 0, 0, 0);
	 		    Set_LED(2, 0, 0, 0);
	 		    Set_LED(3, 0, 0, 0);
	 		    Set_LED(4, 0, 0, 0);
	 	  	 }

	  if (AxesData.Y>150 && AxesData.Y<300 )
	  	 {
		  	Set_LED(0, 0, 0, 255);
		    Set_LED(1, 0, 0, 0);
		    Set_LED(2, 0, 0, 0);
		    Set_LED(3, 0, 0, 0);
		    Set_LED(4, 0, 0, 0);
	  	 }
	  if (AxesData.Y>300 && AxesData.Y<450)
	  	 {
		  	Set_LED(0, 0, 0, 0);
			   Set_LED(1, 0, 247, 255);
			    Set_LED(2, 0, 0, 0);
			    Set_LED(3, 0, 0, 0);
			    Set_LED(4, 0, 0, 0);
	  	 }
	  if (AxesData.Y>450 && AxesData.Y<600)
	  	 {
		  	Set_LED(0, 0, 0, 0);
			    Set_LED(1, 0, 0, 0);
			    Set_LED(2, 0, 255, 0);
			    Set_LED(3, 0, 0, 0);
			    Set_LED(4, 0, 0, 0);
	  	 }
	  if (AxesData.Y>600 && AxesData.Y<750)
	  	 {
		  	Set_LED(0, 0, 0, 0);
			    Set_LED(1, 0, 0, 0);
			    Set_LED(2, 0, 0, 0);
			    Set_LED(3, 255, 238, 0);
			    Set_LED(4, 0, 0, 0);
	  	 }
	  if (AxesData.Y>750)
	  	 {
		  	Set_LED(0, 0, 0, 0);
			    Set_LED(1, 0, 0, 0);
			    Set_LED(2, 0, 0, 0);
			    Set_LED(3, 0, 0, 0);
			    Set_LED(4, 255, 0, 0);
	  	 }

	  if (AxesData.Y<-150 && AxesData.Y>-300 )
	  	  	 {
	  		  	Set_LED(0, 0, 0, 255);
	  		    Set_LED(1, 0, 0, 0);
	  		    Set_LED(2, 0, 0, 0);
	  		    Set_LED(3, 0, 0, 0);
	  		    Set_LED(4, 0, 0, 0);
	  	  	 }
	  	  if (AxesData.Y<-300 && AxesData.Y>-450)
	  	  	 {
	  		  	Set_LED(0, 0, 0, 0);
	  			   Set_LED(1, 0, 247, 255);
	  			    Set_LED(2, 0, 0, 0);
	  			    Set_LED(3, 0, 0, 0);
	  			    Set_LED(4, 0, 0, 0);
	  	  	 }
	  	  if (AxesData.Y<-450 && AxesData.Y>-600)
	  	  	 {
	  		  	Set_LED(0, 0, 0, 0);
	  			    Set_LED(1, 0, 0, 0);
	  			    Set_LED(2, 0, 255, 0);
	  			    Set_LED(3, 0, 0, 0);
	  			    Set_LED(4, 0, 0, 0);
	  	  	 }
	  	  if (AxesData.Y<-600 && AxesData.Y>-750)
	  	  	 {
	  		  	Set_LED(0, 0, 0, 0);
	  			    Set_LED(1, 0, 0, 0);
	  			    Set_LED(2, 0, 0, 0);
	  			    Set_LED(3, 255, 238, 0);
	  			    Set_LED(4, 0, 0, 0);
	  	  	 }
	  	  if (AxesData.Y<-750)
	  	  	 {
	  		  	Set_LED(0, 0, 0, 0);
	  			    Set_LED(1, 0, 0, 0);
	  			    Set_LED(2, 0, 0, 0);
	  			    Set_LED(3, 0, 0, 0);
	  			    Set_LED(4, 255, 0, 0);
	  	  	 }


	    WS2812_Send();
	    Delay_ms(1);

	    /*

	  if (AxesData.Y<150)
	  	  {

	  	  }
	  if (AxesData.Y<-300)
	  	  {

	  	  }
	  if (AxesData.Y<-450)
	  	  {

	  	  }
	  if (AxesData.Y<-600)
	  	  {

	  	  }
	  if (AxesData.Y<-750)
	  	  {

	  	  }
	  */



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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
