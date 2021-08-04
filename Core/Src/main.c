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
#include <stdio.h>
#include "math.h"
#include "arm_math.h"
#include "svpwm.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float sinePeriodFiltered = 0;
uint32_t hallCounter = 0;
uint8_t hallCounterChanged = 0;
float theta = 0.0f;  // in rad
float theta_hall = 0.0f;
float gfOmega = 0.0f;  // in rad/s/pi
q31_t gqwOmegaDt = 0;
q31_t gqwTheta = 0;		// electrical angle [-1, 1) is mapped to [-pi, pi) in radian or [-180, 180) in degrees
q31_t gqwThetaHall = 0;
uint8_t hall_pattern = 0x00;
//float theta_at_edge[] = { 0, 2.0 * M_PI / 3.0, M_PI / 3.0, -2.0 * M_PI / 3.0, -M_PI / 3.0, M_PI };
q31_t qwThetaAtEdge[] = { Q31(0.0), Q31(0.0), Q31(2 / 3.0), Q31(1 / 3.0), Q31(-2 / 3.0), Q31(-1 / 3.0), Q31(-1.0) };
uint8_t gaubHallPatternSector[] = {0, 0, 2, 1, 4, 5, 3};
int8_t gabHallDirectionTable[6][6] = {
		{0, 1, 0, 0, 0, -1},
		{-1, 0, 1, 0, 0, 0},
		{0, -1, 0, 1, 0, 0},
		{0, 0, -1, 0, 1, 0},
		{0, 0, 0, -1, 0, 1},
		{1, 0, 0, 0, -1, 0},
};
uint8_t gubHallPatternPrev = 0x00;
uint8_t gubHallPatternCurrent = 0x00;
float k_pll = 0.1;

float adv = M_PI;

#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)   4)
uint16_t aADC1ConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];


q31_t gaqwPhaseCurrent[3] = {0, 0, 0};
q31_t gaqwCurrentAB[2] = {0, 0};
q31_t gaqwCurrentDQ[2] = {0, 0};

uint16_t uhVDDA = 0;
uint16_t aAdcOffset[3] = {0,0,0};

char text[256];

/* Variable to report status of DMA transfer of ADC group regular conversions */
/*  0: DMA transfer is not completed                                          */
/*  1: DMA transfer is completed                                              */
/*  2: DMA transfer has not been started yet (initial state)                  */
volatile uint8_t ubAdc1DmaTransferCplt = 0; /* Variable set into DMA interruption callback */

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == GPIO_PIN_15) { // Hall A
//		if(GPIOA->IDR & GPIO_IDR_15 != GPIO_PIN_RESET){
//			// Pseudo-Sector 0
//			uint32_t now = HAL_GetTick();
//			sinePeriodFiltered = 1.0f / (now - counterU);
//		}
//	}
//}

void prints(const char * const str){
	HAL_UART_Transmit(&huart2, str, strlen(str), 10);
}

uint16_t ADC_Self_Cal_Channel(uint32_t channel) {
	const uint16_t ubPowNbConvSelfCal = 8;
	uint16_t ubConvIdx = 0;
	uint32_t uwAdcAccum = 0;

//	// configure channel
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);
    LL_ADC_SetChannelSamplingTime(ADC1, channel, LL_ADC_SAMPLINGTIME_601CYCLES_5);

	ubAdc1DmaTransferCplt = 0;
	for(ubConvIdx = 0; ubConvIdx < (1u << ubPowNbConvSelfCal); ubConvIdx++)
	{
		LL_ADC_REG_StartConversion(ADC1);
		while(ubAdc1DmaTransferCplt == 0) ;
		ubAdc1DmaTransferCplt = 0;
		uwAdcAccum += aADC1ConvertedData[0];
	}
	return uwAdcAccum >> ubPowNbConvSelfCal;
}

void ADC_Self_Cal(void) {
	// disable ADC first to configure
	if(LL_ADC_IsEnabled(ADC1)) {
		LL_ADC_Disable(ADC1);
		while(LL_ADC_IsEnabled(ADC1) != 0) ;
	}

	// configure ADC
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV4);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);

	LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);			// this line requires ADC disabled
    //LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    //LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);

    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

    // configure DMA
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
			LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
			(uint32_t) &aADC1ConvertedData, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 1);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
		; // wait for calibration completion

	LL_ADC_Enable(ADC1);
	while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
		; // wait until adc is ready

	HAL_Delay(100);

	// Calibrate VDDA
	uhVDDA = __LL_ADC_CALC_VREFANALOG_VOLTAGE(ADC_Self_Cal_Channel(LL_ADC_CHANNEL_VREFINT), LL_ADC_RESOLUTION_12B);
	//uhVDDA = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, ADC_Self_Cal_Channel(LL_ADC_CHANNEL_VREFINT), LL_ADC_RESOLUTION_12B);
	//uhVDDA = ADC_Self_Cal_Channel(LL_ADC_CHANNEL_VREFINT);

	// Calibrate I_u
    aAdcOffset[0] = ADC_Self_Cal_Channel(LL_ADC_CHANNEL_1);

	// Calibrate I_u
    aAdcOffset[1] = ADC_Self_Cal_Channel(LL_ADC_CHANNEL_7);

	// Calibrate I_u
    aAdcOffset[2] = ADC_Self_Cal_Channel(LL_ADC_CHANNEL_6);
}

void Configure_ADC_Normal(void) {
	// disable ADC first to configure
	if(LL_ADC_IsEnabled(ADC1)) {
		LL_ADC_Disable(ADC1);
		while(LL_ADC_IsEnabled(ADC1) != 0) ;
	}

    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV1);
	LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_LEFT);
    LL_ADC_SetOffset(ADC1, LL_ADC_OFFSET_1, LL_ADC_CHANNEL_1, aAdcOffset[0]);
    LL_ADC_SetOffset(ADC1, LL_ADC_OFFSET_2, LL_ADC_CHANNEL_7, aAdcOffset[1]);
    LL_ADC_SetOffset(ADC1, LL_ADC_OFFSET_3, LL_ADC_CHANNEL_6, aAdcOffset[2]);

    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM6_TRGO);
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

    // configure DMA
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
			LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
			(uint32_t) &aADC1ConvertedData, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_CONVERTED_DATA_BUFFER_SIZE);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	// Enable ADC
	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
		; // wait for calibration completion

	LL_ADC_Enable(ADC1);
	while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
		; // wait until adc is ready

    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS);

    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_TEMPSENSOR);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_8);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_2);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_12);

    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_181CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_19CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_19CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_181CYCLES_5);

    // for injected channels
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_181CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_19CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_19CYCLES_5);
}

void Configure_DAC(void)
{
	/*## Configuration of GPIO used by DAC channels ############################*/

	/* Enable GPIO Clock */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	/* Configure GPIO in analog mode to be used as DAC output */
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);

	/*## Configuration of DAC ##################################################*/

	/* Enable DAC clock */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);

	/* Select trigger source */
	LL_DAC_SetTriggerSource(DAC1, LL_DAC_CHANNEL_1, LL_DAC_TRIG_SOFTWARE);

	/* Set the output for the selected DAC channel */
	LL_DAC_SetOutputBuffer(DAC1, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_BUFFER_ENABLE);

	/* Disable DAC channel DMA request */
	LL_DAC_DisableDMAReq(DAC1, LL_DAC_CHANNEL_1);

	/* Set the data to be loaded in the data holding register */
	LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 0x000);

	/* Enable interruption DAC channel1 underrun */
	//LL_DAC_EnableIT_DMAUDR1(DAC1);
}

void Activate_DAC(void)
{
  __IO uint32_t wait_loop_index = 0;

  /* Enable DAC channel */
  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);

  /* Delay for DAC channel voltage settling time from DAC channel startup.    */
  /* Compute number of CPU cycles to wait for, from delay in us.              */
  /* Note: Variable divided by 2 to compensate partially                      */
  /*       CPU processing cycles (depends on compilation optimization).       */
  /* Note: If system core clock frequency is below 200kHz, wait time          */
  /*       is only a few CPU processing cycles.                               */
  wait_loop_index = ((LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		hallCounter = TIM2->CCR1;
		hallCounterChanged = 1;
		gubHallPatternPrev = gubHallPatternCurrent;
		hall_pattern = ((GPIOA->IDR & GPIO_IDR_15) >> 13)
				| ((GPIOB->IDR & GPIO_IDR_3) >> 2)
				| ((GPIOB->IDR & GPIO_IDR_10) >> 10);
		gubHallPatternCurrent = gaubHallPatternSector[hall_pattern];
		gfOmega = 1.0f / (3 * hallCounter);
		float fOmegaDt = 1e-3 / 3.0f * 72e6 / hallCounter;

		gfOmega *= gabHallDirectionTable[gubHallPatternPrev][gubHallPatternCurrent];
		fOmegaDt *= gabHallDirectionTable[gubHallPatternPrev][gubHallPatternCurrent];

		arm_float_to_q31(&fOmegaDt, &gqwOmegaDt, 1);
		gqwThetaHall = qwThetaAtEdge[hall_pattern];
		//gqwTheta = gqwThetaHall;
	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		// do nothing
	} else if (htim->Instance == TIM6) {
		GPIOB->BSRR = GPIO_BSRR_BS_2;
		//q31_t qwOmegaDt = 0;
		//float fOmegaDt = omega / M_PI * 1e-3;
		//arm_float_to_q31(&fOmegaDt, &qwOmegaDt, 1);
		//gqwTheta += gqwOmegaDt;
		//LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (gqwTheta >> 20) ^ 0x800);
		//q31_t qwThetaRT = gqwThetaHall + Q31(gfOmega * TIM2->CNT);
		gqwTheta += Q31(0.03f);
		q31_t qwThetaRT = gqwTheta;
		//LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (qwThetaRT >> 20) ^ 0x800);


		q31_t qwAdv =  Q31(0.7);
		q31_t qwSines[3];
		q31_t dummy;

		arm_sin_cos_q31(qwThetaRT + qwAdv, &qwSines[0], &dummy);
		arm_sin_cos_q31(qwThetaRT + Q31(2.0f / 3.0f) + qwAdv, &qwSines[1], &dummy);
		arm_sin_cos_q31(qwThetaRT - Q31(2.0f / 3.0f) + qwAdv, &qwSines[2], &dummy);

		//UVW uvw;
		//SVPWM_Calc(gqwTheta, Q31(0.4f), &uvw);

		//LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (qwSines[0] >> 20) ^ 0x800);
		//LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (uvw.V >> 20) ^ 0x800);

		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_RESET) {
	//			TIM1->CCR1 = 1800.0 * sin(theta + adv);
	//			TIM1->CCR2 = 1800.0 * sin(theta + 2.0 * M_PI / 3.0 + adv);
	//			TIM1->CCR3 = 1800.0 * sin(theta - 2.0 * M_PI / 3.0 + adv);
			TIM1->CCR1 = 500.0 + (int32_t)qwSines[0] * (100.0/0x80000000);
			TIM1->CCR2 = 500.0 + (int32_t)qwSines[1] * (100.0/0x80000000);
			TIM1->CCR3 = 500.0 + (int32_t)qwSines[2] * (100.0/0x80000000);
			//TIM1->CCR1 = (int32_t)(uvw.U) * (1000.0/0x80000000);
			//TIM1->CCR2 = (int32_t)(uvw.V) * (1000.0/0x80000000);
			//TIM1->CCR3 = (int32_t)(uvw.W) * (1000.0/0x80000000);
		}
		GPIOB->BSRR = GPIO_BSRR_BR_2;
	}
}

void Adc1DmaTransferComplete_Callback(void) {
	ubAdc1DmaTransferCplt = 1;
}

void Adc1InjConvComplete_Callback(void) {
//	qwPhaseCurrent[0] = (uint32_t)(ADC1->JDR1) << 16;
//	qwPhaseCurrent[1] = ADC1->JDR2 << 16;
//	qwPhaseCurrent[2] = ADC1->JDR3 << 16;

	gaqwPhaseCurrent[0] = ADC1->JDR1 << 16;
	gaqwPhaseCurrent[1] = ADC1->JDR2 << 16;
	//LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, ((int32_t)(gaqwPhaseCurrent[0]) >> 20) ^ 0x800);

	//q31_t qwThetaRT = gqwThetaHall + Q31(gfOmega * TIM2->CNT);

	q31_t qwSin, qwCos;
	arm_sin_cos_q31(gqwTheta, &qwSin, &qwCos);
	arm_clarke_q31(gaqwPhaseCurrent[0], gaqwPhaseCurrent[1], &gaqwCurrentAB[0], &gaqwCurrentAB[1]);
	arm_park_q31(gaqwCurrentAB[1], gaqwCurrentAB[0], &gaqwCurrentDQ[0], &gaqwCurrentDQ[1], qwSin, qwCos);

	//LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, ((int32_t)(gaqwCurrentAB[1]) >> 20) ^ 0x800);
	LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, ((int32_t)(gaqwCurrentDQ[1]) >> 20) ^ 0x800);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */

	Configure_DAC();
	Activate_DAC();

	prints("Starting Self-Cal...");

	ADC_Self_Cal();

	sprintf(&text, "done. VDDA: %04d; offsets: %04d, %04d, %04d.\r\n", uhVDDA, (short)aAdcOffset[0], (short)aAdcOffset[1], (short)aAdcOffset[2]);
	prints(&text);

	HAL_TIMEx_HallSensor_Start_IT(&htim2);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	TIM1->CCR4 = 995;

	HAL_TIM_Base_Start_IT(&htim6);


	Configure_ADC_Normal();
	//LL_ADC_REG_StartConversion(ADC1);
	LL_ADC_INJ_StartConversion(ADC1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		uint32_t startDelay = 5;
		uint32_t startDuty = 3000;

		//LL_ADC_REG_StartConversion(ADC1);

		//while(LL_ADC_REG_IsConversionOngoing(ADC1) != 0) ; // wait for conversion

		//while (ubAdc1DmaTransferCplt != 1)
		//	;

		//uint16_t result = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300,
		//		aADC1ConvertedData[0], LL_ADC_RESOLUTION_12B);

		ubAdc1DmaTransferCplt = 0;

		//if (hallCounterChanged != 0) {
		//	uint32_t h = TIM2->CCR1;

		//i = sprintf(&text, "%04d, %04d, %04d\r\n", i_u, i_v, i_w);
		//	sprintf(&text, "%d\r\n", hall_pattern);
		//HAL_UART_Transmit(&huart2, text, i, 10);
		//float fPhaseCurrent[3] = {0.0f, 0.0f, 0.0f};
		//arm_q31_to_float(gaqwPhaseCurrent, fPhaseCurrent, 3);
		sprintf(&text, "%07.4f, %07.3f, %07.3f, %07.3f\r\n", Q31_TO_F(gqwOmegaDt),
				Q31_TO_F(gaqwPhaseCurrent[0])*19.83471074380165,
				Q31_TO_F(gaqwPhaseCurrent[1])*19.83471074380165,
				Q31_TO_F(gaqwPhaseCurrent[2])*19.83471074380165);
		prints(&text);
		//	sprintf(&text, "%d\r\n", hall_pattern);
		//prints(&text);
		//	hallCounterChanged = 0;
		//}
		HAL_Delay(1);

		while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
			TIM1->CCR1 = startDuty;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			HAL_Delay(startDelay);
			sprintf(&text, "A->B: %1d\r\n", hall_pattern);
			HAL_UART_Transmit(&huart2, text, 10, 10);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_Delay(startDelay);
			sprintf(&text, "A->C: %1d\r\n", hall_pattern);
			HAL_UART_Transmit(&huart2, text, 10, 10);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			TIM1->CCR1 = 0;
			TIM1->CCR2 = startDuty;
			TIM1->CCR3 = 0;
			HAL_Delay(startDelay);
			sprintf(&text, "B->C: %1d\r\n", hall_pattern);
			HAL_UART_Transmit(&huart2, text, 10, 10);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_Delay(startDelay);
			sprintf(&text, "B->A: %1d\r\n", hall_pattern);
			HAL_UART_Transmit(&huart2, text, 10, 10);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = startDuty;
			HAL_Delay(startDelay);
			sprintf(&text, "C->A: %1d\r\n", hall_pattern);
			HAL_UART_Transmit(&huart2, text, 10, 10);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_Delay(startDelay);
			sprintf(&text, "C->B: %1d\r\n", hall_pattern);
			HAL_UART_Transmit(&huart2, text, 10, 10);
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
	}
	LL_RCC_HSE_EnableBypass();
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1) {

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1) {

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

	}
	LL_SetSystemCoreClock(72000000);

	/* Update the time base */
	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
		Error_Handler();
	}
	LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PCLK2);
	LL_RCC_SetADCClockSource(LL_RCC_ADC1_CLKSRC_PLL_DIV_1);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	NVIC_SetPriority(ADC1_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(ADC1_IRQn);

	/* USER CODE END ADC1_Init 0 */

	LL_ADC_InitTypeDef ADC_InitStruct = { 0 };
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = { 0 };
	LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC1);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	/**ADC1 GPIO Configuration
	 PC0   ------> ADC1_IN6
	 PC1   ------> ADC1_IN7
	 PC2   ------> ADC1_IN8
	 PC3   ------> ADC1_IN9
	 PA0   ------> ADC1_IN1
	 PA1   ------> ADC1_IN2
	 PA7   ------> ADC1_IN15
	 PB0   ------> ADC1_IN11
	 PB1   ------> ADC1_IN12
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2
			| LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* ADC1 DMA Init */

	/* ADC1 Init */
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
	LL_ADC_Init(ADC1, &ADC_InitStruct);
	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM6_TRGO;
	ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
	ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
	ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
	LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

	/* Enable ADC internal voltage regulator */
	LL_ADC_EnableInternalRegulator(ADC1);
	/* Delay for ADC internal voltage regulator stabilization. */
	/* Compute number of CPU cycles to wait for, from delay in us. */
	/* Note: Variable divided by 2 to compensate partially */
	/* CPU processing cycles (depends on compilation optimization). */
	/* Note: If system core clock frequency is below 200kHz, wait time */
	/* is only a few CPU processing cycles. */
	uint32_t wait_loop_index;
	wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US
			* (SystemCoreClock / (100000 * 2))) / 10);
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}
	/** Configure injected channel ADC_JSQR register
	 */
	LL_ADC_INJ_ConfigQueueContext(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2,
			LL_ADC_INJ_TRIG_EXT_RISING, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
			LL_ADC_CHANNEL_7, LL_ADC_CHANNEL_6, 0x0000, 0x0000);
	LL_ADC_INJ_SetTrigAuto(ADC1, LL_ADC_INJ_TRIG_INDEPENDENT);
	LL_ADC_INJ_SetSequencerDiscont(ADC1, LL_ADC_INJ_SEQ_DISCONT_DISABLE);
	/** Configure Regular Channel
	 */
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1,
			LL_ADC_CHANNEL_VREFINT);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT,
			LL_ADC_SAMPLINGTIME_181CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VREFINT,
			LL_ADC_SINGLE_ENDED);
	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
			LL_ADC_PATH_INTERNAL_VREFINT);
	/** Configure Injected Channel
	 */
	LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_2CONTEXTS_LAST_ACTIVE);
	LL_ADC_DisableIT_JEOC(ADC1);
	LL_ADC_DisableIT_JEOS(ADC1);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1,
			LL_ADC_SAMPLINGTIME_19CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
	LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
	/** Configure Injected Channel
	 */
	LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_2CONTEXTS_LAST_ACTIVE);
	LL_ADC_DisableIT_JEOC(ADC1);
	LL_ADC_DisableIT_JEOS(ADC1);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7,
			LL_ADC_SAMPLINGTIME_19CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SINGLE_ENDED);
	LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
	/** Configure Injected Channel
	 */
	LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_2CONTEXTS_LAST_ACTIVE);
	LL_ADC_DisableIT_JEOC(ADC1);
	LL_ADC_DisableIT_JEOS(ADC1);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6,
			LL_ADC_SAMPLINGTIME_19CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SINGLE_ENDED);
	LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
	/** Configure Regular Channel
	 */
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2,
			LL_ADC_CHANNEL_TEMPSENSOR);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR,
			LL_ADC_SAMPLINGTIME_181CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_TEMPSENSOR,
			LL_ADC_SINGLE_ENDED);
	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
			LL_ADC_PATH_INTERNAL_TEMPSENSOR);
	/** Configure Regular Channel
	 */
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_2);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2,
			LL_ADC_SAMPLINGTIME_19CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);
	/** Configure Regular Channel
	 */
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_8);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8,
			LL_ADC_SAMPLINGTIME_19CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);
	/* USER CODE BEGIN ADC1_Init 2 */

	LL_ADC_DisableIT_JEOC(ADC1);
	LL_ADC_EnableIT_JEOS(ADC1);

	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_601CYCLES_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_181CYCLES_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_181CYCLES_5);


    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_19CYCLES_5);
//
//  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_12);
//  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_19CYCLES_5);
//  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SINGLE_ENDED);
//
//  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_TEMPSENSOR);
//  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_181CYCLES_5);
//  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SINGLE_ENDED);
//
//  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_VREFINT);
//  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_181CYCLES_5);
//  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SINGLE_ENDED);
	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim1.Init.Period = 1000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_HallSensor_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xffffffff;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.Commutation_Delay = 0;
	if (HAL_TIMEx_HallSensor_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 72 - 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 100 - 1;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* Init with LL driver */
	/* DMA controller clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	NVIC_SetPriority(DMA1_Channel1_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED_RED_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_RED_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LED_RED_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PC10 PC11 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 2);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
