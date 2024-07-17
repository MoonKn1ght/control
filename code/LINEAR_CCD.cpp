#include "LINEAR_CCD.h"

uint32_t gray_scale[128];
uint8_t CCD_ADC_flag = 0;
uint8_t CCD_point_num = 0;
//uint32_t adc_result;
void CCD_scale_get()
{
	HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
//	HAL_ADC_Start(&hadc3);
//	HAL_ADC_PollForConversion(&hadc3, 100);

	CCD_ADC_flag = 1;
//  adc_result = HAL_ADC_GetValue(&hadc3);
//	HAL_ADC_Stop(&hadc3);

}
void CCD_init()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim4);
	//HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
	TIM4->CCR1 = 84;
	HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
}

void CCD_Handler()
{
	if(CCD_ADC_flag == 0)
		return;
	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, 100);
	gray_scale[CCD_point_num] = HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);
	if(++CCD_point_num == 128)
	{
		CCD_point_num = 0;
		CCD_ADC_flag = 0;
		HAL_ADC_Stop(&hadc3);

	}
}
