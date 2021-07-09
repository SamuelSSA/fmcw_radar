#include "radar.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "uart_wrapper.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#define RECURRENCES 1
#define SAMPLES 256
#define HALF_SAMPLES (SAMPLES/2)

#define NUM_TAPS 32
static float32_t firStateF32[HALF_SAMPLES + NUM_TAPS - 1];

#define ADCCONVERTEDVALUES_BUFFER_SIZE 256 // it must can be divided by two

typedef enum
{
	NONE,
	INITIALIZING,
	RUNNING
}RADAR_STATE;


ALIGN_32BYTES (__IO uint16_t   g_video[ADCCONVERTEDVALUES_BUFFER_SIZE]);
typedef uint16_t CHIRP_DATA[sizeof(g_video)/sizeof(g_video[0])];
CHIRP_DATA g_chirp_serie[RECURRENCES];

uint16_t g_raw_adc_data[RECURRENCES][SAMPLES];

float raised_cos_lpf_coeff[] =
{
	 0.009749130994775397,
	 0.012923502275958659,
	 0.012575122349232363,
	 0.007647965989114582,
	-0.001641307946253627,
	-0.013506345892185998,
	-0.024724709654456022,
	-0.031241711278755802,
	-0.029141032673122441,
	-0.015749817863407338,
	 0.009425401808155646,
	 0.044220730428656756,
	 0.083979430402059588,
	 0.122375092779745495,
	 0.152724234547427618,
	 0.169472020661696643,
	 0.169472020661696643,
	 0.152724234547427618,
	 0.122375092779745495,
	 0.083979430402059588,
	 0.044220730428656756,
	 0.009425401808155646,
	-0.015749817863407338,
	-0.029141032673122441,
	-0.031241711278755802,
	-0.024724709654456022,
	-0.013506345892185998,
	-0.001641307946253627,
	 0.007647965989114582,
	 0.012575122349232363,
	 0.012923502275958659,
	 0.009749130994775397
};

enum
{
	UP_CHIRP,
	DOWN_CHIRP
};

uint16_t g_chirp_raw_data[2][RECURRENCES][HALF_SAMPLES];

typedef struct
{
	char header[5];
	uint8_t size;
	uint16_t video[sizeof(g_video)/sizeof(g_video[0])];
	char  end_data_marker[4];
}RADAR_DATA;

static uint8_t g_chirp_index = 0;
RADAR_DATA g_data_packet = {.size = sizeof(g_video), .header = "init", .end_data_marker = "end"};

char data[] = {"end"};

static void init_timers();
static void init_dac();
static void init_adc();
void serialize_data();

void radar_routine()
{
	static RADAR_STATE state = INITIALIZING;

	switch(state)
	{
		case INITIALIZING:

		break;

		case RUNNING:

		break;

		default:
		break;
	}

	float up_filtered_data[HALF_SAMPLES];
	float down_filtered_data[HALF_SAMPLES];
	if(g_chirp_index == (RECURRENCES - 1))
	{
		parse_chirp_data();
		dsp(up_filtered_data, down_filtered_data);
	}
	serialize_data(up_filtered_data,down_filtered_data);
}


void init_hardware()
{
	init_dac();
	init_adc();
	init_timers();
}

static void init_timers()
{
	if(HAL_TIM_Base_Start(&htim3) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_TIM_Base_Start(&htim4) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_TIM_Base_Start(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
}

static void init_dac()
{
	if(HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2,(uint32_t *)g_video, sizeof(g_video),DAC_ALIGN_12B_R) != HAL_OK)
	{
		Error_Handler();
	}
}

static void init_adc()
{
	if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*) g_video, sizeof(g_video)) != HAL_OK)
	{
		Error_Handler();
	}

/*
	if(HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_ADC_Start_DMA(&hadc3, (uint32_t*) adc_buffer, sizeof(adc_buffer)) != HAL_OK)
	{
		Error_Handler();
	}
	*/
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	memcpy((void *)g_raw_adc_data[g_chirp_index], g_video, ADCCONVERTEDVALUES_BUFFER_SIZE*2);

	if(g_chirp_index < (RECURRENCES - 1))
		g_chirp_index++;
	else
		g_chirp_index = 0;
}

void serialize_data(float *filtered_up, float *filtered_down)
{
	static uint32_t counter = 1;

	if (counter == 100000)
	{
		//memcpy((void *)&g_data_packet.video, g_video, sizeof(g_data_packet.video));
		for(int i =0; i < HALF_SAMPLES; i++)
		{
			g_data_packet.video[i] = (uint16_t) filtered_up[i];
			g_data_packet.video[i + HALF_SAMPLES] = (uint16_t) filtered_down[i];
		}

		if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)&g_data_packet.header, sizeof(RADAR_DATA)) != HAL_OK )
		{
			Error_Handler();
		}
		counter = 1;
	}
	counter++;
}
/*
float averageSignal(uint16_t *data, size_t size)
{

}
*/

void parse_chirp_data()
{
	for(int i = 0; i < RECURRENCES; i++)
	{
	    memcpy(g_chirp_raw_data[UP_CHIRP][i], g_raw_adc_data[i], HALF_SAMPLES*sizeof(uint16_t));
		memcpy(g_chirp_raw_data[DOWN_CHIRP][i], (((uint8_t*) g_raw_adc_data[i]) + HALF_SAMPLES*sizeof(uint16_t)), HALF_SAMPLES*sizeof(uint16_t));
	}
}

void dsp(float *up, float *down)
{
	float f_up_chirp[RECURRENCES][HALF_SAMPLES];
	float f_down_chirp[RECURRENCES][HALF_SAMPLES];
	float up_filter_result[RECURRENCES][HALF_SAMPLES];
	float down_filter_result[RECURRENCES][HALF_SAMPLES];

	float fft_result[RECURRENCES][2][HALF_SAMPLES];
	float fft_mag[RECURRENCES][2][HALF_SAMPLES];

	arm_status status;
	arm_rfft_fast_instance_f32 fft_instance;
	uint8_t ifftFlag = 0;

	arm_fir_instance_f32 arm_fir_instance;

	/* Call FIR init function to initialize the instance structure. */
	arm_fir_init_f32(&arm_fir_instance, NUM_TAPS, (float32_t *)&raised_cos_lpf_coeff[0], &firStateF32[0], HALF_SAMPLES);
	status = ARM_MATH_SUCCESS;
	status = arm_rfft_fast_init_f32(&fft_instance, HALF_SAMPLES);

	for(int j = 0; j < RECURRENCES; j++)
	{
		for(int k = 0; k < HALF_SAMPLES; k++)
		{
			f_up_chirp[j][k] 	= (float) g_chirp_raw_data[UP_CHIRP][j][k];
			f_down_chirp[j][k]  = (float) g_chirp_raw_data[DOWN_CHIRP][j][k];
		}

		arm_fir_f32(&arm_fir_instance, f_up_chirp[j], up, HALF_SAMPLES);
		arm_fir_f32(&arm_fir_instance, f_down_chirp[j], down, HALF_SAMPLES);

		arm_fir_f32(&arm_fir_instance, f_up_chirp[j], &up_filter_result[j][UP_CHIRP], HALF_SAMPLES);
		arm_fir_f32(&arm_fir_instance, f_down_chirp[j], &down_filter_result[j][DOWN_CHIRP], HALF_SAMPLES);

		arm_rfft_fast_f32(&fft_instance, up_filter_result[j], fft_result[j][UP_CHIRP], ifftFlag);
		arm_rfft_fast_f32(&fft_instance, down_filter_result[j], fft_result[j][DOWN_CHIRP], ifftFlag);

		arm_cmplx_mag_squared_f32(fft_result[j][UP_CHIRP],fft_mag[j][UP_CHIRP],HALF_SAMPLES);
		arm_cmplx_mag_squared_f32(fft_result[j][DOWN_CHIRP],fft_mag[j][DOWN_CHIRP],HALF_SAMPLES);
	}
}
