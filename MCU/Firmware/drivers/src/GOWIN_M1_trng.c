/*
 ************************************************************************************************
 * @file      GOWIN_M1_trng.c
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the TRNG firmware library.
 ************************************************************************************************
 */
 
/* Includes ------------------------------------------------------------------*/
#include "GOWIN_M1_trng.h"


/* Definitions ---------------------------------------------------------------*/
void Set_Interrupt_Mask(uint8_t int_mask)
{
	TRNG->RNG_IMR = int_mask;
}

RNG_ISR_STATE Get_Int_State(void)
{
	RNG_ISR_STATE return_value = VN_ERR;
	
	if(TRNG->RNG_ISR == 0x08)
	{
		return_value = VN_ERR;
	}
	else if(TRNG->RNG_ISR == 0x04)
	{
		return_value = CRNGT_ERR;
	}
	else if(TRNG->RNG_ISR == 0x02)
	{
		return_value = AUTOCORR_ERR;
	}
	else if(TRNG->RNG_ISR == 0x01)
	{
		return_value = EHR_VALID;
	}
	
	return return_value;
}

void Clear_Int(RNG_ISR_STATE isr_state)
{
	if(isr_state == VN_ERR)
	{
		TRNG->RNG_ICR = 0x08;
	}
	else if(isr_state == CRNGT_ERR)
	{
		TRNG->RNG_ICR = 0x04;
	}
	else if(isr_state == AUTOCORR_ERR)
	{
		TRNG->RNG_ICR = 0x02;
	}
	else if(isr_state == EHR_VALID)
	{
		TRNG->RNG_ICR = 0x01;
	}
}

void Set_Config(RND_SRC_SEL_STATE src_state)
{
	if(src_state == SHORTEST_INVERT_LENGTH)
	{
		TRNG->TRNG_CONFIG = 0x00;
	}
	else if(src_state == SHORT_INVERT_LENGTH)
	{
		TRNG->TRNG_CONFIG = 0x01;
	}
	else if(src_state == LONG_INVERT_LENGTH)
	{
		TRNG->TRNG_CONFIG = 0x02;
	}
	else if(src_state == LONGEST_INVERT_LENGTH)
	{
		TRNG->TRNG_CONFIG = 0x03;
	}
}

void Get_EHR_Data(uint32_t * point)
{
	*point= TRNG->EHR_DATA0;
	*(point+1)= TRNG->EHR_DATA1;
	*(point+2)= TRNG->EHR_DATA2;
	*(point+3)= TRNG->EHR_DATA3;
	*(point+4)= TRNG->EHR_DATA4;
	*(point+5)= TRNG->EHR_DATA5;
}

void Set_Random_Source_Enable(void)
{
	TRNG->RND_SOURCE_ENABLE = 0x01;
}

void Clr_Random_Source_Enable()
{
	TRNG->RND_SOURCE_ENABLE = 0x00; 
}

void Set_Sample_Count(uint32_t sample_count)
{
	TRNG->SAMPLE_CNT1 = sample_count;
}

void Trng_SW_Reset(void)
{
	TRNG->TRNG_SW_RESET = 0x01;
}

TRNG_STATE Get_TRNG_State()
{
	TRNG_STATE return_value = NOTBUSY;
	
	if(TRNG->TRNG_BUSY == 0x01)
	{
		return_value = BUSY;
	}
	else if(TRNG->TRNG_BUSY == 0x00)
	{
		return_value = NOTBUSY;
	}
	
	return return_value;
}

void Reset_Bit_Count(void)
{
	if(TRNG->RND_SOURCE_ENABLE == 0x00)
	{
		TRNG->RST_BIT_COUNT = 0xFF;
	}
}

void Get_BIT_Counter(uint32_t *point)
{
	*(point+0) = TRNG->RNG_BIST_CNTR0;
	*(point+1) = TRNG->RNG_BIST_CNTR1;
	*(point+2) = TRNG->RNG_BIST_CNTR2;
}

TRNG_TEST_PASS Set_Debug_Control(void)
{
	TRNG_TEST_PASS return_value = TRNG_AUTOCORR_BYPASS;
	
	if(TRNG->TRNG_DEBUG_CONTROL == 0x08)
	{
		return_value = TRNG_AUTOCORR_BYPASS;
	}
	else if(TRNG->TRNG_DEBUG_CONTROL == 0x04)
	{
		return_value = TRNG_CRNGT_BYPASS;
	}
	else if(TRNG->TRNG_DEBUG_CONTROL == 0x02)
	{
		return_value = VNC_BYPASS;
	}
	
	return return_value;
}

uint32_t Fail_Start_State_times(void)
{
	return TRNG->AUTOCORR_STATISTIC;
}

void Clr_Fail_Start_State_register(void)
{
	TRNG->AUTOCORR_STATISTIC = 0xFF;
}

void Init_TRNG(void)
{
	Set_Interrupt_Mask(0x0E);
	Set_Config(SHORT_INVERT_LENGTH);
	Set_Random_Source_Enable();
	Set_Sample_Count(1000);
}
