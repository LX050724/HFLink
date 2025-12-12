/*
 ****************************************************************************************************
 * @file      GOWIN_M1_dualtimer.c
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the DualTimer firmware library.
 ****************************************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "GOWIN_M1_dualtimer.h"


/* Definitions ---------------------------------------------------------------*/
void ENABLE_Dtimer_function(TIMER_ID timer_id)
{
	if(timer_id == TIMER1_ID)
	{
		DUALTIMER->TIMER1CONTROL = DUALTIMER->TIMER1CONTROL |shift_set_H(TIMER_ENABLE_BIT);
		return;
	}
	
	if(timer_id == TIMER2_ID)
	{
		DUALTIMER->TIMER2CONTROL = DUALTIMER->TIMER2CONTROL |shift_set_H(TIMER_ENABLE_BIT);
		return;
	}
	
}

void TIMER_SIZE_function(TIMER_ID timer_id,TIMER_SIZE timer_size)
{
	if(timer_id == TIMER1_ID)
	{
		if(timer_size == BIT16)
		{
			DUALTIMER ->TIMER1CONTROL = DUALTIMER ->TIMER1CONTROL & shift_set_L(TIMER_SIZE_BIT);
		}
		
		if(timer_size == BIT32)
		{
			DUALTIMER ->TIMER1CONTROL = DUALTIMER ->TIMER1CONTROL | shift_set_H(TIMER_SIZE_BIT);
		}
	}
	
	if(timer_id == TIMER2_ID)
	{
		if(timer_size == BIT16)
		{
			DUALTIMER ->TIMER2CONTROL = DUALTIMER ->TIMER2CONTROL & shift_set_L(TIMER_SIZE_BIT);
		}
		
		if(timer_size == BIT32)
		{
			DUALTIMER ->TIMER2CONTROL = DUALTIMER ->TIMER2CONTROL | shift_set_H(TIMER_SIZE_BIT);
		}
	}
}

void ENABLE_interrupt_Dtimer_function(TIMER_ID timer_id,INTR_STATE intr_state)
{
	if(timer_id == TIMER1_ID)
	{
		if(intr_state == OPEN_INTR)
		{
			DUALTIMER -> TIMER1CONTROL = DUALTIMER -> TIMER1CONTROL | shift_set_H(TIMER_INTR_ENABLE_BIT);
		}
		
		if(intr_state == CLOSE_INTR)
		{
			DUALTIMER -> TIMER1CONTROL = DUALTIMER -> TIMER1CONTROL & shift_set_L(TIMER_INTR_ENABLE_BIT);
		}
	}
	
	if(timer_id == TIMER2_ID)
	{
		if(intr_state == OPEN_INTR)
		{
			DUALTIMER -> TIMER2CONTROL = DUALTIMER -> TIMER2CONTROL | shift_set_H(TIMER_INTR_ENABLE_BIT);
		}
		
		if(intr_state == CLOSE_INTR)
		{
			DUALTIMER -> TIMER2CONTROL = DUALTIMER -> TIMER2CONTROL & shift_set_L(TIMER_INTR_ENABLE_BIT);
		}
	}
}

void INIT_NUM_load_function(TIMER_ID timer_id,uint32_t loadnum)
{
	if(timer_id == TIMER1_ID)
	{
		DUALTIMER ->TIMER1LOAD = loadnum;
		DUALTIMER ->TIMER1BGLOAD = loadnum;
	}
	
	if(timer_id == TIMER2_ID)
	{
		DUALTIMER ->TIMER2LOAD = loadnum;
		DUALTIMER ->TIMER2BGLOAD = loadnum;
	}
}

void Dtimer_PRE_function (TIMER_ID timer_id,TIMER_PRE timer_pre)
{
	if(timer_id == TIMER1_ID)
	{
		if(timer_pre == NON_PRE) 
		{
			DUALTIMER ->TIMER1CONTROL = DUALTIMER ->TIMER1CONTROL &(shift_set_L(TIMER_PRE_BIT_H)&shift_set_L(TIMER_PRE_BIT_L));
		}
		
		if(timer_pre == PRE_16 )
		{
			DUALTIMER ->TIMER1CONTROL = (DUALTIMER ->TIMER1CONTROL & shift_set_L(TIMER_PRE_BIT_H))|shift_set_H(TIMER_PRE_BIT_L);
		}
		
		if(timer_pre == PRE_256)
		{
			DUALTIMER ->TIMER1CONTROL = (DUALTIMER ->TIMER1CONTROL |shift_set_H(TIMER_PRE_BIT_H))&shift_set_L(TIMER_PRE_BIT_L);
		}
		
		return;
	}
	
	if(timer_id == TIMER2_ID)
	{
		if(timer_pre == NON_PRE) 
		{
			DUALTIMER ->TIMER2CONTROL = DUALTIMER ->TIMER2CONTROL &(shift_set_L(TIMER_PRE_BIT_H)&shift_set_L(TIMER_PRE_BIT_L));
		}
		
		if(timer_pre == PRE_16 )
		{
			DUALTIMER ->TIMER2CONTROL = (DUALTIMER ->TIMER2CONTROL & shift_set_L(TIMER_PRE_BIT_H))|shift_set_H(TIMER_PRE_BIT_L);
		}
		
		if(timer_pre == PRE_256)
		{
			DUALTIMER ->TIMER2CONTROL = (DUALTIMER ->TIMER2CONTROL |shift_set_H(TIMER_PRE_BIT_H))&shift_set_L(TIMER_PRE_BIT_L);
		}
		
		return;
	}
}

void Dtimer_MODE_function(TIMER_ID timer_id,TIMER_MODE timer_mode)
{
	if(timer_id == TIMER1_ID)
	{
		if(timer_mode == FRE_RUNNING)
		{
			DUALTIMER ->TIMER1CONTROL = DUALTIMER ->TIMER1CONTROL & (shift_set_L(TIMER_MODE_BIT_H)&shift_set_L(TIMER_MODE_BIT_L));
		}
		
		if(timer_mode == RETIODIC_MODE)
		{
			DUALTIMER ->TIMER1CONTROL = (DUALTIMER ->TIMER1CONTROL | shift_set_H(TIMER_MODE_BIT_H))&shift_set_L(TIMER_MODE_BIT_L);
		}
		
		if(timer_mode == ONE_SHOT_MODE)
		{
			DUALTIMER ->TIMER1CONTROL = (DUALTIMER ->TIMER1CONTROL & shift_set_L(TIMER_MODE_BIT_H))|shift_set_H(TIMER_MODE_BIT_L);
		}
	}
	
	if(timer_id == TIMER2_ID)
	{
		if(timer_mode == FRE_RUNNING)
		{
			DUALTIMER ->TIMER2CONTROL = DUALTIMER ->TIMER2CONTROL & (shift_set_L(TIMER_MODE_BIT_H)&shift_set_L(TIMER_MODE_BIT_L));
		}
		
		if(timer_mode == RETIODIC_MODE)
		{
			DUALTIMER ->TIMER2CONTROL = (DUALTIMER ->TIMER2CONTROL | shift_set_H(TIMER_MODE_BIT_H))&shift_set_L(TIMER_MODE_BIT_L);
		}
		
		if(timer_mode == ONE_SHOT_MODE)
		{
			DUALTIMER ->TIMER2CONTROL = (DUALTIMER ->TIMER2CONTROL & shift_set_L(TIMER_MODE_BIT_H))|shift_set_H(TIMER_MODE_BIT_L);
		}
	}
}

void Clear_DULATIMER_interrupt(TIMER_ID timer_id)
{
	if(timer_id == TIMER1_ID)
	{
		DUALTIMER ->TIMER1INTCLR =0xFF;
	}
		
	if(timer_id == TIMER2_ID)
	{
		DUALTIMER ->TIMER2INTCLR =0xFF;
	}
}

TIMER_ID Get_DULATIMER_interrupt_num()
{
	TIMER_ID return_value;
	
	if(DUALTIMER ->TIMER1MIS )
	{
		return_value = TIMER1_ID;
	}
	
	if(DUALTIMER ->TIMER2MIS)
	{
		return_value = TIMER2_ID;
	}
	
	return return_value;
}

void DUALTIMER2_Init()
{
	//INIT DUALTIMER 2
	INIT_NUM_load_function(TIMER2_ID,50000000);
	Dtimer_MODE_function(TIMER2_ID,RETIODIC_MODE);
	Dtimer_PRE_function(TIMER2_ID,NON_PRE);
	TIMER_SIZE_function(TIMER2_ID,BIT32);
	ENABLE_interrupt_Dtimer_function(TIMER2_ID,OPEN_INTR);
	ENABLE_Dtimer_function(TIMER2_ID);
}

void DUALTIMER1_Init()
{
	//INIT DUALTIMER 1
	INIT_NUM_load_function(TIMER1_ID,5000000);
	Dtimer_MODE_function(TIMER1_ID,RETIODIC_MODE);
	Dtimer_PRE_function(TIMER1_ID,NON_PRE);
	TIMER_SIZE_function(TIMER1_ID,BIT32);
	ENABLE_interrupt_Dtimer_function(TIMER1_ID,OPEN_INTR);
	ENABLE_Dtimer_function(TIMER1_ID);
}
