/*
 ****************************************************************************************************
 * @file      GOWIN_M1_misc.c
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the miscellaneous firmware library.
 ****************************************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "GOWIN_M1_misc.h"


/* Definitions ---------------------------------------------------------------*/
/** @addtogroup GOWIN_M1_StdPeriph_Driver
  * @{
  */

/** @defgroup MISC
  * @brief MISC driver modules
  * @{
  */

/** @defgroup MISC_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup MISC_Private_Macros
  * @{
  */

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

/**
  * @}
  */

/** @defgroup MISC_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup MISC_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup MISC_Private_Functions
  * @{
  */
	
/**
  * @param  none
  * @return none
	* @brief  System reset
  */
void nvic_system_reset(void)
{
  NVIC_SystemReset();
}

/**
  * @param  irqn (IRQn_Type number)
  * @param  preempt_priority: preemptive priority value (starting from 0)
  * @param  sub_priority: subpriority value (starting from 0)
  * @return none
  * @brief  Enable nvic irq
  */
void nvic_irq_enable(IRQn_Type irqn, uint32_t preempt_priority, uint32_t sub_priority)
{
  uint32_t temp_priority = 0;

  /* Encode priority */
  temp_priority = NVIC_EncodePriority(NVIC_GetPriorityGrouping(), preempt_priority, sub_priority);
  /* Set priority */
  NVIC_SetPriority(irqn, temp_priority);
  /* Enable irqn */
  NVIC_EnableIRQ(irqn);
}

/**
  * @param  irqn (IRQn_Type number)
  * @return none
  * @brief  Disable nvic irq number
  */
void nvic_irq_disable(IRQn_Type irqn)
{
  NVIC_DisableIRQ(irqn);
}

/**
  * @param  the priority grouping bits length
  *         This parameter can be one of the following value:
  *         @arg NVIC_PriorityGroup_0: 0 bits for pre-emption priority
  *                                    3 bits for subpriority
  *         @arg NVIC_PriorityGroup_1: 1 bits for pre-emption priority
  *                                    2 bits for subpriority
  *         @arg NVIC_PriorityGroup_2: 2 bits for pre-emption priority
  *                                    1 bits for subpriority
  *         @arg NVIC_PriorityGroup_3: 3 bits for pre-emption priority
  *                                    0 bits for subpriority
  * @return none
  * @brief  Sets interrupt priority groups.
  */
void nvic_priority_group_config(uint32_t priority_group)
{
  /* Set the PRIGROUP[10:8] bits according to NVIC_PriorityGroup value */
  NVIC_SetPriorityGrouping(priority_group);
}

/**
	* @param  nvic_initstruct: NVIC_InitTypeDef Pointer
  * @return none
  * @brief  Initial interrupt priority.
  */
void nvic_init(NVIC_InitTypeDef* nvic_initstruct)
{
  uint32_t tmppriority=0x00,tmppre=0x00,tmpsub=0x0F;

  if(nvic_initstruct->NVIC_IRQChannelCmd!=DISABLE)
  {
    /* Compute the corresponding IRQ priority */
    tmppriority = (0x700 - ((SCB->AIRCR) & (uint32_t)0x700))>> 0x08;
    tmppre = (0x4 - tmppriority);
    tmpsub = tmpsub >> tmppriority;

    tmppriority = (uint32_t)nvic_initstruct->NVIC_IRQChannelPreemptionPriority << tmppre;
    tmppriority |=  nvic_initstruct->NVIC_IRQChannelSubPriority & tmpsub;
    tmppriority = tmppriority << 0x04;
        
    NVIC->IPR[nvic_initstruct->NVIC_IRQChannel] = tmppriority;
    
    /* Enable the Selected IRQ Channels */
    NVIC->ISER[nvic_initstruct->NVIC_IRQChannel >> 0x05] =
      (uint32_t)0x01 << (nvic_initstruct->NVIC_IRQChannel & (uint8_t)0x1F);
  }
  else
  {
    /* Disable the Selected IRQ Channels */
    NVIC->ICER[nvic_initstruct->NVIC_IRQChannel >> 0x05] =
      (uint32_t)0x01 << (nvic_initstruct->NVIC_IRQChannel & (uint8_t)0x1F);
  }
}

/**
  * @param  uint32_t the SysTick clock source
  * @return none
  * @brief  Sets systick clock source.
  */
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource)
{
	if(SysTick_CLKSource == SysTick_CLKSource_HCLK)
	{
		SysTick->CTRL |= SysTick_CLKSource_HCLK;
	}
	else
	{
		/* Don't support other clock source in Gowin_EMPU_M1 */
		SysTick->CTRL &= ~(uint32_t)SysTick_CLKSource_HCLK;
	}
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
