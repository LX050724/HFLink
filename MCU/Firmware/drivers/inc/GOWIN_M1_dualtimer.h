/*
 ************************************************************************************************
 * @file      GOWIN_M1_dualtimer.h
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the DualTimer firmware library.
 ************************************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GOWIN_M1_DUALTIMER_H
#define GOWIN_M1_DUALTIMER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "GOWIN_M1.h"

/* Macros --------------------------------------------------------------------*/
#define TIMER_ENABLE_BIT      7
#define TIMER_INTR_ENABLE_BIT 5
#define TIMER_SIZE_BIT        1

#define TIMER_MODE_BIT_H      6
#define TIMER_MODE_BIT_L      0

#define TIMER_PRE_BIT_H       3
#define TIMER_PRE_BIT_L       2


#define shift_set_H(num)    ((unsigned char) 0x01)<<num
#define shift_set_L(num)    ~(((unsigned char) 0x01)<<num)
	
/* Enumerations ---------------------------------------------------------------------*/
typedef enum 
{
	NON_PRE = 0x00,
	PRE_16  = 0x01,
	PRE_256 = 0x02
	
}TIMER_PRE;

typedef enum
{
	FRE_RUNNING   =0x00,
	RETIODIC_MODE =0x01,
	ONE_SHOT_MODE =0x02
}TIMER_MODE;

typedef enum
{
	OPEN_INTR  = 0x01,
	CLOSE_INTR = 0x00
}INTR_STATE;

typedef enum
{
	BIT16 = 0x00,
	BIT32 = 0x01

}TIMER_SIZE;

typedef enum
{
	TIMER1_ID = 0x01,
	TIMER2_ID = 0x02
}TIMER_ID;

/* Functions ---------------------------------------------------------------------*/
extern void DUALTIMER1_Init(void);
extern void DUALTIMER2_Init(void);
extern void Clear_DULATIMER_interrupt(TIMER_ID timer_id);
extern void Dtimer_MODE_function(TIMER_ID timer_id,TIMER_MODE timer_mode);
extern void Dtimer_PRE_function (TIMER_ID timer_id,TIMER_PRE timer_pre);
extern void INIT_NUM_load_function(TIMER_ID timer_id,uint32_t loadnum);
extern void ENABLE_interrupt_Dtimer_function(TIMER_ID timer_id,INTR_STATE intr_state);
extern void TIMER_SIZE_function(TIMER_ID timer_id,TIMER_SIZE timer_size);
extern void ENABLE_Dtimer_function(TIMER_ID timer_id);
extern TIMER_ID Get_DULATIMER_interrupt_num(void);

#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_DUALTIMER_H */
