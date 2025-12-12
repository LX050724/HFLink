/*
 ********************************************************************************************
 * @file      GOWIN_M1_trng.h
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the TRNG firmware library.
 ********************************************************************************************
 */

 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GOWIN_M1_TRNG_H
#define GOWIN_M1_TRNG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "GOWIN_M1.h"

/* Macros --------------------------------------------------------------------*/
#define VN_ERR_INT_MASK       0x08
#define CRNGT_ERR_INT_MASK    0x04
#define AUTOCORR_ERR_INT_MASK 0x02
#define EHR_VALID_INT_MASK    0x01

/* Enumerations --------------------------------------------------------------*/
typedef enum
{
	VN_ERR,
	CRNGT_ERR,
	AUTOCORR_ERR,
	EHR_VALID
}RNG_ISR_STATE;

typedef enum
{
	SHORTEST_INVERT_LENGTH,
	SHORT_INVERT_LENGTH,
	LONG_INVERT_LENGTH,
	LONGEST_INVERT_LENGTH
}RND_SRC_SEL_STATE;

typedef enum
{
	BUSY,
	NOTBUSY
}TRNG_STATE;

typedef enum
{
	TRNG_AUTOCORR_BYPASS,
	TRNG_CRNGT_BYPASS,
	VNC_BYPASS
	
}TRNG_TEST_PASS;

/* Functions --------------------------------------------------------------------*/
extern void Set_Interrupt_Mask(uint8_t int_mask);
extern RNG_ISR_STATE Get_Int_State(void);
extern void Clear_Int(RNG_ISR_STATE isr_state);
extern void Set_Config(RND_SRC_SEL_STATE src_state);
extern void Get_EHR_Data(uint32_t * point);
extern void Set_Random_Source_Enable(void);
extern void Clr_Random_Source_Enable(void);
extern void Set_Sample_Count(uint32_t sample_count);
extern void Trng_SW_Reset(void);
extern TRNG_STATE Get_TRNG_State(void);
extern void Reset_Bit_Count(void);
extern void Get_BIT_Counter(uint32_t *point);
extern TRNG_TEST_PASS Set_Debug_Control(void);
extern uint32_t Fail_Start_State_times(void);
extern void Clr_Fail_Start_State_register(void);
extern void Init_TRNG(void);

#ifdef __cplusplus
}
#endif

#endif  /* GOWIN_M1_TRNG_H */
