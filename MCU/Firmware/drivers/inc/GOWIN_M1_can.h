/*
 **********************************************************************************************
 * @file      GOWIN_M1_can.h
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the CAN firmware library.
 **********************************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GOWIN_M1_CAN_H
#define GOWIN_M1_CAN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "GOWIN_M1.h"

/* Macros --------------------------------------------------------------------*/
#define NOAF     2 //Number of Acceptance Filters

#define FALSE 0
#define TRUE  1

//bit mask definition of Interrupt Status Register (IS)
#define CAN_ISR_RX_FIFO_VALID        (1UL << 0)
#define CAN_ISR_RX_FIFO_OVERFLOW     (1UL << 1)
#define CAN_ISR_TX_FIFO_OVERFLOW     (1UL << 4)
#define CAN_ISR_TX_HP_FIFO_OVERFLOW  (1UL << 5)
#define CAN_ISR_ERR_STATUS           (1UL << 8)
#define CAN_ISR_TX_HP_MESSAGE_FAIL   (1UL << 21)
#define CAN_ISR_TX_HP_MESSAGE_RETRY  (1UL << 22)
#define CAN_ISR_TX_HP_MESSAGE_OK     (1UL << 23)
#define CAN_ISR_TX_MESSAGE_FAIL      (1UL << 25)
#define CAN_ISR_TX_MESSAGE_RETRY     (1UL << 26)
#define CAN_ISR_TX_MESSAGE_OK        (1UL << 27)
#define CAN_ISR_BUS_OFF              (1UL << 31)

//bit mask definition of Interrupt Enable Register (IE)
#define CAN_IE_RX_FIFO_VALID         (1UL << 0)
#define CAN_IE_RX_FIFO_OVERFLOW      (1UL << 1)
#define CAN_IE_TX_FIFO_OVERFLOW      (1UL << 4)
#define CAN_IE_TX_HP_FIFO_OVERFLOW   (1UL << 5)
#define CAN_IE_ERR_STATUS            (1UL << 8)
#define CAN_IE_TX_HP_MESSAGE_FAIL    (1UL << 21)
#define CAN_IE_TX_HP_MESSAGE_RETRY   (1UL << 22)
#define CAN_IE_TX_HP_MESSAGE_OK      (1UL << 23)
#define CAN_IE_TX_MESSAGE_FAIL       (1UL << 25)
#define CAN_IE_TX_MESSAGE_RETRY      (1UL << 26)
#define CAN_IE_TX_MESSAGE_OK         (1UL << 27)
#define CAN_IE_BUS_OFF               (1UL << 31)

//bit mask definition of Interrupt Clear Register (IC)
#define CAN_IC_RX_FIFO_VALID         (1UL << 0)
#define CAN_IC_RX_FIFO_OVERFLOW      (1UL << 1)
#define CAN_IC_TX_FIFO_OVERFLOW      (1UL << 4)
#define CAN_IC_TX_HP_FIFO_OVERFLOW   (1UL << 5)
#define CAN_IC_ERR_STATUS            (1UL << 8)
#define CAN_IC_TX_HP_MESSAGE_FAIL    (1UL << 21)
#define CAN_IC_TX_HP_MESSAGE_RETRY   (1UL << 22)
#define CAN_IC_TX_HP_MESSAGE_OK      (1UL << 23)
#define CAN_IC_TX_MESSAGE_FAIL       (1UL << 25)
#define CAN_IC_TX_MESSAGE_RETRY      (1UL << 26)
#define CAN_IC_TX_MESSAGE_OK         (1UL << 27)
#define CAN_IC_BUS_OFF               (1UL << 31)

//bit mask definition of Configuration Register (CFG)
#define CAN_CFG_ISOFD               (1UL << 0)
#define CAN_CFG_DISPROTEXCEPONRES   (1UL << 4)
//Note: This bit is applicable and interpreted
//      only if the node is operating in ISO FD mode

//bit mask definition of Receive Acceptance Filter Register
#define CAN_AF_ENABLE         (1UL << 31)
#define CAN_AF_IDE            (1UL << 30)
#define CAN_AF_EDL            (1UL << 29)

/* Functions---------------------------------------------------------------------------*/
void can_srst(void);
void can_set_cmd(void);
void can_set_brp(unsigned char baud_rate_prescalar);
unsigned int can_read_brp(void);

//set sections in BTN (Bit Timing Nominal Register)
void can_set_btn_phseg1_nom(unsigned int phseg1_nom);
void can_set_btn_phseg2_nom(unsigned int phseg2_nom);
void can_set_btn_sjw_nom(unsigned int sjw_nom);
void can_set_btn(unsigned int phseg1_nom, unsigned int phseg2_nom, unsigned int sjw_nom);

//read sections in BTN
unsigned int can_read_btn_phseg1_nom(void);
unsigned int can_read_btn_phseg2_nom(void);
unsigned int can_read_btn_sjw_nom(void);

//set sections in BTD (Bit Timing Data Register)
void can_set_btd_phseg1_d(unsigned int phseg1_d);
void can_set_btd_phseg2_d(unsigned int phseg2_d);
void can_set_btd_sjw_d(unsigned int sjw_d);
void can_set_btd(unsigned int phseg1_d, unsigned int phseg2_d, unsigned int sjw_d);

//read sections in BTN
unsigned int can_read_btd_phseg1_d(void);
unsigned int can_read_btd_phseg2_d(void);
unsigned int can_read_btd_sjw_d(void);

//read bits in IS (Interrupt Status Register)
bool can_read_is_bit(unsigned int is_bit_mask);

//set bits in IE (Interrupt Enable Register)
void can_set_ie_bit(unsigned int ie_bit_mask);

//clear bits in IE (Interrupt Enable Register)
void can_clear_ie_bit(unsigned int ie_bit_mask);

//read bits in IE (Interrupt Enable Register)
bool can_read_ie_bit(unsigned int ie_bit_mask);

//set bits in IC (Interrupt Clear Register)
//to clear corresponding bits in IS (Interrupt Status Register)
void can_set_ic_bit(unsigned int ic_bit_mask);

//set bits in CFG (Configuration Register)
void can_set_cfg_bit_as_one(unsigned int cfg_bit_mask);

//clear bits in CFG (Configuration Register)
void can_set_cfg_bit_as_zero(unsigned int cfg_bit_mask);

//read bits in CFG (Configuration Register)
bool can_read_cfg_bit(unsigned int cfg_bit_mask);

//set rxb_start in RXBCFG (RX Buffer/fifo Configuration Register)
void can_set_rxbcfg_rxb_start(unsigned int rxb_start);

//read rxb_start in RXBCFG (RX Buffer/fifo Configuration Register)
unsigned int can_read_rxbcfg_rxb_start(void);

//set rxb_end in RXBCFG (RX Buffer/fifo Configuration Register)
void can_set_rxbcfg_rxb_end(unsigned int rxb_start);

//read rxb_end in RXBCFG (RX Buffer/fifo Configuration Register)
unsigned int can_read_rxbcfg_rxb_end(void);
void set_rxbcfg(unsigned int rxb_start, unsigned int rxb_end);

//set txb_start in TXBCFG (TX Buffer/fifo Configuation Register)
void can_set_txbcfg_txb_start(unsigned int txb_start);

//read txb_start in TXBCFG (TX Buffer/fifo Configuration Register)
unsigned int can_read_txbcfg_txb_start(void);

//set txb_end in TXBCFG (TX Buffer/fifo Configuration Register)
void can_set_txbcfg_txb_end(unsigned int txb_start);

//read rxb_end in TXBCFG (TX Buffer/fifo Configuration Register)
unsigned int can_read_txbcfg_txb_end(void);
void set_txbcfg(unsigned int txb_start, unsigned int txb_end);

//set txhb_start in TXHBCFG (TX High-Priority Buffer/fifo Configuation Register)
void can_set_txhbcfg_txhb_start(unsigned int txhb_start);

//read txhb_start in TXBCFG (TX High-Priority Buffer/fifo Configuration Register)
unsigned int can_read_txhbcfg_txhb_start(void);

//set txhb_end in TXBCFG (TX High-Priority Buffer/fifo Configuration Register)
void can_set_txhbcfg_txhb_end(unsigned int txhb_start);

//read rxhb_end in TXBCFG (TX High-Priority Buffer/fifo Configuration Register)
unsigned int can_read_txhbcfg_txhb_end(void);
void set_txhbcfg(unsigned int txhb_start, unsigned int txhb_end);

//set Tx Buffer Retry Count
void can_set_txbretry(unsigned int txbretry);

//read Tx Buffer Retry Count
unsigned int can_read_txbretry(void);

//set Tx High-Priority Buffer Retry Count
void can_set_txhbretry(unsigned int txhbretry);

//read Tx High-Priority Buffer Retry Count
unsigned int can_read_txhbretry(void);

//read Transmit Message Status Register - Message Status
unsigned int can_read_txmsgsts(void);

//read Transmit Message Status Register - Message ID
unsigned int can_read_txmsgid(void);

//read Transmit High-Priority Message Status Register - Message Status
unsigned int can_read_txhmsgsts(void);

//read Transmit High-Priority Message Status Register - Message ID
unsigned int can_read_txhmsgid(void);

//read Error Status Register
unsigned int can_read_errsts(void);

//read Error Counter Register - REC
unsigned int can_read_errcntr_rec(void);

//read Error Counter Register - TEC
unsigned int can_read_errcntr_tec(void);

//set specific bit as one in Receive Acceptance Filter
void can_set_af_bit_as_one(unsigned int af_bit_mask, unsigned int af_idx);

//set specific bit as zero in Receive Acceptance Filter
void can_set_af_bit_as_zero(unsigned int af_bit_mask, unsigned int af_idx);

//read specific bit in Receive Acceptance Filter
bool can_read_af_bit(unsigned int af_bit_mask, unsigned int af_idx);

void can_set_af_ie(unsigned int ie, unsigned int af_idx);
unsigned int can_read_af_ie(unsigned int af_idx);
void can_set_af_bid(unsigned int bid, unsigned int af_idx);
unsigned int can_read_af_bid(unsigned int af_idx);

void can_set_afm_ie(unsigned int ie, unsigned int afm_idx);
unsigned int can_read_afm_ie(unsigned int afm_idx);
void can_set_afm_bid(unsigned int bid, unsigned int afm_idx);
unsigned int can_read_afm_bid(unsigned int afm_idx);

unsigned int can_read_rxb(void);
void can_set_txb(unsigned int txb);
void can_set_txhb(unsigned int txhb);

//read txbspace in TXBSTS (Transmit Buffer/FIFO Status Register)
unsigned int can_read_txbsts_txbspace(void);

//read txbwerr in TXHBSTS (Transmit Buffer/FIFO Status Register)
bool can_read_txbsts_txbwerr(void);

//read txhbspace in TXHBSTS (Transmit High-Priority Buffer/FIFO Status Register)
unsigned int can_read_txhbsts_txhbspace(void);

//read txhbwerr in TXHBSTS (Transmit High-Priority Buffer/FIFO Status Register)
bool can_read_txhbsts_txhbwerr(void);

//read rxbdepth in RXBSTS (Receive Buffer/FIFO Status Register)
unsigned int can_read_rxbsts(void);

#ifdef __cplusplus
}
#endif

#endif /* GOWIN_M1_CAN_H */
