/*
 *********************************************************************************************
 * @file      GOWIN_M1_CAN.c
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     This file contains all the functions prototypes for the CAN firmware library.
 *********************************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "GOWIN_M1_can.h"


/* Definitions ---------------------------------------------------------------*/
inline void can_srst(void)
{
	CAN->SRST = 1;
}

inline void can_set_cmd(void)
{
	CAN->CMD = 1;
}

/**************************set and read BRP*******************************/
inline void can_set_brp(unsigned char baud_rate_prescalar)
{
	CAN->BRP &= baud_rate_prescalar;
}
inline unsigned int can_read_brp(void)
{
	return CAN->BRP;
}

/************************set sections in BTN***********************************/
inline void can_set_btn_phseg1_nom(unsigned int phseg1_nom)
{
	unsigned int btn = CAN->BTN & 0xFFFFFF00;
	phseg1_nom &= 0x000000FF;
	btn |= phseg1_nom;
	CAN->BTN = btn;
}

inline void can_set_btn_phseg2_nom(unsigned int phseg2_nom)
{
	unsigned int btn = CAN->BTN & 0xFFFF00FF;
	phseg2_nom = phseg2_nom << 8;
	phseg2_nom &= 0x0000FF00;
	btn |= phseg2_nom;
	CAN->BTN = btn;
}

inline void can_set_btn_sjw_nom(unsigned int sjw_nom)
{
	unsigned int btn = CAN->BTN & 0x00FFFFFF;
	sjw_nom = sjw_nom << 24;
	sjw_nom &= 0xFF000000;
	btn |= sjw_nom;
	CAN->BTN = btn;
}

inline void can_set_btn(unsigned int phseg1_nom, unsigned int phseg2_nom, unsigned int sjw_nom)
{
	unsigned int btn = (phseg1_nom) + (phseg2_nom << 8) + (sjw_nom << 24);
	CAN->BTN = btn;
}

/***********************read sections in BTN**************************/
inline unsigned int can_read_btn_phseg1_nom(void)
{
	return (CAN->BTN & 0x000000FF);
}

inline unsigned int can_read_btn_phseg2_nom(void)
{
	return ((CAN->BTN & 0x0000FF00) >> 8);
}

inline unsigned int can_read_btn_sjw_nom(void)
{
	return ((CAN->BTN & 0xFF000000) >> 24);
}

/***********************set sections in BTD****************************/
inline void can_set_btd_phseg1_d(unsigned int phseg1_d)
{
	unsigned int btd = CAN->BTD & 0xFFFFFF00;
	phseg1_d &= 0x000000FF;
	btd |= phseg1_d;
	CAN->BTD = btd;
}

inline void can_set_btd_phseg2_d(unsigned int phseg2_d)
{
	unsigned int btd = CAN->BTD & 0xFFFF00FF;
	phseg2_d = phseg2_d << 8;
	phseg2_d &= 0x0000FF00;
	btd |= phseg2_d;
	CAN->BTD = btd;
}

inline void can_set_btd_sjw_d(unsigned int sjw_d)
{
	unsigned int btd = CAN->BTD & 0x00FFFFFF;
	sjw_d = sjw_d << 24;
	sjw_d &= 0xFF000000;
	btd |= sjw_d;
	CAN->BTD = btd;
}

inline void can_set_btd(unsigned int phseg1_d, unsigned int phseg2_d, unsigned int sjw_d)
{
	unsigned int btd = (phseg1_d) + (phseg2_d << 8) + (sjw_d << 24);
	CAN->BTD = btd;
}

/********************read sections in BTD******************************/
inline unsigned int can_read_btd_phseg1_d(void)
{
	return (CAN->BTD & 0x000000FF);
}

inline unsigned int can_read_btd_phseg2_d(void)
{
	return ((CAN->BTD & 0x0000FF00) >> 8);
}

inline unsigned int can_read_btd_sjw_d(void)
{
	return ((CAN->BTD & 0xFF000000) >> 24);
}

/***********************read bits in IS*******************************/
inline bool can_read_is_bit(unsigned int is_bit_mask)
{
	return ((CAN->IS & is_bit_mask) ? TRUE : FALSE);
}

/************************set bits in IE*******************************/
inline void can_set_ie_bit(unsigned int ie_bit_mask)
{
	CAN->IE |= ie_bit_mask;
}

/************************clear bits in IE*****************************/
inline void can_clear_ie_bit(unsigned int ie_bit_mask)
{
	CAN->IE &= (~ie_bit_mask);
}

/*************************read bits in IE*****************************/
inline bool can_read_ie_bit(unsigned int ie_bit_mask)
{
	return ((CAN->IS & ie_bit_mask) ? TRUE : FALSE);
}

/***************set bits in IC (Interrupt Clear Register)*************/
/****to clear corresponding bits in IS (Interrupt Status Register)****/
inline void can_set_ic_bit(unsigned int ic_bit_mask)
{
	CAN->IC |= ic_bit_mask;
}

/*****************set bits in CFG as 1(Configuration Register)************/
inline void can_set_cfg_bit_as_one(unsigned int cfg_bit_mask)
{
	CAN->CFG |= cfg_bit_mask;
}

/*****************set bits in CFG as 0 (Configuration Register)***********/
inline void can_set_cfg_bit_as_zero(unsigned int cfg_bit_mask)
{
	CAN->CFG &= (~cfg_bit_mask);
}

/*****************read bits in CFG (Configuration Register)***************/
inline bool can_read_cfg_bit(unsigned int cfg_bit_mask)
{
	return ((CAN->CFG & cfg_bit_mask) ? TRUE : FALSE);
}

/****set rxb_start in RXBCFG (RX Buffer/fifo Configuration Register)******/
inline void can_set_rxbcfg_rxb_start(unsigned int rxb_start)
{
	rxb_start &= 0x0000FFFF;
	CAN->RXBCFG &= 0xFFFF0000;
	CAN->RXBCFG |= rxb_start;
}

/****read rxb_start in RXBCFG (RX Buffer/fifo Configuration Register)*****/
inline unsigned int can_read_rxbcfg_rxb_start(void)
{
	return (CAN->RXBCFG & 0x0000FFFF);
}

/*****set rxb_end in RXBCFG (RX Buffer/fifo Configuration Register)*******/
inline void can_set_rxbcfg_rxb_end(unsigned int rxb_start)
{
	CAN->RXBCFG &= 0x0000FFFF;
	CAN->RXBCFG |= (rxb_start << 16);
}

/****read rxb_end in RXBCFG (RX Buffer/fifo Configuration Register)*******/
inline unsigned int can_read_rxbcfg_rxb_end(void)
{
	return (CAN->RXBCFG >> 16);
}

/***set rxb_start and rxb_end in RXBCFG (RX Buffer/fifo Configuration Register)***/
inline void set_rxbcfg(unsigned int rxb_start, unsigned int rxb_end)
{
	unsigned int rxbcfg = rxb_start + (rxb_end << 16);
	CAN->RXBCFG = rxbcfg;
}

/***set txb_start in TXBCFG (TX Buffer/fifo Configuation Register)***/
inline void can_set_txbcfg_txb_start(unsigned int txb_start)
{
	txb_start &= 0x0000FFFF;
	CAN->TXBCFG &= 0xFFFF0000;
	CAN->TXBCFG |= txb_start;
}

/***read txb_start in TXBCFG (TX Buffer/fifo Configuration Register)***/
inline unsigned int can_read_txbcfg_txb_start(void)
{
	return (CAN->TXBCFG & 0x0000FFFF);
}

/***set txb_end in TXBCFG (TX Buffer/fifo Configuration Register)***/
inline void can_set_txbcfg_txb_end(unsigned int txb_start)
{
	CAN->TXBCFG &= 0x0000FFFF;
	CAN->TXBCFG |= (txb_start << 16);
}

/***read rxb_end in TXBCFG (TX Buffer/fifo Configuration Register)***/
inline unsigned int can_read_txbcfg_txb_end(void)
{
	return (CAN->TXBCFG >> 16);
}

/***set txb_end and txb_end in TXBCFG (TX Buffer/fifo Configuration Register)***/
inline void set_txbcfg(unsigned int txb_start, unsigned int txb_end)
{
	unsigned int txbcfg = txb_start + (txb_end << 16);
	CAN->TXBCFG = txbcfg;
}

/***set txhb_start in TXHBCFG (TX High-Priority Buffer/fifo Configuation Register)***/
inline void can_set_txhbcfg_txhb_start(unsigned int txhb_start)
{
	txhb_start &= 0x0000FFFF;
	CAN->TXHBCFG &= 0xFFFF0000;
	CAN->TXHBCFG |= txhb_start;
}

/***read txhb_start in TXBCFG (TX High-Priority Buffer/fifo Configuration Register)***/
inline unsigned int can_read_txhbcfg_txhb_start(void)
{
	return (CAN->TXHBCFG & 0x0000FFFF);
}

/***set txhb_end in TXBCFG (TX High-Priority Buffer/fifo Configuration Register)***/
inline void can_set_txhbcfg_txhb_end(unsigned int txhb_start)
{
	CAN->TXHBCFG &= 0x0000FFFF;
	CAN->TXHBCFG |= (txhb_start << 16);
}

/***read txhb_end in TXBCFG (TX High-Priority Buffer/fifo Configuration Register)***/
inline unsigned int can_read_txhbcfg_txhb_end(void)
{
	return (CAN->TXHBCFG >> 16);
}

/***set txhb_start and txhb_end in TXBCFG (TX High-Priority Buffer/fifo Configuration Register)***/
inline void set_txhbcfg(unsigned int txhb_start, unsigned int txhb_end)
{
	unsigned int txhbcfg = txhb_start + (txhb_end << 16);
	CAN->TXHBCFG = txhbcfg;
}

/******************************set tx buffer retry count************************************/
inline void can_set_txbretry(unsigned int txbretry)
{
	CAN->TXBRETRY = txbretry;
}

/*****************************read tx buffer retry count************************************/
inline unsigned int can_read_txbretry(void)
{
	return CAN->TXBRETRY;
}

/***********************set tx high-priority buffer retry count*****************************/
inline void can_set_txhbretry(unsigned int txhbretry)
{
	CAN->TXHBRETRY = txhbretry;
}

/**********************read tx high-priority buffer retry count*****************************/
inline unsigned int can_read_txhbretry(void)
{
	return CAN->TXHBRETRY;
}

/******************read txmsgsts in transmit message status register************************/
inline unsigned int can_read_txmsgsts(void)
{
	return ((CAN->TXMSGSTS) >> 30);
}

/******************read txmgid in transmit message status register**************************/
inline unsigned int can_read_txmsgid(void)
{
	return ((CAN->TXMSGSTS) & 0x1FFFFFFF);
}

/**********read txhmsgsts in transmit high-priority message status register*****************/
inline unsigned int can_read_txhmsgsts(void)
{
	return ((CAN->TXHMSGSTS) >> 30);
}

/**********read txhmgid in transmit high-priority message status register*******************/
inline unsigned int can_read_txhmsgid(void)
{
	return ((CAN->TXHMSGSTS) & 0x1FFFFFFF);
}

/*****************************read Error Status Register*************************************/
inline unsigned int can_read_errsts(void)
{
	return (CAN->ERRSTS);
}

/*************************read rec in Error Counter Register*********************************/
inline unsigned int can_read_errcntr_rec(void)
{
	return ((CAN->ERRCNTR) & 0x000001FF);
}

/*************************read tec in Error Counter Register*********************************/
inline unsigned int can_read_errcntr_tec(void)
{
	return ((CAN->ERRCNTR) >> 16);
}

/**********set specific bit in Receive Acceptance Filter Register as 1 by af_bit_mask******************/
inline void can_set_af_bit_as_one(unsigned int af_bit_mask, unsigned int af_idx)
{
	CAN->AF[af_idx] |= af_bit_mask;
}

/**********set specific bit in Receive Acceptance Filter Register as 0 by af_bit_mask******************/
inline void can_set_af_bit_as_zero(unsigned int af_bit_mask, unsigned int af_idx)
{
	CAN->AF[af_idx] &= (~af_bit_mask);
}

/**********read specific bit in Receive Acceptance Filter Register by af_bit_mask**********************/
inline bool can_read_af_bit(unsigned int af_bit_mask, unsigned int af_idx)
{
	return ((CAN->AF[af_idx]) & af_bit_mask);
}

/***********set IE in Receive Acceptance Filter Register by af_bit_mask********************************/
inline void can_set_af_ie(unsigned int ie, unsigned int af_idx)
{
	ie = ie & 0x0003FFFF;
	CAN->AF[af_idx] &= (~0x0003FFFF);
	CAN->AF[af_idx] |= ie;
}

/***********read IE in Receive Acceptance Filter Register by af_bit_mask********************************/
inline unsigned int can_read_af_ie(unsigned int af_idx)
{
	return ((CAN->AF[af_idx]) & 0x0003FFFF);
}

/***********set BID in Receive Acceptance Filter Register by af_bit_mask********************************/
inline void can_set_af_bid(unsigned int bid, unsigned int af_idx)
{
	bid = bid << 18;
	CAN->AF[af_idx] &= (~0x1FFC0000);
	CAN->AF[af_idx] |= bid;
}

/***********read BID in Receive Acceptance Filter Register by af_bit_mask********************************/
inline unsigned int can_read_af_bid(unsigned int af_idx)
{
	return (((CAN->AF[af_idx]) & 0x1FFC0000) >> 18);
}

/****************set IE in Receive Acceptance Filter Mask Register by af_bit_mask**************************/
inline void can_set_afm_ie(unsigned int ie, unsigned int afm_idx)
{
	ie = ie & 0x0003FFFF;
	CAN->AFM[afm_idx] &= (~0x0003FFFF);
	CAN->AFM[afm_idx] |= ie;
}

/*******************read IE in Receive Acceptance Filter Mask Register by af_bit_mask**********************/
inline unsigned int can_read_afm_ie(unsigned int afm_idx)
{
	return ((CAN->AFM[afm_idx]) & 0x0003FFFF);
}

/***********set BID in Receive Acceptance Filter Mask Register by af_bit_mask********************************/
inline void can_set_afm_bid(unsigned int bid, unsigned int afm_idx)
{
	bid = bid << 18;
	CAN->AFM[afm_idx] &= (~0x1FFC0000);
	CAN->AFM[afm_idx] |= bid;
}

/***********read BIDM in Receive Acceptance Filter Mask Register by af_bit_mask********************************/
inline unsigned int can_read_afm_bid(unsigned int afm_idx)
{
	return (((CAN->AFM[afm_idx]) & 0x1FFC0000) >> 18);
}

inline unsigned int can_read_rxb(void)
{
	return (CAN->RXB);
}

inline void can_set_txb(unsigned int txb)
{
	CAN->TXB = txb;
}

inline void can_set_txhb(unsigned int txhb)
{
	CAN->TXHB = txhb;
}

inline unsigned int can_read_txbsts_txbspace(void)
{
	return ((CAN->TXBSTS) & 0x0000FFFF);
}

inline bool can_read_txbsts_txbwerr(void)
{
	return((((CAN->TXBSTS) >> 31) ? TRUE : FALSE));
}

inline unsigned int can_read_txhbsts_txhbspace(void)
{
	return ((CAN->TXHBSTS) & 0x0000FFFF);
}

inline bool can_read_txhbsts_txhbwerr(void)
{
	return((((CAN->TXHBSTS) >> 31) ? TRUE : FALSE));
}

inline unsigned int can_read_rxbsts(void)
{
	return ((CAN->RXBSTS) & 0x0000FFFF);
}
