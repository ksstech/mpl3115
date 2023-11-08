/*
 * mpl3115.c
 * Copyright 2022 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_config.h"

#if (halHAS_MPL3115 > 0)
#include "endpoints.h"
#include "hal_i2c_common.h"
#include "mpl3115.h"
#include "printfx.h"
#include "rules.h"
#include "syslog.h"
#include "systiming.h"					// timing debugging
#include "x_errors_events.h"

#define	debugFLAG					0xF000

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ############################################# Macros ############################################


// ################################ Forward function declaration ###################################

void mpl3115ReadCB(void *);

// ######################################### Constants #############################################

const u16_t mpl3115Dly[] = { 6, 10, 18, 34, 66, 130, 258, 512 };

// ###################################### Local variables ##########################################

mpl3115_t sMPL3115 = { 0 };

// #################################### Local ONLY functions #######################################

int mpl3115ReadReg(u8_t Reg, u8_t * pRxBuf, size_t RxLen) {
	return halI2C_Queue(sMPL3115.psI2C, i2cWR_B, &Reg, sizeof(Reg), pRxBuf, RxLen, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

int mpl3115WriteReg(u8_t reg, u8_t val) {
	u8_t u8Buf[2] = { reg, val };
	return halI2C_Queue(sMPL3115.psI2C, i2cW_B, u8Buf, sizeof(u8Buf), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

#if (mpl3115I2C_LOGIC == 1)								// 1 step no wait
/**
 * @brief	simply read current values, expected it to be new/valid
 * @param 	pointer to endpoint to be read
 */
int	mpl3115Sense(epw_t * psEWP) {
	IF_SYSTIMER_START(debugTIMING, stMPL3115);
	int iRV = mpl3115ReadReg(mpl3115STATUS, (u8_t *) &sMPL3115.Reg, 6);
	IF_SYSTIMER_STOP(debugTIMING, stMPL3115);
	x64_t X64;
	// Convert & update pressure/altitude sensor
	X64.x32[0].f32 = (float) (sMPL3115.Reg.OUT_P_MSB << 16 | sMPL3115.Reg.OUT_P_CSB << 8 | sMPL3115.Reg.OUT_P_LSB) / 64.0;
	vCV_SetValueRaw(&table_work[URI_MPL3115_VAL].var, X64);
	// convert& update the temperature sensor
	X64.x32[0].f32 = (float) (sMPL3115.Reg.OUT_T_MSB << 8 | sMPL3115.Reg.OUT_T_LSB) / 256.0;
	vCV_SetValueRaw(&table_work[URI_MPL3115_TMP].var, X64);
	return iRV;
}

#elif (mpl3115I2C_LOGIC == 2)							// clock stretching

	#error "Not supported"

#elif (mpl3115I2C_LOGIC == 3)							// 3 stages

/**
 * @brief	step 3: sample read, convert  store
 * @param 	Expired timer handle
 */
void mpl3115ReadCB(void * pvPara) {
	IF_SYSTIMER_STOP(debugTIMING, stMPL3115);
	x64_t X64;
	// Convert & update pressure/altitude sensor
	X64.x32[0].f32 = (float) (sMPL3115.Reg.OUT_P_MSB << 16 | sMPL3115.Reg.OUT_P_CSB << 8 | sMPL3115.Reg.OUT_P_LSB) / 64.0;
	vCV_SetValueRaw(&table_work[URI_MPL3115_VAL].var, X64);
	// convert& update the temperature sensor
	X64.x32[0].f32 = (float) (sMPL3115.Reg.OUT_T_MSB << 8 | sMPL3115.Reg.OUT_T_LSB) / 256.0;
	vCV_SetValueRaw(&table_work[URI_MPL3115_TMP].var, X64);
}

/**
 * @brief	step 2: conversion timer expired, trigger sample read
 * @param 	(expired) timer handle
 */
void mpl3115TimerHdlr(TimerHandle_t xTimer) {
	halI2C_Queue(sMPL3115.psI2C, i2cRC_B, NULL, 0, sMPL3115.u8Buf, SO_MEM(mpl3115_t, u8Buf), (i2cq_p1_t) mpl3115ReadCB, (i2cq_p2_t) (void *) pvTimerGetTimerID(xTimer));
}

void mpl3115SenseCB(void * pV) {
	vTimerSetTimerID(sMPL3115.th, pV);
	// delay required if sampling interval < 1000mSec
	xTimerStart(sMPL3115.th, pdMS_TO_TICKS(mpl3115Dly[sMPL3115.Reg.ctrl_reg1.OS]));
}

/**
 * @brief	step 1: trigger A->D conversion with delay
 * @param 	pointer to endpoint to be read
 */
int	mpl3115Sense(epw_t * psEWP) {
	IF_SYSTIMER_START(debugTIMING, stMPL3115);
	u8_t Cmd = mpl3115STATUS;
	return halI2C_Queue(sMPL3115.psI2C, i2cWC, &Cmd, sizeof(Cmd), &sMPL3115.Reg.STATUS, 6, (i2cq_p1_t) mpl3115SenseCB, (i2cq_p2_t) (void *) psEWP);
}
#endif

// ################################ Rules configuration support ####################################

int	mpl3115ConfigMode (struct rule_t * psR, int Xcur, int Xmax, int EI) {
	// mode /mpl3115 idx mode os Tstep [fifo] [event]
	u8_t AI = psR->ActIdx;
	i32_t mode = psR->para.x32[AI][0].i32;
	i32_t os = psR->para.x32[AI][1].i32;				// OverSampling 0 = 2^0 ... 2^7 ie 128
	i32_t step = psR->para.x32[AI][2].i32;				// Auto Acquire time 1 -> 2^15 ie 9:06:00.8s
	IF_P(debugTRACK && ioB1GET(dbgMode), "MODE 'MPL3115' Xcur=%d Xmax=%d mode=%ld os=%ld step=%ld\r\n", Xcur, Xmax, mode, os, step);

	if (OUTSIDE(0, mode, 1) || OUTSIDE(0, os, 7) || OUTSIDE(0, step, 15))
		RETURN_MX("Invalid Resolution or Heater value", erINV_PARA);

	sMPL3115.Reg.ctrl_reg1.ALT = mode;
	sMPL3115.Reg.ctrl_reg1.OS = os;
	int iRV = mpl3115WriteReg(mpl3115CTRL_REG1, sMPL3115.Reg.CTRL_REG1);
	IF_RETURN_X(iRV != erSUCCESS, iRV);
	sMPL3115.Reg.ctrl_reg2.ST = step;
	return mpl3115WriteReg(mpl3115CTRL_REG2, sMPL3115.Reg.CTRL_REG2);
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	mpl3115Identify(i2c_di_t * psI2C) {
	sMPL3115.psI2C = psI2C;
	psI2C->Type = i2cDEV_MPL3115;
	psI2C->Speed = i2cSPEED_400;
	psI2C->TObus = 25;
	psI2C->Test = 1;
	int iRV = mpl3115ReadReg(mpl3115WHOAMI, &sMPL3115.Reg.WHO_AM_I, 1);
	if (iRV < erSUCCESS) goto exit;
	if (sMPL3115.Reg.WHO_AM_I != 0xC4) goto err_whoami;
	psI2C->IDok = 1;
	psI2C->Test = 0;
	goto exit;
err_whoami:
	iRV = erINV_WHOAMI;
exit: //RPL(" [iRV=%d] ", iRV);
	return iRV;
}

int	mpl3115Config(i2c_di_t * psI2C) {
	if (!psI2C->IDok) return erINV_STATE;

	psI2C->CFGok = 0;
	sMPL3115.Reg.pt_data_cfg.DREM = 1;					// Data Ready Event Mode
	sMPL3115.Reg.pt_data_cfg.PDEFE = 1;					// Pressure Data Event Flag Enable
	sMPL3115.Reg.pt_data_cfg.TDEFE = 1;					// Temperature Data Event Flag Enable
	int iRV = mpl3115WriteReg(mpl3115PT_DATA_CFG, sMPL3115.Reg.PT_DATA_CFG);
	if (iRV < erSUCCESS) goto exit;

	sMPL3115.Reg.ctrl_reg1.SBYB = 0;					// Put into standby mode
	iRV = mpl3115WriteReg(mpl3115CTRL_REG1, sMPL3115.Reg.CTRL_REG1);
	if (iRV < erSUCCESS) goto exit;

	sMPL3115.Reg.ctrl_reg1.ALT = 1;						// Change pressure to altitude readings
	sMPL3115.Reg.ctrl_reg1.SBYB = 1;
	iRV = mpl3115WriteReg(mpl3115CTRL_REG1, sMPL3115.Reg.CTRL_REG1);
	if (iRV < erSUCCESS) goto exit;
	psI2C->CFGok = 1;

	// once off init....
	if (!psI2C->CFGerr) {
		IF_SYSTIMER_INIT(debugTIMING, stMPL3115, stMICROS, "MPL3115", 10, 1000);
		#if (mpl3115I2C_LOGIC == 3)
		sMPL3115.th = xTimerCreateStatic("mpl3115", pdMS_TO_TICKS(5), pdFALSE, NULL, mpl3115TimerHdlr, &sMPL3115.ts);
		IF_myASSERT(debugRESULT, sMPL3115.th);
		#endif
	}
exit: //RPL("iRV=%d  ", iRV);
	return iRV;
}

int	mpl3115Diags(i2c_di_t * psI2C) { return erSUCCESS; }

// ######################################### Reporting #############################################

int mpl3115ReportAll(report_t * psR) {
	int iRV = halI2C_DeviceReport(psR, sMPL3115.psI2C);
	mpl3115ReadReg(mpl3115DR_STATUS, (u8_t *) &sMPL3115.Reg.DR_STATUS, 1);
	iRV += wprintfx(psR, "\tDR_STATUS: 0x%02X  PTOW=%d  POW=%d  TOW=%d  PTDR=%d  PDR=%d  TDR=%d\r\n", sMPL3115.Reg.DR_STATUS,
		sMPL3115.Reg.dr_status.PTOW, sMPL3115.Reg.dr_status.POW, sMPL3115.Reg.dr_status.TOW,
		sMPL3115.Reg.dr_status.PTDR, sMPL3115.Reg.dr_status.PDR, sMPL3115.Reg.dr_status.TDR);

	mpl3115ReadReg(mpl3115OUT_P_DELTA_MSB, (u8_t *) &sMPL3115.Reg.OUT_P_DELTA_MSB, 5);
	iRV += wprintfx(psR, "\tOUT_P_D=%f  OUT_T_D=%f\r\n",
		(float) (sMPL3115.Reg.OUT_P_DELTA_MSB << 16 | sMPL3115.Reg.OUT_P_DELTA_CSB << 8 | sMPL3115.Reg.OUT_P_DELTA_LSB) / 64.0,
		(float) (sMPL3115.Reg.OUT_T_DELTA_MSB << 8 | sMPL3115.Reg.OUT_T_DELTA_LSB) / 256.0);

	mpl3115ReadReg(mpl3115F_STATUS, (u8_t *) &sMPL3115.Reg.F_STATUS, 1);
	iRV += wprintfx(psR, "\tF_STATUS: 0x%02X  OVF=%d  WMRK=%d  CNT=%d\r\n", sMPL3115.Reg.F_STATUS,
		sMPL3115.Reg.f_status.F_OVF, sMPL3115.Reg.f_status.F_WMRK_FLAG, sMPL3115.Reg.f_status.F_CNT);

	mpl3115ReadReg(mpl3115F_SETUP, (u8_t *) &sMPL3115.Reg.F_SETUP, 1);
	iRV += wprintfx(psR, "\tF_SETUP: 0x%02X  MODE=%d  WMRK=%d\r\n", sMPL3115.Reg.F_SETUP,
		sMPL3115.Reg.f_setup.F_MODE, sMPL3115.Reg.f_setup.F_WMRK);

	mpl3115ReadReg(mpl3115TIME_DLY, (u8_t *) &sMPL3115.Reg.TIME_DLY, 30);
	iRV += wprintfx(psR, "\tTIME_DLY: %d\r\n", sMPL3115.Reg.TIME_DLY);
	iRV += wprintfx(psR, "\tSYSMOD: %sabled\r\n", sMPL3115.Reg.SYSMOD ? "EN" : "DIS");

	iRV += wprintfx(psR, "\tINT_SOURCE: 0x%02X\r\n", sMPL3115.Reg.INT_SOURCE);
	iRV += wprintfx(psR, "\tPT_DATA_CFG: 0x%02X  DREM=%d  PDEFE=%d  TDEFE=%d\r\n", sMPL3115.Reg.PT_DATA_CFG,
		sMPL3115.Reg.pt_data_cfg.DREM, sMPL3115.Reg.pt_data_cfg.PDEFE, sMPL3115.Reg.pt_data_cfg.TDEFE);
	uint16_t U16 = (sMPL3115.Reg.BAR_IN_MSB << 8) | sMPL3115.Reg.BAR_IN_LSB;
	iRV += wprintfx(psR, "\tBAR_IN: 0x%04X (%u)\r\n", U16, U16 << 1);

	x32_t X32;
	if (sMPL3115.Reg.ctrl_reg1.ALT) X32.f32 = (float) (sMPL3115.Reg.P_TGT_MSB << 8 | sMPL3115.Reg.P_TGT_LSB);
	else X32.f32 = (int16_t) (sMPL3115.Reg.P_TGT_MSB << 8 | sMPL3115.Reg.P_TGT_LSB);
	iRV += wprintfx(psR, "\tP_TGT=%f(%s)  T_TGT=%d(degC)\r\n", X32.f32, sMPL3115.Reg.ctrl_reg1.ALT ? "m" : "Pa", sMPL3115.Reg.T_TGT);

	if (sMPL3115.Reg.ctrl_reg1.ALT) X32.f32 = (float) (sMPL3115.Reg.P_WND_MSB << 8 | sMPL3115.Reg.P_WND_LSB);
	else X32.f32 = (int16_t) (sMPL3115.Reg.P_WND_MSB << 8 | sMPL3115.Reg.P_WND_LSB);
	iRV += wprintfx(psR, "\tP_WND=%f(%s)  T_WND=%d(degC)\r\n", X32.f32, sMPL3115.Reg.ctrl_reg1.ALT ? "m" : "Pa", sMPL3115.Reg.T_WND);

	iRV += wprintfx(psR, "\tP_MIN=%f  T_MIN=%f\r\n",
		(float) (sMPL3115.Reg.P_MIN_MSB << 16 | sMPL3115.Reg.P_MIN_CSB << 8 | sMPL3115.Reg.P_MIN_LSB) / 64.0,
		(float) (sMPL3115.Reg.T_MIN_MSB << 8 | sMPL3115.Reg.T_MIN_LSB) / 256.0);

	iRV += wprintfx(psR, "\tP_MAX=%f  T_MAX=%f\r\n",
		(float) (sMPL3115.Reg.P_MAX_MSB << 16 | sMPL3115.Reg.P_MAX_CSB << 8 | sMPL3115.Reg.P_MAX_LSB) / 64.0,
		(float) (sMPL3115.Reg.T_MAX_MSB << 8 | sMPL3115.Reg.T_MAX_LSB) / 256.0);

	iRV += wprintfx(psR, "\tCTRL_REG1: 0x%02X  ALT=%d  OS=%d  RST=%d  OST=%d  SBYB=%d\r\n", sMPL3115.Reg.CTRL_REG1,
		sMPL3115.Reg.ctrl_reg1.ALT, sMPL3115.Reg.ctrl_reg1.OS, sMPL3115.Reg.ctrl_reg1.RST,
		sMPL3115.Reg.ctrl_reg1.OST, sMPL3115.Reg.ctrl_reg1.SBYB);
	iRV += wprintfx(psR, "\tCTRL_REG2: 0x%02X  LOAD_OUTPUT=%d  ALARM_SEL=%d  ST=%d\r\n", sMPL3115.Reg.CTRL_REG2,
		sMPL3115.Reg.ctrl_reg2.LOAD_OUTPUT, sMPL3115.Reg.ctrl_reg2.ALARM_SEL, sMPL3115.Reg.ctrl_reg2.ST);
	iRV += wprintfx(psR, "\tCTRL_REG3: 0x%02X  IPOL1=%d  PP_OD1=%d  IPOL2=%d  PP_OD2=%d\r\n", sMPL3115.Reg.CTRL_REG3,
		sMPL3115.Reg.ctrl_reg3.IPOL1, sMPL3115.Reg.ctrl_reg3.PP_OD1,
		sMPL3115.Reg.ctrl_reg3.IPOL2, sMPL3115.Reg.ctrl_reg3.PP_OD2);
	iRV += wprintfx(psR, "\tCTRL_REG4=0x%02X   CTRL_REG5=0x%02X\r\n",
		sMPL3115.Reg.CTRL_REG4, sMPL3115.Reg.CTRL_REG5);

	iRV += wprintfx(psR, "\tOFF_P=0x%02X   OFF_T=0x%02X   OFF_H=0x%02X\r\n",
		sMPL3115.Reg.OFF_P, sMPL3115.Reg.OFF_T, sMPL3115.Reg.OFF_H);
	#if (mpl3115I2C_LOGIC == 3)
	iRV += xRtosReportTimer(psR, sMPL3115.th);
	#endif
	return iRV;
}

#endif
