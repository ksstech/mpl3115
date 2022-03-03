/*
 * Copyright 2022 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#include	"mpl3115.h"
#include	<string.h>

#include	"hal_variables.h"
#include	"endpoints.h"
#include	"options.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"					// timing debugging
#include	"x_errors_events.h"

#define	debugFLAG					0xF001

#define	debugCONFIG					(debugFLAG & 0x0001)
#define	debugCONVERT				(debugFLAG & 0x0002)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ############################################# Macros ############################################

#define	mpl3115I2C_LOGIC			1					// 1=delay, 2=stretch, 3=stages

// #################################### SI7006/13/20/21 Addresses ##################################

#define	MPL3115_T_SNS				1000

// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################

const uint16_t mpl3115Dly[] = { 6, 10, 18, 34, 66, 130, 258, 512 };

// ###################################### Local variables ##########################################

mpl3115_t sMPL3115 = { 0 };

// #################################### Local ONLY functions #######################################

int mpl3115ReadReg(uint8_t Reg, uint8_t * pRxBuf, size_t RxLen) {
	return halI2C_Queue(sMPL3115.psI2C, i2cWR_B, &Reg, sizeof(Reg),
			pRxBuf, RxLen, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

int mpl3115WriteReg(uint8_t reg, uint8_t val) {
	uint8_t u8Buf[2] = { reg, val };
	return halI2C_Queue(sMPL3115.psI2C, i2cW_B, u8Buf, sizeof(u8Buf), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

#if (mpl3115I2C_LOGIC == 1)								// 1 step no wait
/**
 * @brief	simply read current values, expected it to be new/valid
 * @param 	pointer to endpoint to be read
 */
int	mpl3115ReadHdlr(epw_t * psEWP) {
	IF_SYSTIMER_START(debugTIMING, stMPL3115);
	int iRV = mpl3115ReadReg(mpl3115STATUS, (uint8_t *) &sMPL3115.Reg, 6);
	IF_SYSTIMER_STOP(debugTIMING, stMPL3115);
	IF_P(debugCONVERT, "mpl3115  [ %-'B ]\n", 6, &sMPL3115.Reg);
	x64_t X64;
	// Convert & update pressure/altitude sensor
	X64.x32[0].f32 = (float) (sMPL3115.Reg.OUT_P_MSB << 16 | sMPL3115.Reg.OUT_P_CSB << 8 | sMPL3115.Reg.OUT_P_LSB) / 64.0;
	vCV_SetValue(&table_work[URI_MPL3115_VAL].var, X64);
	// convert& update the temperature sensor
	X64.x32[0].f32 = (float) (sMPL3115.Reg.OUT_T_MSB << 8 | sMPL3115.Reg.OUT_T_LSB) / 256.0;
	vCV_SetValue(&table_work[URI_MPL3115_TMP].var, X64);
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
	IF_P(debugCONVERT, "mpl3115  [ %-'B ]\n", 6, &sMPL3115.Reg);
	x64_t X64;
	// Convert & update pressure/altitude sensor
	X64.x32[0].f32 = (float) (sMPL3115.Reg.OUT_P_MSB << 16 | sMPL3115.Reg.OUT_P_CSB << 8 | sMPL3115.Reg.OUT_P_LSB) / 64.0;
	vCV_SetValue(&table_work[URI_MPL3115_VAL].var, X64);
	// convert& update the temperature sensor
	X64.x32[0].f32 = (float) (sMPL3115.Reg.OUT_T_MSB << 8 | sMPL3115.Reg.OUT_T_LSB) / 256.0;
	vCV_SetValue(&table_work[URI_MPL3115_TMP].var, X64);
}

/**
 * @brief	step 2: conversion timer expired, trigger sample read
 * @param 	(expired) timer handle
 */
void mpl3115TimerHdlr(TimerHandle_t xTimer) {
	halI2C_Queue(sMPL3115.psI2C, i2cRC_B, NULL, 0, sMPL3115.u8Buf, SO_MEM(mpl3115_t, u8Buf),
			(i2cq_p1_t) mpl3115ReadCB, (i2cq_p2_t) (void *) pvTimerGetTimerID(xTimer));
}

/**
 * @brief	step 1: trigger A->D conversion with delay
 * @param 	pointer to endpoint to be read
 */
int	mpl3115ReadHdlr(epw_t * psEWP) {
	vTimerSetTimerID(sMPL3115.timer, (void *) psEWP);
	IF_SYSTIMER_START(debugTIMING, stMPL3115);
	uint8_t Cmd = mpl3115STATUS;
	// delay not really required if sampling interval >= 1000mSec
	uint32_t Dly = mpl3115Dly[sMPL3115.Reg.ctrl_reg1.OS] ;
	return halI2C_Queue(sMPL3115.psI2C, i2cWT, &Cmd, sizeof(Cmd),
			&sMPL3115.Reg.STATUS, 6, (i2cq_p1_t) sMPL3115.timer, (i2cq_p2_t) (uint32_t) Dly);
}
#endif

// ################################ Rules configuration support ####################################

int	mpl3115ConfigMode (struct rule_t * psR, int Xcur, int Xmax, int EI) {
	// mode /mpl3115 idx mode os Tstep [fifo] [event]
	uint8_t	AI = psR->ActIdx;
	int mode = psR->para.x32[AI][0].i32;
	int os = psR->para.x32[AI][1].i32;					// OverSampling 0 = 2^0 ... 2^7 ie 128
	int step = psR->para.x32[AI][2].i32;				// Auto Acquire time 1 -> 2^15 ie 9:06:00.8s

	IF_P(debugCONFIG && ioB1GET(ioMode), "MODE 'MPL3115' Xcur=%d Xmax=%d mode=%d os=%d step=%d\n", Xcur, Xmax, mode, os, step);

	if (OUTSIDE(0, mode, 1, int) || OUTSIDE(0, os, 7, int) || OUTSIDE(0, step, 15, int))
		ERR_RETURN("Invalid Resolution or Heater value", erINVALID_PARA);

	sMPL3115.Reg.ctrl_reg1.ALT = mode;
	sMPL3115.Reg.ctrl_reg1.OS = os;
	int iRV = mpl3115WriteReg(mpl3115CTRL_REG1, sMPL3115.Reg.CTRL_REG1);
	if (iRV == erSUCCESS) {
		sMPL3115.Reg.ctrl_reg2.ST = step ;
		iRV = mpl3115WriteReg(mpl3115CTRL_REG2, sMPL3115.Reg.CTRL_REG2);
	}
	return iRV;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	mpl3115Identify(i2c_di_t * psI2C_DI) {
	psI2C_DI->TRXmS	= 50;
	psI2C_DI->CLKuS = 400;
	psI2C_DI->Test = 1;
	sMPL3115.psI2C = psI2C_DI;
	int iRV = mpl3115ReadReg(mpl3115WHOAMI, &sMPL3115.Reg.WHO_AM_I, 1);
	if ((iRV == erSUCCESS) && (sMPL3115.Reg.WHO_AM_I == 0xC4)) {
		psI2C_DI->Type		= i2cDEV_MPL3115;
		psI2C_DI->Speed		= i2cSPEED_400;
		psI2C_DI->DevIdx 	= 0;
	}
	psI2C_DI->Test = 0;
	return iRV ;
}

void mpl3115ConfigALT(epw_t * psEWP) {
	psEWP->var.def.cv.vc = 1;
	psEWP->var.def.cv.vs = vs32B;
	psEWP->var.def.cv.vf = vfFXX;
	psEWP->var.def.cv.vt = vtVALUE;
	psEWP->Tsns = psEWP->Rsns = MPL3115_T_SNS;
	psEWP->uri = URI_MPL3115_VAL;
}

void mpl3115ConfigBMP(epw_t * psEWP) {
	psEWP->var.def.cv.vc = 1;
	psEWP->var.def.cv.vs = vs32B;
	psEWP->var.def.cv.vf = vfFXX;
	psEWP->var.def.cv.vt = vtVALUE;
	psEWP->Tsns = psEWP->Rsns = MPL3115_T_SNS;
	psEWP->uri = URI_MPL3115_VAL;
}

int	mpl3115Config(i2c_di_t * psI2C_DI) {
	sMPL3115.Reg.pt_data_cfg.DREM = 1;
	sMPL3115.Reg.pt_data_cfg.PDEFE = 1;
	sMPL3115.Reg.pt_data_cfg.TDEFE = 1;
	mpl3115WriteReg(mpl3115PT_DATA_CFG, sMPL3115.Reg.PT_DATA_CFG);
	sMPL3115.Reg.ctrl_reg1.SBYB = 1;
	mpl3115WriteReg(mpl3115CTRL_REG1, sMPL3115.Reg.CTRL_REG1);

	mpl3115ConfigALT(&table_work[URI_MPL3115_VAL]);		// default mode on reset
	epw_t * psEWP = &table_work[URI_MPL3115_TMP];
	psEWP->var.def.cv.vc = 1;
	psEWP->var.def.cv.vs = vs32B;
	psEWP->var.def.cv.vf = vfFXX;
	psEWP->var.def.cv.vt = vtVALUE;
	psEWP->Tsns = psEWP->Rsns = MPL3115_T_SNS;
	psEWP->uri = URI_MPL3115_TMP;

#if (mpl3115I2C_LOGIC == 3)
	sMPL3115.timer = xTimerCreate("mpl3115", pdMS_TO_TICKS(5), pdFALSE, NULL, mpl3115TimerHdlr);
#endif
	IF_SYSTIMER_INIT(debugTIMING, stMPL3115, stMICROS, "MPL3115", 10, 1000);
	return erSUCCESS ;
}

int mpl3115ReConfig(i2c_di_t * psI2C_DI) { return erSUCCESS; }

int	mpl3115Diags(i2c_di_t * psI2C_DI) { return erSUCCESS; }

// ######################################### Reporting #############################################

void mpl3115ReportAll(void) {
	halI2C_DeviceReport(sMPL3115.psI2C);
	mpl3115ReadReg(mpl3115DR_STATUS, (uint8_t *) &sMPL3115.Reg.DR_STATUS, 1);
	P("\tDR_STATUS: 0x%02X  PTOW=%d  POW=%d  TOW=%d  PTDR=%d  PDR=%d  TDR=%d\n", sMPL3115.Reg.DR_STATUS,
		sMPL3115.Reg.dr_status.PTOW, sMPL3115.Reg.dr_status.POW, sMPL3115.Reg.dr_status.TOW,
		sMPL3115.Reg.dr_status.PTDR, sMPL3115.Reg.dr_status.PDR, sMPL3115.Reg.dr_status.TDR);

	mpl3115ReadReg(mpl3115OUT_P_DELTA_MSB, (uint8_t *) &sMPL3115.Reg.OUT_P_DELTA_MSB, 5);
	P("\tOUT_P_D=%f  OUT_T_D=%f\n",
		(float) (sMPL3115.Reg.OUT_P_DELTA_MSB << 16 | sMPL3115.Reg.OUT_P_DELTA_CSB << 8 | sMPL3115.Reg.OUT_P_DELTA_LSB) / 64.0,
		(float) (sMPL3115.Reg.OUT_T_DELTA_MSB << 8 | sMPL3115.Reg.OUT_T_DELTA_LSB) / 256.0);

	mpl3115ReadReg(mpl3115F_STATUS, (uint8_t *) &sMPL3115.Reg.F_STATUS, 1);
	P("\tF_STATUS: 0x%02X  OVF=%d  WMRK=%d  CNT=%d\n", sMPL3115.Reg.F_STATUS,
		sMPL3115.Reg.f_status.F_OVF, sMPL3115.Reg.f_status.F_WMRK_FLAG, sMPL3115.Reg.f_status.F_CNT);

	mpl3115ReadReg(mpl3115F_SETUP, (uint8_t *) &sMPL3115.Reg.F_SETUP, 1);
	P("\tF_SETUP: 0x%02X  MODE=%d  WMRK=%d\n", sMPL3115.Reg.F_SETUP,
		sMPL3115.Reg.f_setup.F_MODE, sMPL3115.Reg.f_setup.F_WMRK);

	mpl3115ReadReg(mpl3115TIME_DLY, (uint8_t *) &sMPL3115.Reg.TIME_DLY, 30);
	P("\tTIME_DLY: %d\n", sMPL3115.Reg.TIME_DLY);
	P("\tSYSMOD: %sabled\n", sMPL3115.Reg.SYSMOD ? "EN" : "DIS");

	P("\tINT_SOURCE: 0x%02X\n", sMPL3115.Reg.INT_SOURCE);
	P("\tPT_DATA_CFG: 0x%02X  DREM=%d  PDEFE=%d  TDEFE=%d\n", sMPL3115.Reg.PT_DATA_CFG,
		sMPL3115.Reg.pt_data_cfg.DREM, sMPL3115.Reg.pt_data_cfg.PDEFE, sMPL3115.Reg.pt_data_cfg.TDEFE);
	uint16_t U16 = (sMPL3115.Reg.BAR_IN_MSB << 8) | sMPL3115.Reg.BAR_IN_LSB;
	P("\tBAR_IN: 0x%04X (%u)\n", U16, U16 << 1);

	x32_t X32 ;
	if (sMPL3115.Reg.ctrl_reg1.ALT)
		X32.f32 = (float) (sMPL3115.Reg.P_TGT_MSB << 8 | sMPL3115.Reg.P_TGT_LSB);
	else
		X32.f32 = (int16_t) (sMPL3115.Reg.P_TGT_MSB << 8 | sMPL3115.Reg.P_TGT_LSB);
	P("\tP_TGT=%f(%s)  T_TGT=%d(degC)\n", X32.f32, sMPL3115.Reg.ctrl_reg1.ALT ? "m" : "Pa", sMPL3115.Reg.T_TGT);

	if (sMPL3115.Reg.ctrl_reg1.ALT)
		X32.f32 = (float) (sMPL3115.Reg.P_WND_MSB << 8 | sMPL3115.Reg.P_WND_LSB);
	else
		X32.f32 = (int16_t) (sMPL3115.Reg.P_WND_MSB << 8 | sMPL3115.Reg.P_WND_LSB);
	P("\tP_WND=%f(%s)  T_WND=%d(degC)\n", X32.f32, sMPL3115.Reg.ctrl_reg1.ALT ? "m" : "Pa", sMPL3115.Reg.T_WND);

	P("\tP_MIN=%f  T_MIN=%f\n",
		(float) (sMPL3115.Reg.P_MIN_MSB << 16 | sMPL3115.Reg.P_MIN_CSB << 8 | sMPL3115.Reg.P_MIN_LSB) / 64.0,
		(float) (sMPL3115.Reg.T_MIN_MSB << 8 | sMPL3115.Reg.T_MIN_LSB) / 256.0);

	P("\tP_MAX=%f  T_MAX=%f\n",
		(float) (sMPL3115.Reg.P_MAX_MSB << 16 | sMPL3115.Reg.P_MAX_CSB << 8 | sMPL3115.Reg.P_MAX_LSB) / 64.0,
		(float) (sMPL3115.Reg.T_MAX_MSB << 8 | sMPL3115.Reg.T_MAX_LSB) / 256.0);

	P("\tCTRL_REG1: 0x%02X  ALT=%d  OS=%d  RST=%d  OST=%d  SBYB=%d\n", sMPL3115.Reg.CTRL_REG1,
		sMPL3115.Reg.ctrl_reg1.ALT, sMPL3115.Reg.ctrl_reg1.OS, sMPL3115.Reg.ctrl_reg1.RST,
		sMPL3115.Reg.ctrl_reg1.OST, sMPL3115.Reg.ctrl_reg1.SBYB);
	P("\tCTRL_REG2: 0x%02X  LOAD_OUTPUT=%d  ALARM_SEL=%d  ST=%d\n", sMPL3115.Reg.CTRL_REG2,
		sMPL3115.Reg.ctrl_reg2.LOAD_OUTPUT, sMPL3115.Reg.ctrl_reg2.ALARM_SEL, sMPL3115.Reg.ctrl_reg2.ST);
	P("\tCTRL_REG3: 0x%02X  IPOL1=%d  PP_OD1=%d  IPOL2=%d  PP_OD2=%d\n", sMPL3115.Reg.CTRL_REG3,
		sMPL3115.Reg.ctrl_reg3.IPOL1, sMPL3115.Reg.ctrl_reg3.PP_OD1,
		sMPL3115.Reg.ctrl_reg3.IPOL2, sMPL3115.Reg.ctrl_reg3.PP_OD2);
	P("\tCTRL_REG4=0x%02X   CTRL_REG5=0x%02X\n",
		sMPL3115.Reg.CTRL_REG4, sMPL3115.Reg.CTRL_REG5);

	P("\tOFF_P=0x%02X   OFF_T=0x%02X   OFF_H=0x%02X\n",
		sMPL3115.Reg.OFF_P, sMPL3115.Reg.OFF_T, sMPL3115.Reg.OFF_H);
}
