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

#define	debugFLAG					0xF003

#define	debugCONFIG					(debugFLAG & 0x0001)
#define	debugCONVERT				(debugFLAG & 0x0002)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ##################################### Developer notes ###########################################


// ############################################# Macros ############################################

#define	mpl3115I2C_LOGIC				0					// 0 = delay, 1= stretch, 2= stages

// #################################### SI7006/13/20/21 Addresses ##################################

#define	mpl3115ADDR0				0x60
#define	MPL3115_T_SNS				60000


// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################


// ###################################### Local variables ##########################################

mpl3115_t sMPL3115 = { 0 };
uint8_t mpl3115NumDev;

// #################################### Local ONLY functions #######################################

int mpl3115ReadReg(uint8_t Reg, uint8_t * pRxBuf, size_t RxLen) {
	return halI2C_Queue(sMPL3115.psI2C, i2cWR_B, &Reg, sizeof(Reg),
			pRxBuf, RxLen, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

int mpl3115WriteReg(uint8_t reg, uint8_t val) {
	sMPL3115.u8Buf[reg] = val;
	uint8_t u8Buf[2] = { reg, val };
	return halI2C_Queue(sMPL3115.psI2C, i2cW_B, u8Buf, sizeof(u8Buf), NULL, 0, (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
}

#if (mpl3115I2C_LOGIC == 0)
/**
 * @brief	trigger A->D conversion with clock stretching
 * @param 	pointer to endpoint to be read
 */
int	mpl3115ReadHdlr(epw_t * psEWP) {
	IF_SYSTIMER_START(debugTIMING, stMPL3115);
	int iRV = mpl3115ReadReg(mpl3115R_STATUS, (uint8_t *) &sMPL3115.Reg, 6);
	IF_SYSTIMER_STOP(debugTIMING, stMPL3115);
	IF_PRINT(debugCONVERT, "mpl3115  [ %-'B ]\n", 6, &sMPL3115.Reg);
	x64_t X64;
	// Convert & update pressure/altitude sensor
	X64.x32[0].f32 = (float) (sMPL3115.Reg.OUT_P_MSB << 16 | sMPL3115.Reg.OUT_P_CSB << 8 | sMPL3115.Reg.OUT_P_LSB) / 64.0;
	vCV_SetValue(&table_work[URI_MPL3115_VAL].var, X64);
	// convert& update the temperature sensor
	X64.x32[0].f32 = (float) (sMPL3115.Reg.OUT_T_MSB << 8 | sMPL3115.Reg.OUT_T_LSB) / 256.0;
	vCV_SetValue(&table_work[URI_MPL3115_TMP].var, X64);
	return iRV;
}

#elif (mpl3115I2C_LOGIC == 1)
/**
 * @brief	trigger A->D conversion with clock stretching
 * @param 	pointer to endpoint to be read
 */
int	mpl3115ReadHdlr(epw_t * psEWP) {
	IF_TT(debugTIMING, "A\n");
	table_work[psEWP->uri == URI_MPL3115_RH ? URI_MPL3115_TMP : URI_MPL3115_RH].fBusy = 1;
	const uint8_t * pCMD = (psEWP == &table_work[URI_MPL3115_RH]) ? &mpl3115MRH_HMM : &mpl3115MT_HMM;
	IF_SYSTIMER_START(debugTIMING, stMPL3115);
	int iRV = halI2C_Queue(sMPL3115.psI2C, i2cWR_B, (uint8_t *) pCMD, sizeof(uint8_t),
			sMPL3115.u8Buf, SO_MEM(mpl3115_t, u8Buf), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	IF_SYSTIMER_STOP(debugTIMING, stMPL3115);
	IF_TT(debugCONVERT, "uri=%d  [ %-'B ]", psEWP->uri, SO_MEM(mpl3115_t, u8Buf), sMPL3115.u8Buf);
	sMPL3115.RawVal = (sMPL3115.u8Buf[0] << 8) + sMPL3115.u8Buf[1];
	x64_t X64;
	if (psEWP == &table_work[URI_MPL3115_RH])
		X64.x32[0].f32 = (float) (((sMPL3115.RawVal * 125) >> 16) - 6);
	else
		X64.x32[0].f32 = (((float) sMPL3115.RawVal * 175.72) / 65536.0) - 46.85;
	vCV_SetValue(&psEWP->var, X64);
	table_work[psEWP->uri == URI_MPL3115_RH ? URI_MPL3115_TMP : URI_MPL3115_RH].fBusy = 0;
	IF_PRINT(debugCONVERT, "  Raw=%d  Norm=%f\n", sMPL3115.RawVal, X64.x32[0].f32);
	return iRV;
}

#elif (mpl3115I2C_LOGIC == 2)

/**
 * @brief	step 3: sample read, convert  store
 * @param 	Expired timer handle
 */
 void mpl3115ReadCB(void * pvPara) {
	IF_SYSTIMER_STOP(debugTIMING, stMPL3115);
	epw_t * psEWP = pvPara;
	IF_TT(debugCONVERT, "uri=%d  [ %-'B ]", psEWP->uri, SO_MEM(mpl3115_t, u8Buf), sMPL3115.u8Buf);
	sMPL3115.RawVal = (sMPL3115.u8Buf[0] << 8) + sMPL3115.u8Buf[1];
	IF_PRINT(debugCONVERT, "  Raw=%d", sMPL3115.RawVal);
	x64_t X64;
	if (psEWP == &table_work[URI_MPL3115_RH]) {
//		IF_myASSERT(debugRESULT, (sMPL3115.RawVal & 0x0003) == 0x0002);
		sMPL3115.RawVal &= 0xFFFC;
		X64.x32[0].f32 = (float) (((sMPL3115.RawVal * 125) >> 16) - 6);
	} else {
//		IF_myASSERT(debugRESULT, (sMPL3115.RawVal & 0x0003) == 0x0000);
		X64.x32[0].f32 = (((float) sMPL3115.RawVal * 175.72) / 65536.0) - 46.85;
	}
	vCV_SetValue(&psEWP->var, X64);
	table_work[psEWP->uri == URI_MPL3115_RH ? URI_MPL3115_TMP : URI_MPL3115_RH].fBusy = 0;
	IF_PRINT(debugCONVERT, "  Norm=%f\n", X64.x32[0].f32);
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
	IF_TT(debugTIMING, "A\n");
	table_work[psEWP->uri == URI_MPL3115_RH ? URI_MPL3115_TMP : URI_MPL3115_RH].fBusy = 1;
	vTimerSetTimerID(sMPL3115.timer, (void *) psEWP);
	uint8_t Cfg = sMPL3115.sUR1.cfg1 ? 2 : 0;
	Cfg += sMPL3115.sUR1.cfg0 ? 1 : 0;
	uint32_t Dly = (psEWP == &table_work[URI_MPL3115_RH]) ? mpl3115DelayRH[Cfg] : mpl3115DelayT[Cfg];
	const uint8_t * pCMD = (psEWP == &table_work[URI_MPL3115_RH]) ? &mpl3115MRH_NHMM : &mpl3115MT_NHMM;
	IF_SYSTIMER_START(debugTIMING, stMPL3115);
	return halI2C_Queue(sMPL3115.psI2C, i2cWT, (uint8_t *) pCMD, 1, NULL, 0, (i2cq_p1_t) sMPL3115.timer, (i2cq_p2_t) (uint32_t) Dly);
}
#else
	#error " Invalid mpl3115I2C_LOGIC value"
#endif

// ################################ Rules configuration support ####################################

int	mpl3115ConfigMode (struct rule_t * psR, int Xcur, int Xmax, int EI) {
	// mode /mpl3115 idx res htr lev
	uint8_t	AI = psR->ActIdx;
	int res = psR->para.x32[AI][0].i32;
	int htr = psR->para.x32[AI][1].i32;
	int lev = psR->para.x32[AI][2].i32;
	IF_PRINT(debugCONFIG && ioB1GET(ioMode), "MODE 'MPL3115' Xcur=%d Xmax=%d res=%d htr=%d lev=%d\n", Xcur, Xmax, res, htr, lev);

	if (OUTSIDE(0, res, 3, int) || OUTSIDE(0, htr, 1, int) || OUTSIDE(0, lev, 15, int))
		ERR_RETURN("Invalid Resolution or Heater value", erSCRIPT_INV_PARA);

	int iRV;
	do { } while (++Xcur < Xmax) ;
	return iRV;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	mpl3115Identify(i2c_di_t * psI2C_DI) {
	uint8_t	u8Buf;
	psI2C_DI->Delay	= 50;
	psI2C_DI->TOuS = 400;
	psI2C_DI->Test = 1;
	sMPL3115.psI2C = psI2C_DI;
	int iRV = mpl3115ReadReg(mpl3115R_WHOAMI, &u8Buf, sizeof(u8Buf));
	if ((iRV == erSUCCESS) && (u8Buf == 0xC4)) {
		psI2C_DI->Type		= i2cDEV_MPL3115;
		psI2C_DI->Speed		= i2cSPEED_400;
		psI2C_DI->DevIdx 	= mpl3115NumDev++ ;
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
	mpl3115WriteReg(mpl3115R_CTRL_REG1, 0x02);
	mpl3115WriteReg(mpl3115R_PT_DATA_CFG, 0x07);
	mpl3115WriteReg(mpl3115R_CTRL_REG1, 0x03);

	mpl3115ConfigALT(&table_work[URI_MPL3115_VAL]);		// default mode on reset
	epw_t * psEWP = &table_work[URI_MPL3115_TMP];
	psEWP->var.def.cv.vc = 1;
	psEWP->var.def.cv.vs = vs32B;
	psEWP->var.def.cv.vf = vfFXX;
	psEWP->var.def.cv.vt = vtVALUE;
	psEWP->Tsns = psEWP->Rsns = MPL3115_T_SNS;
	psEWP->uri = URI_MPL3115_TMP;

#if (mpl3115I2C_LOGIC == 2)
	sMPL3115.timer = xTimerCreate("mpl3115", pdMS_TO_TICKS(5), pdFALSE, NULL, mpl3115TimerHdlr);
#endif
	IF_SYSTIMER_INIT(debugTIMING, stMPL3115, stMILLIS, "MPL3115", 1, 300);
	// Get initial register values
	mpl3115ReadReg(mpl3115R_STATUS, (uint8_t *) &sMPL3115.Reg, sizeof(mpl3115_reg_t));
	return erSUCCESS ;
}

int mpl3115ReConfig(i2c_di_t * psI2C_DI) { return erSUCCESS; }

int	mpl3115Diags(i2c_di_t * psI2C_DI) { return erSUCCESS; }

// ######################################### Reporting #############################################

void mpl3115ReportAll(void) {
	for (int dev = 0; dev < mpl3115NumDev; ++dev) {
		for (int reg = mpl3115R_STATUS; reg <= mpl3115R_OFF_H; ++reg) {
			switch (reg) {
			case mpl3115R_STATUS:
				break;
			case mpl3115R_DR_STATUS:
				sMPL3115.Reg.
				break;
			case mpl3115R_F_STATUS:
				break;
			case mpl3115R_F_DATA:
				break;
			case mpl3115R_F_SETUP:
				break;
			case mpl3115R_TIME_DLY:
				break;
			case mpl3115R_SYSMOD:
				break;
			case mpl3115R_INT_SOURCE:
				break;
			case mpl3115R_PT_DATA_CFG:
				break;
			case mpl3115R_CTRL_REG1:
				break;
			case mpl3115R_CTRL_REG2:
				break;
			case mpl3115R_CTRL_REG3:
				break;
			case mpl3115R_CTRL_REG4:
				break;
			case mpl3115R_CTRL_REG5:
				break;
			case mpl3115R_OFF_P:
				break;
			case mpl3115R_OFF_T:
				break;
			case mpl3115R_OFF_H:
				break;
/*
			case :
				break;
*/
			default:
				break;
			}
		}
	}
}
