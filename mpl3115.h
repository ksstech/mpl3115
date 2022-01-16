/*
 * Copyright 2022 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#include <stdint.h>

#include "endpoint_struct.h"
#include "hal_i2c.h"

#ifdef __cplusplus
	extern "C" {
#endif

// ########################################### Macros ##############################################


// ######################################## Enumerations ###########################################

enum {
	mpl3115R_STATUS,
	mpl3115R_PRESSURE_MSB,
	mpl3115R_PRESSURE_CSB,
	mpl3115R_PRESSURE_LSB,
	mpl3115R_TEMP_MSB,
	mpl3115R_TEMP_LSB,
	mpl3115R_DR_STATUS,
	mpl3115R_OUT_P_DELTA_MSB,
	mpl3115R_OUT_P_DELTA_CSB,
	mpl3115R_OUT_P_DELTA_LSB,
	mpl3115R_OUT_T_DELTA_MSB,
	mpl3115R_OUT_T_DELTA_LSB,
	mpl3115R_WHOAMI,
	mpl3115R_F_STATUS,
	mpl3115R_F_DATA,
	mpl3115R_F_SETUP,
	mpl3115R_TIME_DLY,				// x10
	mpl3115R_SYSMOD,
	mpl3115R_INT_SOURCE,
	mpl3115R_PT_DATA_CFG,
	mpl3115R_BAR_IN_MSB,
	mpl3115R_BAR_IN_LSB,
	mpl3115R_P_TGT_MSB,
	mpl3115R_P_TGT_LSB,
	mpl3115R_T_TGT,
	mpl3115R_P_WND_MSB,
	mpl3115R_P_WND_LSB,
	mpl3115R_T_WND,
	mpl3115R_P_MIN_MSB,
	mpl3115R_P_MIN_CSB,
	mpl3115R_P_MIN_LSB,
	mpl3115R_T_MIN_MSB,
	mpl3115R_T_MIN_LSB,			// x20
	mpl3115R_P_MAX_MSB,
	mpl3115R_P_MAX_CSB,
	mpl3115R_P_MAX_LSB,
	mpl3115R_T_MAX_MSB,
	mpl3115R_T_MAX_LSB,
	mpl3115R_CTRL_REG1,
	mpl3115R_CTRL_REG2,
	mpl3115R_CTRL_REG3,
	mpl3115R_CTRL_REG4,
	mpl3115R_CTRL_REG5,
	mpl3115R_OFF_P,
	mpl3115R_OFF_T,
	mpl3115R_OFF_H,					// MSB
};

enum {								// Status register bits
	mpl3115R_STATUS_TDR = 0x02,
	mpl3115R_STATUS_PDR = 0x04,
	mpl3115R_STATUS_PTDR = 0x08,
};

enum {								// PT DATA register bits
	mpl3115PT_DATA_CFG = 0x13,
	mpl3115PT_DATA_CFG_TDEFE = 0x01,
	mpl3115PT_DATA_CFG_PDEFE = 0x02,
	mpl3115PT_DATA_CFG_DREM = 0x04,
};

enum {
	FIFO_DIS = 0, 		// FIFO is disabled (reset value)
	FIFO_ENAB = 1,		// FIFO contains the most recent samples when overflowed (circular buffer). Oldest sample is discarded to be replaced by new sample
	FIFO_STOP = 2,		// FIFO stops accepting new samples when overflowed
	FIFO_UNUSED = 3		// Not used
};

// ######################################### Structures ############################################

// See http://www.catb.org/esr/structure-packing/
// Also http://c0x.coding-guidelines.com/6.7.2.1.html

typedef struct __attribute__((packed)) {
	int	res1 : 4;					// LSB
	int fract : 4;
	int integ : 8;
} Q8dot4_t;

typedef struct __attribute__((packed)) {
	int	res1 : 4;					// LSB
	int fract : 2;
	int integ : 18;
} Q18dot2_t;

typedef struct __attribute__((packed)) {
	uint8_t res2: 1;				// LSB
	uint8_t	TDR : 1;
	uint8_t	PDR : 1;
	uint8_t	PTDR : 1;
	uint8_t	res1 : 1;
	uint8_t	TOW : 1;
	uint8_t	POW : 1;
	uint8_t	PTOW : 1;
} mpl3115_dr_status_t;

typedef struct __attribute__((packed)) {
	uint8_t F_CNT: 6;				// LSB
	uint8_t	F_WMRK_FLAG : 1;
	uint8_t	F_OVF : 1;
} mpl3115_f_status_t;

typedef struct __attribute__((packed)) {
	uint8_t F_WMRK : 6;				// LSB
	uint8_t	F_MODE : 2;
} mpl3115_f_setup_t;

typedef struct __attribute__((packed)) {
	uint8_t SRC_TCHG : 1;				// LSB
	uint8_t	SRC_PCHG : 1;
	uint8_t	SRC_TTH : 1;
	uint8_t	SRC_PTH : 1;
	uint8_t	SRC_TW : 1;
	uint8_t	SRC_PW : 1;
	uint8_t	SRC_FIFO : 1;
	uint8_t	SRC_DRDY : 1;
} mpl3115_int_source_t;

typedef struct __attribute__((packed)) {
	uint8_t TDEFE : 1;
	uint8_t	PDEFE : 1;
	uint8_t	DREM : 1;
	uint8_t RES : 5;
} mpl3115_pt_data_t;

typedef struct __attribute__((packed)) {				// CTRL_REG1
	uint8_t SBYB : 1;				// 0=Standby 1=Active
	uint8_t	OST : 1;				// 1=OneShot Trigger
	uint8_t	RST : 1;				// 1=Reset
	uint8_t OS : 3;					// Oversampling n^OS
	uint8_t RES : 1;				// reserved
	uint8_t ALT : 1;				// 0=BMP 1=ALT
} mpl3115_ctrl_reg1_t;

typedef struct __attribute__((packed)) {				// CTRL_REG2
	uint8_t ST : 4;				// LSB
	uint8_t	ALARM_SEL : 1;
	uint8_t	LOAD_OUTPUT : 1;
	uint8_t RES : 2;
} mpl3115_ctrl_reg2_t;

typedef struct __attribute__((packed)) {				// CTRL_REG3
	uint8_t PP_OD2 : 1;				// LSB
	uint8_t	IPOL2 : 1;
	uint8_t	RES2 : 2;
	uint8_t PP_OD1 : 1;
	uint8_t IPOL1 : 1;
	uint8_t RES1 : 2;
} mpl3115_ctrl_reg3_t;

typedef struct __attribute__((packed)) {				// CTRL_REG4
	uint8_t INT_EN_TCHG : 1;		// LSB
	uint8_t	INT_EN_PCHG : 1;
	uint8_t INT_EN_TTH : 1;
	uint8_t INT_EN_PTH : 1;
	uint8_t	INT_EN_TW : 1;
	uint8_t INT_EN_PW : 1;
	uint8_t INT_EN_FIFO : 1;
	uint8_t	INT_EN_DRDY : 1;
} mpl3115_ctrl_reg4_t;

typedef struct __attribute__((packed)) {				// CTRL_REG5
	uint8_t INT_CFG_TCHG : 1;		// LSB
	uint8_t	INT_CFG_PCHG : 1;
	uint8_t INT_CFG_TTH : 1;
	uint8_t INT_CFG_PTH : 1;
	uint8_t	INT_CFG_TW : 1;
	uint8_t INT_CFG_PW : 1;
	uint8_t INT_CFG_FIFO : 1;
	uint8_t	INT_CFG_DRDY : 1;
} mpl3115_ctrl_reg5_t;

typedef struct __attribute__((packed)) {
	uint8_t STATUS;					// LSB
	union {
		Q18dot2_t OUT_P;
		struct __attribute__((packed)) {
			uint8_t OUT_P_MSB;
			uint8_t OUT_P_CSB;
			uint8_t OUT_P_LSB;
		};
	};
	union {
		Q8dot4_t OUT_T;
		struct __attribute__((packed)) {
			uint8_t OUT_T_MSB;
			uint8_t OUT_T_LSB;
		};
	};
	union {
		mpl3115_dr_status_t dr_status;
		uint8_t DR_STATUS;
	};
	union {
		Q18dot2_t VAL_DELTA;
		struct __attribute__((packed)) {
			uint8_t OUT_P_DELTA_MSB;
			uint8_t OUT_P_DELTA_CSB;
			uint8_t OUT_P_DELTA_LSB;
		};
	};
	union {
		Q8dot4_t TMP_DELTA;
		struct __attribute__((packed)) {
			uint8_t OUT_T_DELTA_MSB;
			uint8_t OUT_T_DELTA_LSB;
		};
	};
	uint8_t WHO_AM_I;
	union {
		mpl3115_f_status_t f_status;
		uint8_t F_STATUS;
	};
	uint8_t F_DATA;
	union {
		mpl3115_f_setup_t f_setup;
		uint8_t F_SETUP;
	};
	uint8_t TIME_DLY;			// x10
	uint8_t SYSMOD;
	union {
		mpl3115_int_source_t int_source;
		uint8_t INT_SOURCE;
	};
	union {
		mpl3115_pt_data_t pt_data_cfg;
		uint8_t PT_DATA_CFG;
	};
	uint8_t BAR_IN_MSB;
	uint8_t BAR_IN_LSB;
	uint8_t P_TGT_MSB;
	uint8_t P_TGT_LSB;
	uint8_t T_TGT;
	uint8_t P_WND_MSB;
	uint8_t P_WND_LSB;
	uint8_t T_WND;
	union {
		Q18dot2_t P_MIN;
		struct __attribute__((packed)) {
			uint8_t P_MIN_MSB;
			uint8_t P_MIN_CSB;
			uint8_t P_MIN_LSB;
		};
	};
	union {
		Q8dot4_t T_MIN;
		struct __attribute__((packed)) {
			uint8_t T_MIN_MSB;
			uint8_t T_MIN_LSB;			// x20
		};
	};
	union {
		Q18dot2_t P_MAX;
		struct __attribute__((packed)) {
			uint8_t P_MAX_MSB;
			uint8_t P_MAX_CSB;
			uint8_t P_MAX_LSB;
		};
	};
	union {
		Q8dot4_t T_MAX;
		struct __attribute__((packed)) {
			uint8_t T_MAX_MSB;
			uint8_t T_MAX_LSB;
		};
	};
	union {
		mpl3115_ctrl_reg1_t CTRL1;
		uint8_t CTRL_REG1;
	};
	union {
		mpl3115_ctrl_reg1_t CTRL2;
		uint8_t CTRL_REG2;
	};
	union {
		mpl3115_ctrl_reg1_t CTRL3;
		uint8_t CTRL_REG3;
	};
	union {
		mpl3115_ctrl_reg1_t CTRL4;
		uint8_t CTRL_REG4;
	};
	union {
		mpl3115_ctrl_reg1_t CTRL5;
		uint8_t CTRL_REG5;
	};
	uint8_t OFF_P;
	uint8_t OFF_T;
	uint8_t OFF_H;					// MSB
} mpl3115_reg_t;
DUMB_STATIC_ASSERT(sizeof(mpl3115_reg_t) == 46);

typedef struct __attribute__((packed)) {				// SI70006/13/14/20/xx TMP & RH sensors
	i2c_di_t *		psI2C;			// 4 bytes
	SemaphoreHandle_t mux;
	TimerHandle_t	timer;
	union {
		mpl3115_reg_t Reg;
		uint8_t u8Buf[sizeof(mpl3115_reg_t)];
	};
	uint8_t	spare[2];
} mpl3115_t;
DUMB_STATIC_ASSERT(sizeof(mpl3115_t) == 60);

// ###################################### Public variables #########################################


// ###################################### Public functions #########################################

int	mpl3115ConvertTemperature(mpl3115_t * psMPL3115) ;
int	mpl3115ReadSP(mpl3115_t * psMPL3115, int Len) ;
int	mpl3115WriteSP(mpl3115_t * psMPL3115) ;
int	mpl3115WriteEE(mpl3115_t * psMPL3115) ;

int	mpl3115Initialize(mpl3115_t * psMPL3115) ;
int	mpl3115ResetConfig(mpl3115_t * psMPL3115) ;
void mpl3115ReportAll(void) ;

// ##################################### I2C Task support ##########################################

struct rule_t ;
int	mpl3115ConfigMode (struct rule_t *, int Xcur, int Xmax, int EI);
int	mpl3115Identify(i2c_di_t * psI2C_DI);
int	mpl3115Config(i2c_di_t * psI2C_DI);
int	mpl3115ReConfig(i2c_di_t * psI2C_DI);
int	mpl3115Diags(i2c_di_t * psI2C_DI);

struct epw_t ;
int	mpl3115ReadHdlr(epw_t * psEWP);

#ifdef __cplusplus
	}
#endif
