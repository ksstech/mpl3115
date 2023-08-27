/*
 * Copyright 2022 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#include "hal_i2c_common.h"
#include "endpoints.h"
#include <stdint.h>

#ifdef __cplusplus
	extern "C" {
#endif

// ########################################### Macros ##############################################

#define	mpl3115ADDR0				0x60				// SI7006/13/20/21 Addresses

#define	mpl3115I2C_LOGIC			3					// 1=delay, 2=stretch, 3=stages

#define	MPL3115_T_SNS				1000

// ######################################## Enumerations ###########################################

enum {								// register numbers
	mpl3115STATUS,
	mpl3115PRESSURE_MSB,
	mpl3115PRESSURE_CSB,
	mpl3115PRESSURE_LSB,
	mpl3115TEMP_MSB,
	mpl3115TEMP_LSB,
	mpl3115DR_STATUS,
	mpl3115OUT_P_DELTA_MSB,
	mpl3115OUT_P_DELTA_CSB,
	mpl3115OUT_P_DELTA_LSB,
	mpl3115OUT_T_DELTA_MSB,
	mpl3115OUT_T_DELTA_LSB,
	mpl3115WHOAMI,
	mpl3115F_STATUS,
	mpl3115F_DATA,
	mpl3115F_SETUP,
	mpl3115TIME_DLY,
	mpl3115SYSMOD,
	mpl3115INT_SOURCE,
	mpl3115PT_DATA_CFG,
	mpl3115BAR_IN_MSB,
	mpl3115BAR_IN_LSB,
	mpl3115P_TGT_MSB,
	mpl3115P_TGT_LSB,
	mpl3115T_TGT,
	mpl3115P_WND_MSB,
	mpl3115P_WND_LSB,
	mpl3115T_WND,
	mpl3115P_MIN_MSB,
	mpl3115P_MIN_CSB,
	mpl3115P_MIN_LSB,
	mpl3115T_MIN_MSB,
	mpl3115T_MIN_LSB,
	mpl3115P_MAX_MSB,
	mpl3115P_MAX_CSB,
	mpl3115P_MAX_LSB,
	mpl3115T_MAX_MSB,
	mpl3115T_MAX_LSB,
	mpl3115CTRL_REG1,
	mpl3115CTRL_REG2,
	mpl3115CTRL_REG3,
	mpl3115CTRL_REG4,
	mpl3115CTRL_REG5,
	mpl3115OFF_P,
	mpl3115OFF_T,
	mpl3115OFF_H,
};

// ######################################### Structures ############################################

typedef struct __attribute__((packed)) {				// DR_STATUS
	uint8_t res2: 1;
	uint8_t	TDR : 1;
	uint8_t	PDR : 1;
	uint8_t	PTDR : 1;
	uint8_t	res1 : 1;
	uint8_t	TOW : 1;
	uint8_t	POW : 1;
	uint8_t	PTOW : 1;
} mpl3115_dr_status_t;

typedef struct __attribute__((packed)) {				// F_STATUS
	uint8_t F_CNT: 6;
	uint8_t	F_WMRK_FLAG : 1;
	uint8_t	F_OVF : 1;
} mpl3115_f_status_t;

typedef struct __attribute__((packed)) {				// F_SETUP
	uint8_t F_WMRK : 6;
	uint8_t	F_MODE : 2;
} mpl3115_f_setup_t;

typedef struct __attribute__((packed)) {				// INT_SOURCE
	uint8_t SRC_TCHG : 1;
	uint8_t	SRC_PCHG : 1;
	uint8_t	SRC_TTH : 1;
	uint8_t	SRC_PTH : 1;
	uint8_t	SRC_TW : 1;
	uint8_t	SRC_PW : 1;
	uint8_t	SRC_FIFO : 1;
	uint8_t	SRC_DRDY : 1;
} mpl3115_int_source_t;

typedef struct __attribute__((packed)) {				// PT_DATA_CFG
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
	uint8_t PP_OD2 : 1;				// INT2 0=PushPull 1=OpenDrain
	uint8_t	IPOL2 : 1;				// INT2 0=ActiveLow 1=ActiveHigh
	uint8_t	RES2 : 2;
	uint8_t PP_OD1 : 1;				// INT1 0=PushPull 1=OpenDrain
	uint8_t IPOL1 : 1;				// INT1 0=ActiveLow 1=ActiveHigh
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
	uint8_t STATUS;					// either DR_STATUS or F_STATUS
	union {							// OUT_P
		Q18dot2_t OUT_P;
		struct __attribute__((packed)) {
			uint8_t OUT_P_MSB;
			uint8_t OUT_P_CSB;
			uint8_t OUT_P_LSB;
		};
	};
	union {							// OUT_T
		Q8dot4_t OUT_T;
		struct __attribute__((packed)) {
			uint8_t OUT_T_MSB;
			uint8_t OUT_T_LSB;
		};
	};
	union {							// DR_STATUS
		mpl3115_dr_status_t dr_status;
		uint8_t DR_STATUS;
	};
	union {							// OUT_P_DELTA
		Q18dot2_t OUT_P_DELTA;
		struct __attribute__((packed)) {
			uint8_t OUT_P_DELTA_MSB;
			uint8_t OUT_P_DELTA_CSB;
			uint8_t OUT_P_DELTA_LSB;
		};
	};
	union {							// OUT_T_DELTA
		Q8dot4_t OUT_T_DELTA;
		struct __attribute__((packed)) {
			uint8_t OUT_T_DELTA_MSB;
			uint8_t OUT_T_DELTA_LSB;
		};
	};
	uint8_t WHO_AM_I;
	union {							// F_STATUS
		mpl3115_f_status_t f_status;
		uint8_t F_STATUS;
	};
	uint8_t F_DATA;
	union {							// F_SETUP
		mpl3115_f_setup_t f_setup;
		uint8_t F_SETUP;
	};
	uint8_t TIME_DLY;
	uint8_t SYSMOD;
	union {							// INT_SOURCE
		mpl3115_int_source_t int_source;
		uint8_t INT_SOURCE;
	};
	union {							// PT_DATA_CFG
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
	union {							// P_MIN
		Q18dot2_t P_MIN;
		struct __attribute__((packed)) {
			uint8_t P_MIN_MSB;
			uint8_t P_MIN_CSB;
			uint8_t P_MIN_LSB;
		};
	};
	union {							// T_MIN
		Q8dot4_t T_MIN;
		struct __attribute__((packed)) {
			uint8_t T_MIN_MSB;
			uint8_t T_MIN_LSB;
		};
	};
	union {							// P_MAX
		Q18dot2_t P_MAX;
		struct __attribute__((packed)) {
			uint8_t P_MAX_MSB;
			uint8_t P_MAX_CSB;
			uint8_t P_MAX_LSB;
		};
	};
	union {							// T_MAX
		Q8dot4_t T_MAX;
		struct __attribute__((packed)) {
			uint8_t T_MAX_MSB;
			uint8_t T_MAX_LSB;
		};
	};
	union {							// CTRL_REG1
		mpl3115_ctrl_reg1_t ctrl_reg1;
		uint8_t CTRL_REG1;
	};
	union {							// CTRL_REG2
		mpl3115_ctrl_reg2_t ctrl_reg2;
		uint8_t CTRL_REG2;
	};
	union {							// CTRL_REG3
		mpl3115_ctrl_reg3_t ctrl_reg3;
		uint8_t CTRL_REG3;
	};
	union {							// CTRL_REG4
		mpl3115_ctrl_reg4_t ctrl_reg4;
		uint8_t CTRL_REG4;
	};
	union {							// CTRL_REG5
		mpl3115_ctrl_reg5_t ctrl_reg5;
		uint8_t CTRL_REG5;
	};
	uint8_t OFF_P;
	uint8_t OFF_T;
	uint8_t OFF_H;
} mpl3115_reg_t;
DUMB_STATIC_ASSERT(sizeof(mpl3115_reg_t) == 46);

typedef struct __attribute__((packed)) {				// SI70006/13/14/20/xx TMP & RH sensors
	i2c_di_t *		psI2C;			// 4 bytes
	SemaphoreHandle_t mux;
	#if (mpl3115I2C_LOGIC == 3)
	TimerHandle_t th;
	StaticTimer_t ts;
	#endif
	union {
		mpl3115_reg_t Reg;
		uint8_t u8Buf[sizeof(mpl3115_reg_t)];
	};
	uint8_t	spare[2];
} mpl3115_t;
#if (mpl3115I2C_LOGIC == 3)
	DUMB_STATIC_ASSERT(sizeof(mpl3115_t) == 104);
#else
	DUMB_STATIC_ASSERT(sizeof(mpl3115_t) == 56);
#endif

// ###################################### Public variables #########################################


// ###################################### Public functions #########################################

int	mpl3115ConvertTemperature(mpl3115_t * psMPL3115);
int	mpl3115ReadSP(mpl3115_t * psMPL3115, int Len);
int	mpl3115WriteSP(mpl3115_t * psMPL3115);
int	mpl3115WriteEE(mpl3115_t * psMPL3115);

int	mpl3115Initialize(mpl3115_t * psMPL3115);
int	mpl3115ResetConfig(mpl3115_t * psMPL3115);
int mpl3115ReportAllreport * psR);

// ##################################### I2C Task support ##########################################

struct rule_t;
int	mpl3115ConfigMode (struct rule_t *, int Xcur, int Xmax, int EI);
int	mpl3115Identify(i2c_di_t * psI2C_DI);
int	mpl3115Config(i2c_di_t * psI2C_DI);
int	mpl3115ReConfig(i2c_di_t * psI2C_DI);
int	mpl3115Diags(i2c_di_t * psI2C_DI);

struct epw_t;
int	mpl3115Sense(epw_t * psEWP);

#ifdef __cplusplus
	}
#endif
