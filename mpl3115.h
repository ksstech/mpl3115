/*
 * mpl3115.h - Copyright (c) 2022-24 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "struct_union.h"

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
	u8_t res2: 1;
	u8_t TDR:1;						// Temperature Data Ready
	u8_t PDR:1;						// Pressure/altitude Data Ready
	u8_t PTDR:1;					// Pressure/altitude or temperature Data Ready
	u8_t res1:1;
	u8_t TOW:1;						// Temperature overwrite
	u8_t POW:1;						// Pressure/altiture overwrite
	u8_t PTOW:1;					// Pressure/altitude or temperature overwrite
} mpl3115_dr_status_t;

typedef struct __attribute__((packed)) {				// F_STATUS
	u8_t F_CNT:6;
	u8_t F_WMRK_FLAG:1;
	u8_t F_OVF:1;
} mpl3115_f_status_t;

typedef struct __attribute__((packed)) {				// F_SETUP
	u8_t F_WMRK:6;
	u8_t F_MODE:2;
} mpl3115_f_setup_t;

typedef struct __attribute__((packed)) {				// INT_SOURCE
	u8_t SRC_TCHG:1;
	u8_t SRC_PCHG:1;
	u8_t SRC_TTH:1;
	u8_t SRC_PTH:1;
	u8_t SRC_TW:1;
	u8_t SRC_PW:1;
	u8_t SRC_FIFO:1;
	u8_t SRC_DRDY:1;
} mpl3115_int_source_t;

typedef struct __attribute__((packed)) {				// PT_DATA_CFG
	u8_t TDEFE:1;
	u8_t PDEFE:1;
	u8_t DREM:1;
	u8_t RES:5;
} mpl3115_pt_data_t;

typedef struct __attribute__((packed)) {				// CTRL_REG1
	u8_t SBYB:1;					// 0=Standby 1=Active
	u8_t OST:1;						// 1=OneShot Trigger
	u8_t RST:1;						// 1=Reset
	u8_t OS:3;						// Oversampling n^OS
	u8_t RES:1;						// reserved
	u8_t ALT:1;						// 0=BMP 1=ALT
} mpl3115_ctrl_reg1_t;

typedef struct __attribute__((packed)) {				// CTRL_REG2
	u8_t ST:4;						// LSB
	u8_t ALARM_SEL:1;
	u8_t LOAD_OUTPUT:1;
	u8_t RES:2;
} mpl3115_ctrl_reg2_t;

typedef struct __attribute__((packed)) {				// CTRL_REG3
	u8_t PP_OD2:1;					// INT2 0=PushPull 1=OpenDrain
	u8_t IPOL2:1;					// INT2 0=ActiveLow 1=ActiveHigh
	u8_t RES2:2;
	u8_t PP_OD1:1;					// INT1 0=PushPull 1=OpenDrain
	u8_t IPOL1:1;					// INT1 0=ActiveLow 1=ActiveHigh
	u8_t RES1:2;
} mpl3115_ctrl_reg3_t;

typedef struct __attribute__((packed)) {				// CTRL_REG4
	u8_t INT_EN_TCHG:1;				// LSB
	u8_t INT_EN_PCHG:1;
	u8_t INT_EN_TTH:1;
	u8_t INT_EN_PTH:1;
	u8_t INT_EN_TW:1;
	u8_t INT_EN_PW:1;
	u8_t INT_EN_FIFO:1;
	u8_t INT_EN_DRDY:1;
} mpl3115_ctrl_reg4_t;

typedef struct __attribute__((packed)) {				// CTRL_REG5
	u8_t INT_CFG_TCHG:1;			// LSB
	u8_t INT_CFG_PCHG:1;
	u8_t INT_CFG_TTH:1;
	u8_t INT_CFG_PTH:1;
	u8_t INT_CFG_TW:1;
	u8_t INT_CFG_PW:1;
	u8_t INT_CFG_FIFO:1;
	u8_t INT_CFG_DRDY:1;
} mpl3115_ctrl_reg5_t;

typedef struct __attribute__((packed)) {
	u8_t STATUS;					// either DR_STATUS or F_STATUS
	union {							// OUT_P
		Q18dot2_t OUT_P;
		struct __attribute__((packed)) {
			u8_t OUT_P_MSB;
			u8_t OUT_P_CSB;
			u8_t OUT_P_LSB;
		};
	};
	union {							// OUT_T
		Q8dot4_t OUT_T;
		struct __attribute__((packed)) {
			u8_t OUT_T_MSB;
			u8_t OUT_T_LSB;
		};
	};
	union {							// DR_STATUS
		mpl3115_dr_status_t dr_status;
		u8_t DR_STATUS;
	};
	union {							// OUT_P_DELTA
		Q18dot2_t OUT_P_DELTA;
		struct __attribute__((packed)) {
			u8_t OUT_P_DELTA_MSB;
			u8_t OUT_P_DELTA_CSB;
			u8_t OUT_P_DELTA_LSB;
		};
	};
	union {							// OUT_T_DELTA
		Q8dot4_t OUT_T_DELTA;
		struct __attribute__((packed)) {
			u8_t OUT_T_DELTA_MSB;
			u8_t OUT_T_DELTA_LSB;
		};
	};
	u8_t WHO_AM_I;
	union {							// F_STATUS
		mpl3115_f_status_t f_status;
		u8_t F_STATUS;
	};
	u8_t F_DATA;
	union {							// F_SETUP
		mpl3115_f_setup_t f_setup;
		u8_t F_SETUP;
	};
	u8_t TIME_DLY;
	u8_t SYSMOD;
	union {							// INT_SOURCE
		mpl3115_int_source_t int_source;
		u8_t INT_SOURCE;
	};
	union {							// PT_DATA_CFG
		mpl3115_pt_data_t pt_data_cfg;
		u8_t PT_DATA_CFG;
	};
	u8_t BAR_IN_MSB;
	u8_t BAR_IN_LSB;
	u8_t P_TGT_MSB;
	u8_t P_TGT_LSB;
	u8_t T_TGT;
	u8_t P_WND_MSB;
	u8_t P_WND_LSB;
	u8_t T_WND;
	union {							// P_MIN
		Q18dot2_t P_MIN;
		struct __attribute__((packed)) {
			u8_t P_MIN_MSB;
			u8_t P_MIN_CSB;
			u8_t P_MIN_LSB;
		};
	};
	union {							// T_MIN
		Q8dot4_t T_MIN;
		struct __attribute__((packed)) {
			u8_t T_MIN_MSB;
			u8_t T_MIN_LSB;
		};
	};
	union {							// P_MAX
		Q18dot2_t P_MAX;
		struct __attribute__((packed)) {
			u8_t P_MAX_MSB;
			u8_t P_MAX_CSB;
			u8_t P_MAX_LSB;
		};
	};
	union {							// T_MAX
		Q8dot4_t T_MAX;
		struct __attribute__((packed)) {
			u8_t T_MAX_MSB;
			u8_t T_MAX_LSB;
		};
	};
	union {							// CTRL_REG1
		mpl3115_ctrl_reg1_t ctrl_reg1;
		u8_t CTRL_REG1;
	};
	union {							// CTRL_REG2
		mpl3115_ctrl_reg2_t ctrl_reg2;
		u8_t CTRL_REG2;
	};
	union {							// CTRL_REG3
		mpl3115_ctrl_reg3_t ctrl_reg3;
		u8_t CTRL_REG3;
	};
	union {							// CTRL_REG4
		mpl3115_ctrl_reg4_t ctrl_reg4;
		u8_t CTRL_REG4;
	};
	union {							// CTRL_REG5
		mpl3115_ctrl_reg5_t ctrl_reg5;
		u8_t CTRL_REG5;
	};
	u8_t OFF_P;
	u8_t OFF_T;
	u8_t OFF_H;
} mpl3115_reg_t;
DUMB_STATIC_ASSERT(sizeof(mpl3115_reg_t) == 46);

struct i2c_di_t;
typedef struct __attribute__((packed)) {				// SI70006/13/14/20/xx TMP & RH sensors
	struct i2c_di_t * psI2C;		// 4 bytes
	SemaphoreHandle_t mux;
	#if (mpl3115I2C_LOGIC == 3)
	TimerHandle_t th;
	StaticTimer_t ts;
	#endif
	union {
		mpl3115_reg_t Reg;
		u8_t u8Buf[sizeof(mpl3115_reg_t)];
	};
	u8_t spare[2];
} mpl3115_t;
#if (mpl3115I2C_LOGIC == 3)
	DUMB_STATIC_ASSERT(sizeof(mpl3115_t) == 104);
#else
	DUMB_STATIC_ASSERT(sizeof(mpl3115_t) == 56);
#endif

struct report_t;

// ###################################### Public variables #########################################

extern mpl3115_t sMPL3115;
extern const u16_t mpl3115Dly[];

// ###################################### Public functions #########################################

int mpl3115ReadReg(u8_t Reg, u8_t * pRxBuf, size_t RxLen);
int mpl3115WriteReg(u8_t reg, u8_t val);

int mpl3115ReportAll(struct report_t * psR);

// ##################################### I2C Task support ##########################################

struct rule_t;
int	mpl3115ConfigMode (struct rule_t *, int Xcur, int Xmax, int EI);
int	mpl3115Identify(struct i2c_di_t * psI2C);
int	mpl3115Config(struct i2c_di_t * psI2C);
int	mpl3115Diags(struct i2c_di_t * psI2C);

struct epw_t;
int	mpl3115Sense(struct epw_t * psEWP);

#ifdef __cplusplus
}
#endif
