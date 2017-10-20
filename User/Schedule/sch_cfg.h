#ifndef __SCH_CFG_H
#define __SCH_CFG_H

#include "stm32f10x.h"

//定义数据类型
//typedef uint8_t u8;
//typedef uint16_t  u16;


//调度器节拍与硬件系统定时器相关定义
#define SCH_SYS_TICKS_MS 	    1	 	//定义调度系统时钟节拍时间(ms),无特殊情况不建议更改此项
#define SCH_HW_TIM_MS 	    	0.1	 	//硬件定时器中断(溢出)周期(ms),此项根据实际系统调整
#define SCH_TIM_TO_TICKS_CMP	(u8)(SCH_SYS_TICKS_MS/SCH_HW_TIM_MS)	//硬件定时器到系统节拍计数比较值


//定义可裁剪部分
#define SCH_CFG_Q_EN    1u  /* 任务内建消息使能 */



#endif      //__SCH_CFG_H

