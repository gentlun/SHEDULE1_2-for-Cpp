#ifndef __SCH_CFG_H
#define __SCH_CFG_H

#include "stm32f10x.h"

//������������
//typedef uint8_t u8;
//typedef uint16_t  u16;


//������������Ӳ��ϵͳ��ʱ����ض���
#define SCH_SYS_TICKS_MS 	    1	 	//�������ϵͳʱ�ӽ���ʱ��(ms),�����������������Ĵ���
#define SCH_HW_TIM_MS 	    	0.1	 	//Ӳ����ʱ���ж�(���)����(ms),�������ʵ��ϵͳ����
#define SCH_TIM_TO_TICKS_CMP	(u8)(SCH_SYS_TICKS_MS/SCH_HW_TIM_MS)	//Ӳ����ʱ����ϵͳ���ļ����Ƚ�ֵ


//����ɲü�����
#define SCH_CFG_Q_EN    1u  /* �����ڽ���Ϣʹ�� */



#endif      //__SCH_CFG_H

