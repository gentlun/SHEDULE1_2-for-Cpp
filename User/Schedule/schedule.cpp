/*********************************Copyright(c)*********************************
**                     Electrical Technology Co.,Ltd..
**
**                       http://www.***.com
**
**----------------------------File Info----------------------------------------
** File Name:            schedule.c
** Latest modified Date: 2013-05-18
** Description:          ��������ع���ʵ�ֺ�����
**							 
**-----------------------------------------------------------------------------
** Created By:           QQ 237765062
** Created date:         2013-04-13
** Descriptions:         ����ͳһͨ�ó�����
**
**-----------------------------------------------------------------------------
** Modified by:          QQ 237765062 
** Modified date:        2014-04-28
** Description:          ȥ����SCHTaskSchedStart()�������е�"goto"��䣬�á�if else��
**							         �߼���ʵ��,�汾����Ϊ V1.02
**-----------------------------------------------------------------------------
** Modified by:          QQ 237765062 
** Modified date:        2014-07-11
** Description:          "SCHTaskCreate"�������������ʼ��ʱ��ֵ,��ʵ�ָ������������,
**							         �Ӷ����Ч��,�汾����Ϊ V2.0
******************************************************************************/

#include "schedule.h"

SCH_TCB *pFirstTCB, *pCurTCB;

static MAX_TASK_T TaskNumberSum = 0;
u8 u8SchdTicksCnt = 0;

/* ������Ĵ��� */
void SCHTimeTick(void)
{
    MAX_TASK_T i;
    SCH_Task *pTask;
    if (u8SchdTicksCnt >= SCH_TIM_TO_TICKS_CMP)	{
    u8SchdTicksCnt -= SCH_TIM_TO_TICKS_CMP;
    pTask = pFirstTask;
        for (i = 0; i < TaskNumberSum; i++){
            if ((pTask->TimeCounter != SCH_TASK_PEND) && (pTask->TimeCounter > 0))
                pTask->TimeCounter--;
            //pTask = pTCB->pNextTCB;
        }
    }
}


/* ���񴴽�,����ʱ�������񴴽�ʧ�ܴ����öϵ� */
void SCHTaskCreate(SCH_TCB           *pNewTCB,
                   void              (*pNewTask)(void),
				   SCH_DLY			 DLY_ms)
{
    if (TaskNumberSum == 0) {
        pFirstTCB = pNewTCB;                    //���ݵ�һ��������ƿ��ַ
        pNewTCB->pTask = pNewTask;              //�´�����������ƿ�ĺ���ָ��ָ���´���������
        pCurTCB   = pNewTCB;                    //����������ƿ��ַ����ǰ������ƿ�ָ��
        pCurTCB->pNextTCB = pCurTCB;            //��Ϊֻ��һ������,����ָ�����һ��������ƿ��ַ�����Լ�
        pCurTCB->TimeCounter = DLY_ms;
    }
    else if (TaskNumberSum < SCH_MAX_TASKS)    {
        pNewTCB->pTask = pNewTask;              //�´�����������ƿ�ĺ���ָ��ָ���´���������
        pNewTCB->pNextTCB = pCurTCB->pNextTCB;  //��ǰ������ƿ�ָ�����һ��������ƿ����½���������ƿ���ָ��
        pCurTCB->pNextTCB = pNewTCB;            //��ǰ������ƿ�ָ�����һ��������ƿ����Ϊ�½�������
        pCurTCB = pNewTCB;                      //�½���������ƿ����Ϊ��ǰ������ƿ�
        pCurTCB->TimeCounter = DLY_ms;
    }
    else
        TaskNumberSum--;                        //���񴴽�ʧ��,����ʱ���ڴ˷��öϵ�   

    TaskNumberSum++;
    #if SCH_CFG_Q_EN > 0u
    pNewTCB->pData    = (void *)0;
    pNewTCB->Size = 0;
    #endif
}


void SCHTaskStart(void)
{
    pCurTCB = pFirstTCB;                        //ָ���һ������������,֮�󰴴���ʱ��˳��ִ����ȥ
    while (1) {                                 //��������,����һֱѭ����ȥ
        SCHTimeTick();                            //�������Tick��������,�������ڿ�ִ��״̬
        if (SCH_TASK_RUN == pCurTCB->TimeCounter) {//�����ڿ�ִ��״̬
            pCurTCB->TimeCounter = SCH_TASK_PEND;   //����Ϊ����״̬,��֤����ִֻ��һ��
            pCurTCB->pTask();                       //ִ�е�ǰ������ƿ�ָ�������
            pCurTCB = pFirstTCB;                    //ÿִ����һ������,���ص�������²���һ�ο�ִ��������ȼ�����
        }
        else
            pCurTCB = pCurTCB->pNextTCB;              //ָ����һ��������ƿ�,�����¸������Ƿ��ִ��    
    }
}

////----------------------------------����ָ������,������---------------------------------------------------
////������ǰ���񼰵�����������"Schedule.h"��
//
////������ͣ��ָ������
//void SCHTaskPend(SCH_TCB *pTaskPendTCB)
//{
//  pTaskPendTCB->TimeCounter = SCH_TASK_PEND;
//}
//
////�ָ�ָ���������У�
//void SCHTaskResume(SCH_TCB *pTaskResumeTCB)
//{
//  pTaskResumeTCB->TimeCounter = SCH_TASK_RUN;
//}
//
////ָ��������ʱX��ʱ����ĺ�ָ�
//void SCHTaskDly(SCH_TCB *pTaskDlyTCB, SCH_DLY Ticks)
//{
//  pTaskDlyTCB->TimeCounter = Ticks;
//}

//---------------------------------��Ϣ-----------------------------------------
//�ͷ���Ϣ
/*#if SCH_CFG_Q_EN > 0u

void SCHTaskQpost(SCH_TCB   *pPostTCB,
                  void      *pData,
                  u8 Size)
{
  pPostTCB->pData = pData;
  pPostTCB->Size  = Size;
  pPostTCB->TimeCounter = SCH_TASK_RUN;
}*/

//��ѯ��Ϣ�ж�״̬,�Ƿ�������(����)��æ(������),����SCHTaskQpend()ʱ�Ὣ������Ϊ����״̬
/*u8 SCHTaskGetQFree(SCH_TCB   *pTaskTCB)
{
  if (pTaskTCB->pData == ((void *)0))
  {
    return SCH_Q_FREE;
  }
  else
  {
    return SCH_Q_BUSY;
  }
}


#endif*/

