/*********************************Copyright(c)*********************************
**                     Electrical Technology Co.,Ltd..
**
**                       http://www.***.com
**
**----------------------------File Info----------------------------------------
** File Name:            schedule.c
** Latest modified Date: 2013-05-18
** Description:          调度器相关功力实现函数。
**							 
**-----------------------------------------------------------------------------
** Created By:           QQ 237765062
** Created date:         2013-04-13
** Descriptions:         建立统一通用程序框架
**
**-----------------------------------------------------------------------------
** Modified by:          QQ 237765062 
** Modified date:        2014-04-28
** Description:          去除“SCHTaskSchedStart()”函数中的"goto"语句，用“if else”
**							         逻辑来实现,版本升级为 V1.02
**-----------------------------------------------------------------------------
** Modified by:          QQ 237765062 
** Modified date:        2014-07-11
** Description:          "SCHTaskCreate"函数增加任务初始定时器值,可实现各个任务错开运行,
**							         从而提高效率,版本升级为 V2.0
******************************************************************************/

#include "schedule.h"

SCH_TCB *pFirstTCB, *pCurTCB;

static MAX_TASK_T TaskNumberSum = 0;
u8 u8SchdTicksCnt = 0;

/* 任务节拍处理 */
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


/* 任务创建,调试时可在任务创建失败处放置断点 */
void SCHTaskCreate(SCH_TCB           *pNewTCB,
                   void              (*pNewTask)(void),
				   SCH_DLY			 DLY_ms)
{
    if (TaskNumberSum == 0) {
        pFirstTCB = pNewTCB;                    //备份第一个任务控制块地址
        pNewTCB->pTask = pNewTask;              //新创建的任务控制块的函数指针指向新创建的任务
        pCurTCB   = pNewTCB;                    //最新任务控制块地址给当前任务控制块指针
        pCurTCB->pNextTCB = pCurTCB;            //因为只有一个任务,所以指令的下一个任务控制块地址就是自己
        pCurTCB->TimeCounter = DLY_ms;
    }
    else if (TaskNumberSum < SCH_MAX_TASKS)    {
        pNewTCB->pTask = pNewTask;              //新创建的任务控制块的函数指针指向新创建的任务
        pNewTCB->pNextTCB = pCurTCB->pNextTCB;  //当前任务控制块指向的下一个任务控制块由新建的任务控制块来指向
        pCurTCB->pNextTCB = pNewTCB;            //当前任务控制块指向的下一个任务控制块更新为新建的任务
        pCurTCB = pNewTCB;                      //新建的任务控制块更新为当前任务控制块
        pCurTCB->TimeCounter = DLY_ms;
    }
    else
        TaskNumberSum--;                        //任务创建失败,调试时可在此放置断点   

    TaskNumberSum++;
    #if SCH_CFG_Q_EN > 0u
    pNewTCB->pData    = (void *)0;
    pNewTCB->Size = 0;
    #endif
}


void SCHTaskStart(void)
{
    pCurTCB = pFirstTCB;                        //指向第一个创建的任务,之后按创建时的顺序执行下去
    while (1) {                                 //环形链表,可以一直循环下去
        SCHTimeTick();                            //如果任务Tick满足条件,则将其置于可执行状态
        if (SCH_TASK_RUN == pCurTCB->TimeCounter) {//任务处于可执行状态
            pCurTCB->TimeCounter = SCH_TASK_PEND;   //设置为挂起状态,保证任务只执行一次
            pCurTCB->pTask();                       //执行当前任务控制块指向的任务
            pCurTCB = pFirstTCB;                    //每执行完一个任务,都回到起点重新查找一次可执行最高优先级任务
        }
        else
            pCurTCB = pCurTCB->pNextTCB;              //指向下一个任务控制块,查找下个任务是否可执行    
    }
}

////----------------------------------操作指定任务,不常用---------------------------------------------------
////操作当前任务及调用子任务在"Schedule.h"中
//
////挂起（暂停）指定任务
//void SCHTaskPend(SCH_TCB *pTaskPendTCB)
//{
//  pTaskPendTCB->TimeCounter = SCH_TASK_PEND;
//}
//
////恢复指定任务（运行）
//void SCHTaskResume(SCH_TCB *pTaskResumeTCB)
//{
//  pTaskResumeTCB->TimeCounter = SCH_TASK_RUN;
//}
//
////指定任务延时X个时间节拍后恢复
//void SCHTaskDly(SCH_TCB *pTaskDlyTCB, SCH_DLY Ticks)
//{
//  pTaskDlyTCB->TimeCounter = Ticks;
//}

//---------------------------------消息-----------------------------------------
//释放消息
/*#if SCH_CFG_Q_EN > 0u

void SCHTaskQpost(SCH_TCB   *pPostTCB,
                  void      *pData,
                  u8 Size)
{
  pPostTCB->pData = pData;
  pPostTCB->Size  = Size;
  pPostTCB->TimeCounter = SCH_TASK_RUN;
}*/

//查询消息列队状态,是否是自由(可用)或忙(不可用),调用SCHTaskQpend()时会将其设置为自由状态
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

