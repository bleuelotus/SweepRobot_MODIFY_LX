/**
  ******************************************************************************
  * @file    Path.h
  * @author  Reason Chen
  * @version V1.0
  * @date    5-May-2015
  * @brief   Robot Path Planning
  ******************************************************************************
  */

#ifndef __PATH_H__
#define __PATH_H__

#include "stm32f10x_conf.h"

enum PathCondtion {

    PATH_COND_OK,
    PATH_FAULT_L,
    PATH_FAULT_HEAD,
    PATH_FAULT_R,
    PATH_FAULT_180,
};

extern enum PathCondtion   PathCond;

void Path_Init(void);
void Path_PlanningStart(void);
void Path_PlanningStop(void);

#endif /* __PATH_H__ */
