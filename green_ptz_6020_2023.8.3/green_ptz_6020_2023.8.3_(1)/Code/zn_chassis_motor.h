/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef ZN_CHASSIS_MOTOR_H
#define ZN_CHASSIS_MOTOR_H

#include "struct_typedef.h"
#include "zn_pid.h"

extern pid_type_def chassis_follow;    
   
float Chassis_Follow(pid_type_def *ptr);
    
    
    
    
    
    
  


#endif