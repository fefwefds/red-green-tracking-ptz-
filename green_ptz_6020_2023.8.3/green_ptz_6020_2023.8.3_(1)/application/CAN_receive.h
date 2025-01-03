/**
  ****************************(C) �� �� �� �� ս �� ****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date                Author                        Modification
  *  V1.0.0     Jan-7-2022         Intelligent Engine             1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) Intelligent Engine ****************************
**/

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "zn_motor.h"

typedef struct
{
  uint8_t rxData[8];
  uint8_t txData[3][8];//�ֱ���0X200   0X1FF  0X2FF �ķ�������
  
} Data;//������ݻ�����

typedef struct
{
  int16_t speed[4];
  double angle[4];
  
}Communication;

extern Data Data_can1;
extern Data Data_can2;
extern Communication can_message;

void get_moto_measure(Motor_type_def *ptr);
void copy_can_data(Motor_type_def *ptr,Data *dat);

void CAN_Set_Current(Motor_type_def *ptr,int16_t current);

void can_message_sent(motor_CAN CAN,int STDID,int16_t mes1, int16_t mes2, int16_t mes3, int16_t mes4);

void can_communication_tx(uint8_t mes[8]);
void can_communication_rx(Communication *ptr,Data *dat);

#endif
