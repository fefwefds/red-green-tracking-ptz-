#ifndef ZN_GIMBAL_TASK_H
#define ZN_GIMBAL_TASK_H

//��̨�����ֵ
//#define  YAW_MOTOR_MIDDLE  2730
//#define  PITCH_MOTOR_MIDDLE  3600

//����״̬
typedef enum
{
    GIMBAL_LOCK   ,  //��̨����    ���������ֵ����
    GIMBAL_MOUSE  ,  //�������    yaw�����ǽǶȿ���
    GIMBAL_VISION ,  //�Ӿ�����
} eGimbalAction;


void ZN_gimbal_task(void const *pvParameters);

void gimbal_motor_init();

void Gimbal_Mode_Change(eGimbalAction mode);

int gimbal_control_remote();

void gimbal_control_vision();

#endif