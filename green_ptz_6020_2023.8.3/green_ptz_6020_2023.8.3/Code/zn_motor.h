#ifndef ZN_MOTOR_H
#define ZN_MOTOR_H

#include "main.h"
#include "struct_typedef.h"
#include "zn_pid.h"

typedef enum
{
    M3508 = 0,
    M6020	
}motor_kind;

typedef enum 
{
    ZN_CAN1 = 0,
    ZN_CAN2
}motor_CAN;

typedef struct
{
  motor_kind KIND;   //�������
  motor_CAN CAN;     //CAN
  u8 ID;             //ID
  u16 control_StdId; //���Ʊ�ʶ
  u16 back_StdId;    //������ʶ
  
} Motor_message;

/*���յ��ĵ���Ĳ����ṹ��*/
typedef struct{
    int16_t       speed_rpm;
    float  	  real_current;
    int16_t  	  given_current;
    uint16_t 	  angle;		 
    uint16_t 	  last_angle;	
    float         angle_old;
    float       total_angle;
}Motor_measure;

typedef struct
{
  Motor_message mes;           //����趨��Ϣ
  Motor_measure data;          //�����������
  pid_type_def speed_pid;      //�ٶȻ�pid����
  pid_type_def angle_pid;      //�ǶȻ�pid����
  
} Motor_type_def;

void Motor_All_Para_Init();//������в�����ʼ��

void Assign_To_Motor_speed(Motor_type_def *ptr,int16_t speed);//����ٶ��趨
void Assign_To_M6020_angle(Motor_type_def *ptr,int16_t angle);//6020����Ƕ��趨
float M3508_angle_cal_angle(Motor_type_def *ptr,int16_t angle);
void Assign_To_M3508_angle(Motor_type_def *ptr,int16_t angle);//3508����Ƕ��趨

void Motor_message_Init(Motor_type_def *ptr);//�����Ϣ��ʼ��

float Motor_speed_cal(Motor_type_def *ptr,int16_t speed);//����ٶȻ�����
float M6020_angle_cal(Motor_type_def *ptr,int16_t angle);//6020����ǶȻ�����
float M3508_angle_cal(Motor_type_def *ptr,int16_t angle);//3508����ǶȻ�����

void total_angle_get(Motor_type_def *ptr);//���3508���total_angle

extern Motor_type_def yuntai_yaw;
extern Motor_type_def yuntai_pitch;

extern Motor_type_def yaw_motor;     //YAW��6020
extern Motor_type_def pitch_motor;  //Pitch��6020

extern Motor_type_def rotate_motor; //����
extern Motor_type_def rub_motor_L;  //Ħ����
extern Motor_type_def rub_motor_R;

extern Motor_type_def  rudder_FL;//��ǰ
extern Motor_type_def  rudder_FR;//��ǰ
extern Motor_type_def  rudder_BL;//���
extern Motor_type_def  rudder_BR;//�Һ�

extern Motor_type_def  drive_FL;//��ǰ
extern Motor_type_def  drive_FR;//��ǰ
extern Motor_type_def  drive_BL;//���
extern Motor_type_def  drive_BR;//�Һ�
#endif