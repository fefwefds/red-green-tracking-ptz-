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
  motor_kind KIND;   //电机类型
  motor_CAN CAN;     //CAN
  u8 ID;             //ID
  u16 control_StdId; //控制标识
  u16 back_StdId;    //反馈标识
  
} Motor_message;

/*接收到的电机的参数结构体*/
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
  Motor_message mes;           //电机设定信息
  Motor_measure data;          //电机返回数据
  pid_type_def speed_pid;      //速度环pid参数
  pid_type_def angle_pid;      //角度环pid参数
  
} Motor_type_def;

void Motor_All_Para_Init();//电机所有参数初始化

void Assign_To_Motor_speed(Motor_type_def *ptr,int16_t speed);//电机速度设定
void Assign_To_M6020_angle(Motor_type_def *ptr,int16_t angle);//6020电机角度设定
float M3508_angle_cal_angle(Motor_type_def *ptr,int16_t angle);
void Assign_To_M3508_angle(Motor_type_def *ptr,int16_t angle);//3508电机角度设定

void Motor_message_Init(Motor_type_def *ptr);//电机信息初始化

float Motor_speed_cal(Motor_type_def *ptr,int16_t speed);//电机速度环计算
float M6020_angle_cal(Motor_type_def *ptr,int16_t angle);//6020电机角度环计算
float M3508_angle_cal(Motor_type_def *ptr,int16_t angle);//3508电机角度环计算

void total_angle_get(Motor_type_def *ptr);//获得3508电机total_angle

extern Motor_type_def yuntai_yaw;
extern Motor_type_def yuntai_pitch;

extern Motor_type_def yaw_motor;     //YAW轴6020
extern Motor_type_def pitch_motor;  //Pitch轴6020

extern Motor_type_def rotate_motor; //拨弹
extern Motor_type_def rub_motor_L;  //摩擦轮
extern Motor_type_def rub_motor_R;

extern Motor_type_def  rudder_FL;//左前
extern Motor_type_def  rudder_FR;//右前
extern Motor_type_def  rudder_BL;//左后
extern Motor_type_def  rudder_BR;//右后

extern Motor_type_def  drive_FL;//左前
extern Motor_type_def  drive_FR;//右前
extern Motor_type_def  drive_BL;//左后
extern Motor_type_def  drive_BR;//右后
#endif