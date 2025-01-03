#ifndef ZN_INTERACTION_H
#define ZN_INTERACTION_H

#include "zn_chassis_task.h"
#include "zn_gimbal_task.h"//(玄学包含关系)

#define RC_Middle_date 60

typedef struct
{
  float Radius;                      //舵轮底盘轮中心距
  float WHEEL_PERIMETER;             //舵轮轮径
  float CHASSIS_DECELE_RATIO;       //电机减数比
    //无偏移
  int16_t YAW_Motor_Middle;           //yaw电机中位角度值

  int16_t PITCH_Motor_Middle;         //PITCH电机中位角度值  
  int16_t PITCH_Motor_Front;          //PITCH电机最大俯角
  int16_t PITCH_Motor_Behind;         //PITCH电机最大仰角
  
  int16_t GM6020_init_position[4];    //舵向电机初始角度
  
} Mechanical_Para;//机械参数

typedef struct
{  
  //底盘
  Chassis_Speed Chassis_speed_set;               //底盘设定速度
  Chassis_Speed Chassis_speed_max;               //底盘速度限幅
  Chassis_Speed Chassis_speed_out;               //斜坡输出
  double ramp_vx;//斜坡输出系数
  double ramp_vy;
  double ramp_vw;
  
  fp32 chassis_6020_setangle[4];		//四个舵向轮目标角度
  
  float gyroscope_speed;//小陀螺速度
  
  int16_t chassis_3508_setspeed[4];		//四个3508轮子目标转速
  int16_t chassis_3508_maxspeed[4];               //电机转速限幅
  //云台
  float Gimbal_YAW_motor_angle_set;             //云台yaw电机角度设定值
  float Gimbal_YAW_imu_angle_set;               //云台yaw陀螺仪角度设定值  
  float Gimbal_PITCH_motor_angle_set;           //云台pitch电机角度设定值
  float Gimbal_PITCH_imu_angle_set;             //云台pitch陀螺仪角度设定值
  //发射
  int16_t pluck_speed;                        //拨弹盘转速 (连发时)
  int16_t pluck_angle;                       //拨弹盘设定角度（单发时）
  int16_t rub_speed;                          //摩擦轮转速
   
} Movement_Data;//运动数据

typedef struct
{
  eChassisAction actChassis;//底盘状态
  eGimbalAction actGimbal;//云台状态

  int16_t lid_flag;//弹舱盖状态
  int16_t rub_flag;//摩擦轮状态
  int16_t pluck_flag;//拨弹状态
  int16_t collimation_flag;//自瞄状态
  int16_t super_power_flag;//超级电容状态
  
} Status_Flag;

 typedef struct
{
  Mechanical_Para Mechanical;//机械参数
  Movement_Data Movement;//运动数据
  Status_Flag Status;//状态标志
} ROBOT;


extern ROBOT Robot1;

typedef struct
{
  int16_t once;
  int16_t count;
  int16_t flag; 
} KeyBoard;

typedef enum
{
  ABCDEFG,//占位用
  SWITCH_UP,//上
  SWITCH_DOWN,//下
  SWITCH_MIDDLE//中
}SWIT;

/********************/
void Key_pressed_Q();
void Key_pressed_W();
void Key_pressed_E();
void Key_pressed_R();
void Key_pressed_A();
void Key_pressed_S();
void Key_pressed_D();
void Key_pressed_F();
void Key_pressed_G();
void Key_pressed_Z();
void Key_pressed_X();
void Key_pressed_C();
void Key_pressed_V();
void Key_pressed_B();
void Key_pressed_SHIFT();
void Key_pressed_CTRL();

void ALL_Key_Process();

void ROBOT_Para_Init();

void RC_control_remove();
void RC_control_shot();
void Remote_TASK_ALL(SWIT ROBOT_MODE,SWIT SHOT_MODE);
#endif