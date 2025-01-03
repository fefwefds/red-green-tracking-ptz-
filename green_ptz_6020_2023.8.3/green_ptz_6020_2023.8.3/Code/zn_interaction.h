#ifndef ZN_INTERACTION_H
#define ZN_INTERACTION_H

#include "zn_chassis_task.h"
#include "zn_gimbal_task.h"//(��ѧ������ϵ)

#define RC_Middle_date 60

typedef struct
{
  float Radius;                      //���ֵ��������ľ�
  float WHEEL_PERIMETER;             //�����־�
  float CHASSIS_DECELE_RATIO;       //���������
    //��ƫ��
  int16_t YAW_Motor_Middle;           //yaw�����λ�Ƕ�ֵ

  int16_t PITCH_Motor_Middle;         //PITCH�����λ�Ƕ�ֵ  
  int16_t PITCH_Motor_Front;          //PITCH�����󸩽�
  int16_t PITCH_Motor_Behind;         //PITCH����������
  
  int16_t GM6020_init_position[4];    //��������ʼ�Ƕ�
  
} Mechanical_Para;//��е����

typedef struct
{  
  //����
  Chassis_Speed Chassis_speed_set;               //�����趨�ٶ�
  Chassis_Speed Chassis_speed_max;               //�����ٶ��޷�
  Chassis_Speed Chassis_speed_out;               //б�����
  double ramp_vx;//б�����ϵ��
  double ramp_vy;
  double ramp_vw;
  
  fp32 chassis_6020_setangle[4];		//�ĸ�������Ŀ��Ƕ�
  
  float gyroscope_speed;//С�����ٶ�
  
  int16_t chassis_3508_setspeed[4];		//�ĸ�3508����Ŀ��ת��
  int16_t chassis_3508_maxspeed[4];               //���ת���޷�
  //��̨
  float Gimbal_YAW_motor_angle_set;             //��̨yaw����Ƕ��趨ֵ
  float Gimbal_YAW_imu_angle_set;               //��̨yaw�����ǽǶ��趨ֵ  
  float Gimbal_PITCH_motor_angle_set;           //��̨pitch����Ƕ��趨ֵ
  float Gimbal_PITCH_imu_angle_set;             //��̨pitch�����ǽǶ��趨ֵ
  //����
  int16_t pluck_speed;                        //������ת�� (����ʱ)
  int16_t pluck_angle;                       //�������趨�Ƕȣ�����ʱ��
  int16_t rub_speed;                          //Ħ����ת��
   
} Movement_Data;//�˶�����

typedef struct
{
  eChassisAction actChassis;//����״̬
  eGimbalAction actGimbal;//��̨״̬

  int16_t lid_flag;//���ո�״̬
  int16_t rub_flag;//Ħ����״̬
  int16_t pluck_flag;//����״̬
  int16_t collimation_flag;//����״̬
  int16_t super_power_flag;//��������״̬
  
} Status_Flag;

 typedef struct
{
  Mechanical_Para Mechanical;//��е����
  Movement_Data Movement;//�˶�����
  Status_Flag Status;//״̬��־
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
  ABCDEFG,//ռλ��
  SWITCH_UP,//��
  SWITCH_DOWN,//��
  SWITCH_MIDDLE//��
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