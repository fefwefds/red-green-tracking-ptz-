#ifndef ZN_CHASSIS_TASK_H
#define ZN_CHASSIS_TASK_H

#include "cmsis_os.h"
#include "main.h"
#include "zn_motor.h"
#include "CAN_receive.h"
#include "can.h"

#define FL_num 0
#define FR_num 1
#define BL_num 2
#define BR_num 3

#define PI 3.1415926f         

//
extern int yaw_angle;
extern int pitch_angle;

typedef struct{
	int loc_1_x;
        int loc_1_y;
	int loc_2_x;
	int loc_2_y;
        int loc_3_x;
        int loc_3_y;
        
}Point;


//
typedef struct
{
    int dir;
    int value;
    int out_put_P; 
    int out_put_Y; 
    int pitch_set;
    int yaw_set;
    int pitch_mid;
    int yaw_mid;
    int dis;
    int P;
    int D;
}yuntai_;

typedef struct
{
    int step1;
    int step2;
    int step3;
    int step4;
    int step5;
    int step6;
    int step7;
    int flag;
}task1_;

typedef enum
{
    CHASSIS_LOCK          ,  //底盘锁定  
    CHASSIS_SEPARATE      , //云台底盘独立控制
    CHASSIS_FOLLOW_GIMBAL , //底盘跟随云台
    CHASSIS_GYROSCOPE     , //小陀螺模式    

}eChassisAction;

typedef struct
{
    float vx;
    float vy;
    float vw;
}Chassis_Speed;

/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void ZN_chassis_task(void const *pvParameters);

void mecanum_calc(Chassis_Speed *speed, int16_t *out_speed);
void Absolute_Cal(Chassis_Speed* absolute_speed, float angle )	;

void Chassis_Mode_Change(eChassisAction mode);
int chassis_control_remote_KEY();
void CHASSIS_Single_Loop_Out(void); //底盘电机输出

void AGV_calc(Chassis_Speed *speed, int16_t* out_speed) ;
void AGV_angle_calc(Chassis_Speed *speed, fp32* out_angle) ;

void AngleLoop_f (float* angle ,float max);
void AngleLoop_int (int16_t* angle ,int16_t max);
fp32 Find_min_Angle(int16_t angle1,fp32 angle2);

void serial_port_protocol(unsigned char data);
void location_caculate();
float led_gre_flow(float set_distance,float now_distance);
float led_gre_flow_one(float set_distance,float now_distance);
void key_read_pin();


#endif