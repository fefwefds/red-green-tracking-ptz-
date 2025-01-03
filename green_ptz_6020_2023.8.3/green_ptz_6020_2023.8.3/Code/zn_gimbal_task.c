/**
  ****************************(C) 智 能 引 擎 战 队 ****************************
  * @file       zn_gimbal_task.c/h
  * @brief      云台控制任务
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
#include "zn_gimbal_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "ANO_DT.h"
#include "zn_gimbal_motor.h"
#include "bsp_laser.h"
#include "remote_control.h"
#include "zn_interaction.h"
#include "INS_task.h"
#include "zn_vision_task.h"

void ZN_gimbal_task(void const *pvParameters)
{
  static portTickType currentTime;

   gimbal_motor_init();
   
   osDelay(1000);
   
  while(1)
  {
   currentTime = xTaskGetTickCount();	//获取当前系统时间
    /*code begin*/
  
   Gimbal_Mode_Change(Robot1.Status.actGimbal);
   
   laser_on();

  /*code end*/
  vTaskDelayUntil(&currentTime, 2);//绝对延时 ms
  }
}

void gimbal_motor_init()
{
  YAW_imu_angle.kp = 5;
  YAW_imu_angle.ki = 0;
  YAW_imu_angle.kd = 0;
  
  YAW_imu_speed.kp = 100;
  YAW_imu_speed.ki = 0.0001;
  YAW_imu_speed.kd = 2;
  
  YAW_vision.kp = 0;
  YAW_vision.ki = 0;
  YAW_vision.kd = 0;
  
  Pitch_vision.kp =0;
  Pitch_vision.ki =0;
  Pitch_vision.kd =0;
}
/**
  * @brief  切换云台控制模式
  * @param  模式切换标志位
  * @retval none
  * @attention
  */
void Gimbal_Mode_Change(eGimbalAction mode)
{
  switch(mode)
  {
  case  GIMBAL_LOCK:
         Assign_To_M6020_angle(&pitch_motor,(int16_t)Robot1.Movement.Gimbal_PITCH_motor_angle_set);
         Assign_To_M6020_angle(&yaw_motor,(int16_t)Robot1.Movement.Gimbal_YAW_motor_angle_set);   
         break;
  
  case GIMBAL_MOUSE:
         gimbal_control_remote();
         Assign_To_M6020_angle(&pitch_motor,(int16_t)Robot1.Movement.Gimbal_PITCH_motor_angle_set);
         YAW_Angle_Set(&yaw_motor,&YAW_imu_angle,&YAW_imu_speed,Robot1.Movement.Gimbal_YAW_imu_angle_set);       
     
         Robot1.Movement.Gimbal_YAW_motor_angle_set = yaw_motor.data.angle;//衔接
         break;
         
  case GIMBAL_VISION:
//         gimbal_control_vision();
//         Assign_To_M6020_angle(&pitch_motor,(int16_t)Robot1.Movement.Gimbal_PITCH_motor_angle_set);
//         Assign_To_M6020_angle(&yaw_motor,(int16_t)Robot1.Movement.Gimbal_YAW_motor_angle_set);
         break;
         
    default:
        break;          
  }

}

/***************遥控器改变云台角度****************/

float gimbal_IMU_Kp_rc = -0.00015;
float gimbal_Pitch_Kp_rc = 0.02;

float gimbal_IMU_Kp_key = -0.006;
float gimbal_Pitch_Kp_key = -0.2;

#define contorl_MODE 1

int gimbal_control_remote()
{
    if(contorl_MODE == 1)
    {
          LimitDeadBand(DJI_RC.rc.ch1,5);
          LimitDeadBand(DJI_RC.rc.ch2,5);
          LimitDeadBand(DJI_RC.rc.ch3,5);
          LimitDeadBand(DJI_RC.rc.ch4,5);
          LimitDeadBand(DJI_RC.rc.ch5,5);

          Robot1.Movement.Gimbal_YAW_imu_angle_set += gimbal_IMU_Kp_rc*DJI_RC.rc.ch1;   
          Robot1.Movement.Gimbal_PITCH_motor_angle_set += gimbal_Pitch_Kp_rc*DJI_RC.rc.ch2;

    }
    else
    {
          Robot1.Movement.Gimbal_YAW_imu_angle_set += gimbal_IMU_Kp_key*DJI_RC.mouse.x;   
          Robot1.Movement.Gimbal_PITCH_motor_angle_set += gimbal_Pitch_Kp_key*DJI_RC.mouse.y;    
    }
    
    //此处注意front和behind的大小关系
    Limit(Robot1.Movement.Gimbal_PITCH_motor_angle_set,Robot1.Mechanical.PITCH_Motor_Front,Robot1.Mechanical.PITCH_Motor_Behind);
    
    if(Robot1.Movement.Gimbal_YAW_imu_angle_set>=179)//360°处理
    {
        Robot1.Movement.Gimbal_YAW_imu_angle_set=-179;
    }
    else if(Robot1.Movement.Gimbal_YAW_imu_angle_set<=-179)//360°处理
    {
        Robot1.Movement.Gimbal_YAW_imu_angle_set=179;
    }    
    
      return 0;
}

void gimbal_control_vision()
{
  Robot1.Movement.Gimbal_YAW_motor_angle_set=
  Robot1.Movement.Gimbal_YAW_motor_angle_set+
  vision_PID(&YAW_vision,Robot1.Movement.Gimbal_YAW_motor_angle_set+RE_Message.pitch,Robot1.Movement.Gimbal_YAW_motor_angle_set);
  
  Robot1.Movement.Gimbal_PITCH_motor_angle_set=
  Robot1.Movement.Gimbal_PITCH_motor_angle_set+
  vision_PID(&Pitch_vision,Robot1.Movement.Gimbal_PITCH_motor_angle_set+RE_Message.yaw,Robot1.Movement.Gimbal_PITCH_motor_angle_set);
  Limit(Robot1.Movement.Gimbal_PITCH_motor_angle_set,Robot1.Mechanical.PITCH_Motor_Front,Robot1.Mechanical.PITCH_Motor_Behind);
}