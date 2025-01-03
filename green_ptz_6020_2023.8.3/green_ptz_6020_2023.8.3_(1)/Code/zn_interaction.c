#include "main.h"
#include "remote_control.h"
#include "zn_chassis_task.h"
#include "zn_gimbal_task.h"
#include "zn_interaction.h"
#include "zn_pid.h"

ROBOT Robot1;

KeyBoard Key_Q;
KeyBoard Key_W;
KeyBoard Key_E;
KeyBoard Key_R;
KeyBoard Key_A;
KeyBoard Key_S;
KeyBoard Key_D;
KeyBoard Key_F;
KeyBoard Key_G;
KeyBoard Key_Z;
KeyBoard Key_X;
KeyBoard Key_C;
KeyBoard Key_V;
KeyBoard Key_B;
KeyBoard Key_SHIFT;
KeyBoard Key_CTRL;

//WS--Y方向速度
void Key_pressed_WS()
{
  if(DJI_RC.keyBoard.bit.W && !DJI_RC.keyBoard.bit.S)
  {
   Robot1.Movement.Chassis_speed_set.vy = 1; //目标值为当前最大速度
  }
  if(!DJI_RC.keyBoard.bit.W && DJI_RC.keyBoard.bit.S)
  {
   Robot1.Movement.Chassis_speed_set.vy = -1; 
  }
  if(!DJI_RC.keyBoard.bit.W && !DJI_RC.keyBoard.bit.S)
  {
   Robot1.Movement.Chassis_speed_set.vy = 0; 
  }  
//  if(DJI_RC.keyBoard.bit.W && DJI_RC.keyBoard.bit.S)
//  {
//   //如果ws同时按下，保持当前速度不变
//  }  
  
  Robot1.Movement.Chassis_speed_out.vy = RAMP_float(Robot1.Movement.Chassis_speed_set.vy,Robot1.Movement.Chassis_speed_out.vy,Robot1.Movement.ramp_vy);
  
}

//AD--X方向速度
void Key_pressed_AD()
{
  if(DJI_RC.keyBoard.bit.A && !DJI_RC.keyBoard.bit.D)
  {
    Robot1.Movement.Chassis_speed_set.vx = -1;
  }
  if(!DJI_RC.keyBoard.bit.A && DJI_RC.keyBoard.bit.D)
  {
    Robot1.Movement.Chassis_speed_set.vx = 1;
  }
  if(!DJI_RC.keyBoard.bit.A && !DJI_RC.keyBoard.bit.D)
  {
     Robot1.Movement.Chassis_speed_set.vx = 0;
  }  
//  if(DJI_RC.keyBoard.bit.A && DJI_RC.keyBoard.bit.D)
//  {
//   //如果AD同时按下，保持当前速度不变
//  }  
  
  Robot1.Movement.Chassis_speed_out.vx = RAMP_float(Robot1.Movement.Chassis_speed_set.vx,Robot1.Movement.Chassis_speed_out.vx,Robot1.Movement.ramp_vx);
  
}

//QE--角速度自转
void Key_pressed_QE()
{
  if(DJI_RC.keyBoard.bit.Q && !DJI_RC.keyBoard.bit.E)
  {
    Robot1.Movement.Chassis_speed_set.vw = -2;
  }
  if(!DJI_RC.keyBoard.bit.Q && DJI_RC.keyBoard.bit.E)
  {
    Robot1.Movement.Chassis_speed_set.vw = +2;
  }  
  if(!DJI_RC.keyBoard.bit.Q && !DJI_RC.keyBoard.bit.E)
  {
    Robot1.Movement.Chassis_speed_set.vw = 0;
  }    
  
 Robot1.Movement.Chassis_speed_out.vw = RAMP_float(Robot1.Movement.Chassis_speed_set.vw,Robot1.Movement.Chassis_speed_out.vw,Robot1.Movement.ramp_vw); 
  
}

//R--底盘模式切换
void Key_pressed_R()
{ 
  if(DJI_RC.keyBoard.bit.R && Key_R.once)
  {
   //code begin
   Robot1.Status.actChassis ++; //底盘模式切换
   if(Robot1.Status.actChassis>3)Robot1.Status.actChassis=CHASSIS_LOCK;
   //code end
   Key_R.once = 0;
  }
  if(DJI_RC.keyBoard.bit.R == 0)Key_R.once = 1;
  
  
  if(Robot1.Status.actChassis == CHASSIS_GYROSCOPE || Robot1.Status.actChassis == CHASSIS_FOLLOW_GIMBAL)
  {
    Robot1.Status.actGimbal = GIMBAL_MOUSE;//如果底盘为跟随 or 小陀螺，云台自动解除锁定
  }
  
}

//F--云台模式切换
void Key_pressed_F()
{
  if(DJI_RC.keyBoard.bit.F && Key_F.once)
  {
   //code begin
     Robot1.Status.actGimbal ++; //云台模式切换
     if(Robot1.Status.actGimbal>3)Robot1.Status.actGimbal=GIMBAL_LOCK;
   //code end
     Key_F.once = 0;
  }
  if(DJI_RC.keyBoard.bit.F == 0)Key_F.once = 1;
}

//摩擦轮状态切换
void Key_pressed_X()
{
  if(DJI_RC.keyBoard.bit.X && Key_X.once)
  {
   //code begin
   Robot1.Status.rub_flag =!Robot1.Status.rub_flag;
   //code end
   Key_X.once = 0;
  }
  if(DJI_RC.keyBoard.bit.X == 0)Key_X.once = 1;
}

//弹舱开合
void Key_pressed_Z()
{
  if(DJI_RC.keyBoard.bit.Z && Key_Z.once)
  {
   //code begin
   Robot1.Status.lid_flag =!Robot1.Status.lid_flag;
   //code end
   Key_Z.once = 0;
  }
  if(DJI_RC.keyBoard.bit.Z == 0)Key_Z.once = 1;
}

//开超级电容
void Key_pressed_SHIFT()
{
  if(DJI_RC.keyBoard.bit.SHIFT && Key_SHIFT.once)
  {
   //code begin
   Robot1.Status.super_power_flag =!Robot1.Status.super_power_flag;
   //code end
   Key_SHIFT.once = 0;
  }
  if(DJI_RC.keyBoard.bit.SHIFT == 0)Key_SHIFT.once = 1;
}

//打蛋  单发/连发
void Mouse_Left()
{
   static uint16_t time_flag = 0;  
  
  if(DJI_RC.mouse.press_l == 1)
  {
    if(time_flag < 80)
    {     
     Assign_To_M3508_angle(&rotate_motor,Robot1.Movement.pluck_angle); 
    }

   time_flag++;//开始计时
   
      if(time_flag >= 80 && DJI_RC.mouse.press_l == 1)
      {
       Assign_To_Motor_speed(&rotate_motor,-Robot1.Movement.pluck_speed);  
      }
   
  }
    if(DJI_RC.mouse.press_l == 0)
    {
     time_flag = 0;
     rotate_motor.data.total_angle = 0;
     Assign_To_Motor_speed(&rotate_motor,0);
    }
}

void ALL_Key_Process()
{
   Key_pressed_R();
   Key_pressed_F();
   Key_pressed_Z();
   Key_pressed_X();
   Key_pressed_SHIFT();
   
   Key_pressed_WS();
   Key_pressed_AD();
   Key_pressed_QE();

  if(Robot1.Status.rub_flag == 1)   
  {
     Mouse_Left();
  }
   
}

void ROBOT_Para_Init()
{
  Robot1.Mechanical.Radius = 0.21213;
  Robot1.Mechanical.WHEEL_PERIMETER = 0.13;
  Robot1.Mechanical.CHASSIS_DECELE_RATIO = 19;
  
  Robot1.Mechanical.YAW_Motor_Middle = 0;//yaw电机中值
  Robot1.Mechanical.PITCH_Motor_Middle = 5060;
  Robot1.Mechanical.PITCH_Motor_Front  = 5590;//较大值
  Robot1.Mechanical.PITCH_Motor_Behind = 4660;
  
//  Robot1.Movement.Chassis_speed_max.vx= 0;
//  Robot1.Movement.Chassis_speed_max.vy= 0;
//  Robot1.Movement.Chassis_speed_max.vw= 0;
  
  Robot1.Movement.chassis_3508_maxspeed[0] = 20000;
  Robot1.Movement.chassis_3508_maxspeed[1] = 20000;
  Robot1.Movement.chassis_3508_maxspeed[2] = 20000;
  Robot1.Movement.chassis_3508_maxspeed[3] = 20000;
  
  Robot1.Movement.Gimbal_PITCH_motor_angle_set = Robot1.Mechanical.PITCH_Motor_Middle;  
  Robot1.Movement.Gimbal_YAW_motor_angle_set = Robot1.Mechanical.YAW_Motor_Middle;
  
  Robot1.Movement.rub_speed = 8000;//摩擦轮速度
  Robot1.Movement.pluck_angle = 2500;//单发拨弹角度
  Robot1.Movement.pluck_speed =-3000;//连发拨弹速度
  
  Robot1.Movement.ramp_vx = 0.01;//斜坡系数
  Robot1.Movement.ramp_vy = 0.01;
  Robot1.Movement.ramp_vw = 0.005;
  
  Robot1.Movement.gyroscope_speed = 3;//小陀螺速度

  Robot1.Mechanical.GM6020_init_position[0]=2800;
  Robot1.Mechanical.GM6020_init_position[1]=5600;
  Robot1.Mechanical.GM6020_init_position[2]=150;
  Robot1.Mechanical.GM6020_init_position[3]=1320;

}

//遥控器控制前后左右
void RC_control_remove()
{
    //遥控器控制前后
    if(DJI_RC.rc.ch4> RC_Middle_date)Robot1.Movement.Chassis_speed_set.vy= 2;
    if(DJI_RC.rc.ch4<-RC_Middle_date)Robot1.Movement.Chassis_speed_set.vy=-2;
    if(zn_abs(DJI_RC.rc.ch3)<=RC_Middle_date)
    {
      Robot1.Movement.Chassis_speed_set.vx = 0;
    }
    //遥控器左右控制
    if(DJI_RC.rc.ch3<-RC_Middle_date)Robot1.Movement.Chassis_speed_set.vx=-2;
    if(DJI_RC.rc.ch3> RC_Middle_date)Robot1.Movement.Chassis_speed_set.vx= 2;
    if(zn_abs(DJI_RC.rc.ch4)<=RC_Middle_date)
    {
      Robot1.Movement.Chassis_speed_set.vy = 0;
    }
    
    Robot1.Movement.Chassis_speed_out.vy = RAMP_float(Robot1.Movement.Chassis_speed_set.vy,Robot1.Movement.Chassis_speed_out.vy,Robot1.Movement.ramp_vy);
    Robot1.Movement.Chassis_speed_out.vx = RAMP_float(Robot1.Movement.Chassis_speed_set.vx,Robot1.Movement.Chassis_speed_out.vx,Robot1.Movement.ramp_vx);
    Robot1.Movement.Chassis_speed_out.vw = RAMP_float(Robot1.Movement.Chassis_speed_set.vw,Robot1.Movement.Chassis_speed_out.vw,Robot1.Movement.ramp_vw);
}

//遥控器控制打弹  单发/连发
void RC_control_shot()
{
   static uint16_t time_flag = 0;  
  
  if(DJI_RC.rc.ch5 >= RC_Middle_date)
  {
    if(time_flag < 80)
    {
     Assign_To_M3508_angle(&rotate_motor,Robot1.Movement.pluck_angle); 
    }

   time_flag++;//开始计时
   
      if(time_flag >= 80 && DJI_RC.rc.ch5 >= RC_Middle_date)
      {
       Assign_To_Motor_speed(&rotate_motor,-Robot1.Movement.pluck_speed);  
      }
   
  }
    if(DJI_RC.rc.ch5 == 0)
    {
     time_flag = 0;
     rotate_motor.data.total_angle = 0;
     Assign_To_Motor_speed(&rotate_motor,0);
    }
}

/*遥控器总任务，当遥控器为右上（GPS）时进入键鼠控制，
     即函数：ALL_Key_Process（）；已经放入该函数里*/
void Remote_TASK_ALL(SWIT ROBOT_MODE,SWIT SHOT_MODE)
{ 
  //右上：键鼠控制移动
  if(SHOT_MODE ==SWITCH_UP)ALL_Key_Process();
  //右中&右下：遥控器控制移动
  if(SHOT_MODE !=SWITCH_UP)
  {
    RC_control_remove();
    
      //左上：云台不动,底盘自由运动
    if(ROBOT_MODE==SWITCH_UP)
    {
      Robot1.Status.actChassis = CHASSIS_SEPARATE;
      Robot1.Status.actGimbal  = GIMBAL_LOCK;
    }
  
    //左中：云台随动,底盘跟随云台
    if(ROBOT_MODE==SWITCH_MIDDLE)
    {
      Robot1.Status.actChassis = CHASSIS_FOLLOW_GIMBAL;
      Robot1.Status.actGimbal  = GIMBAL_MOUSE;
    }
    
      //左下：云台自由,底盘小陀螺
    if(ROBOT_MODE==SWITCH_DOWN)
    {
      Robot1.Status.actChassis = CHASSIS_GYROSCOPE;
      Robot1.Status.actGimbal  = GIMBAL_MOUSE;
    }
      //右中：关闭摩擦轮
    if(SHOT_MODE ==SWITCH_MIDDLE)Robot1.Status.rub_flag = 0;
    //右下：开启摩擦轮
    if(SHOT_MODE ==SWITCH_DOWN)
    {
      Robot1.Status.rub_flag = 1;
      RC_control_shot();
    }
  }

}