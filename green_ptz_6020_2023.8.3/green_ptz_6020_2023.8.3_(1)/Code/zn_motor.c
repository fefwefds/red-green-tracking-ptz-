#include "zn_motor.h"
#include "CAN_receive.h"
Motor_type_def yuntai_yaw;
Motor_type_def yuntai_pitch;

Motor_type_def yaw_motor;//云台
Motor_type_def pitch_motor;

Motor_type_def  rudder_FL;//左前
Motor_type_def  rudder_FR;//右前
Motor_type_def  rudder_BL;//左后
Motor_type_def  rudder_BR;//右后

Motor_type_def  drive_FL;//左前
Motor_type_def  drive_FR;//右前
Motor_type_def  drive_BL;//左后
Motor_type_def  drive_BR;//右后

Motor_type_def  rotate_motor;//拨弹
Motor_type_def  rub_motor_L;//摩擦轮
Motor_type_def  rub_motor_R;

//电机所有参数初始化
void Motor_All_Para_Init()
{ 
  yuntai_yaw.mes.CAN = ZN_CAN1;
  yuntai_yaw.mes.KIND = M6020;
  yuntai_yaw.mes.ID = 1;
  yuntai_yaw.speed_pid.kp = 10;
  yuntai_yaw.speed_pid.ki = 0.001;  
  yuntai_yaw.speed_pid.kd = 0.5; 
  yuntai_yaw.angle_pid.kp = 5;

  yuntai_pitch.mes.CAN = ZN_CAN1;
  yuntai_pitch.mes.KIND = M6020;
  yuntai_pitch.mes.ID = 2;
  yuntai_pitch.speed_pid.kp = 10;
  yuntai_pitch.speed_pid.ki = 0.001;  
  yuntai_pitch.speed_pid.kd = 0.5; 
  yuntai_pitch.angle_pid.kp = 5;
  
  Motor_message_Init(&yuntai_yaw);
  Motor_message_Init(&yuntai_pitch);
  
// //yaw轴
//  yaw_motor.mes.CAN = ZN_CAN1;
//  yaw_motor.mes.KIND = M6020;
//  yaw_motor.mes.ID = 1;
//  yaw_motor.speed_pid.kp = 0;
//  yaw_motor.speed_pid.ki = 0.0;  
//  yaw_motor.speed_pid.kd = 0; 
//  yaw_motor.angle_pid.kp = 0.0;
  
//  pid_paragram_init(&yaw_motor.speed_pid,350,0.1,30);
//  pid_paragram_init(&yaw_motor.angle_pid,0.1,0,0);

//  pitch轴
//  pitch_motor.mes.CAN = ZN_CAN1;
//  pitch_motor.mes.KIND = M3508;
//  pitch_motor.mes.ID = 2;
//  pitch_motor.speed_pid.kp = 0;
//  pitch_motor.speed_pid.ki = 0.0;
//  pitch_motor.speed_pid.kd = 0; 
//  pitch_motor.angle_pid.kp = 0.0;
//  pitch_motor.angle_pid.ki = 0.0;
  
//  //前左
//  rudder_FL.mes.CAN = ZN_CAN1;
//  rudder_FL.mes.KIND = M6020;
//  rudder_FL.mes.ID = 1;
//  rudder_FL.speed_pid.kp = 49;
//  rudder_FL.speed_pid.ki = 0.0;  
//  rudder_FL.speed_pid.kd = 0; 
//  rudder_FL.angle_pid.kp = 1.9;//0.5
//  //前右
//  rudder_FR.mes.CAN = ZN_CAN1;
//  rudder_FR.mes.KIND = M6020;
//  rudder_FR.mes.ID = 2;
//  rudder_FR.speed_pid.kp = 40;
//  rudder_FR.speed_pid.ki = 0.0;  
//  rudder_FR.speed_pid.kd = 0; 
//  rudder_FR.angle_pid.kp = 4;  
  //后左
//  rudder_BL.mes.CAN = ZN_CAN1;
//  rudder_BL.mes.KIND = M6020;
//  rudder_BL.mes.ID = 3;
//  rudder_BL.speed_pid.kp = 200;
//  rudder_BL.speed_pid.ki = 0.0;  
//  rudder_BL.speed_pid.kd = 30; 
//  rudder_BL.angle_pid.kp = 0.7;    
//  //后右
//  rudder_BR.mes.CAN = ZN_CAN1;
//  rudder_BR.mes.KIND = M6020;
//  rudder_BR.mes.ID = 4;
//  rudder_BR.speed_pid.kp = 200;
//  rudder_BR.speed_pid.ki = 0.0;  
//  rudder_BR.speed_pid.kd = 30; 
//  rudder_BR.angle_pid.kp = 0.7;      
//
//  //左前
//  drive_FL.mes.CAN = ZN_CAN2;
//  drive_FL.mes.KIND = M3508;
//  drive_FL.mes.ID = 1;
//  drive_FL.speed_pid.kp = 1.5;//3
//  drive_FL.speed_pid.ki = 0.5;//0.5
//  drive_FL.angle_pid.kp = 2.5;//3
//  drive_FL.angle_pid.ki = 0.01;//0.01    
//  //右前  
//  drive_FR.mes.CAN = ZN_CAN2;
//  drive_FR.mes.KIND = M3508;
//  drive_FR.mes.ID = 2;
//  drive_FR.speed_pid.kp = 4;
//  drive_FR.speed_pid.ki = 0.5;
//  drive_FR.angle_pid.kp = 3;
//  drive_FR.angle_pid.ki = 0.01; 
//  //左后 
//  drive_BL.mes.CAN = ZN_CAN2;
//  drive_BL.mes.KIND = M3508;
//  drive_BL.mes.ID = 3;
//  drive_BL.speed_pid.kp = 1.5;
//  drive_BL.speed_pid.ki = 0.5;
//  drive_BL.angle_pid.kp = 2.5;
//  drive_BL.angle_pid.ki = 0.01; 
//  //右后 
//  drive_BR.mes.CAN = ZN_CAN2;
//  drive_BR.mes.KIND = M3508;
//  drive_BR.mes.ID = 4;
//  drive_BR.speed_pid.kp = 4;
//  drive_BR.speed_pid.ki = 0.5;
//  drive_BR.angle_pid.kp = 3;
//  drive_BR.angle_pid.ki = 0.01;
  
  //拨弹
//  rotate_motor.mes.CAN      = ZN_CAN2;
//  rotate_motor.mes.KIND     = M3508;
//  rotate_motor.mes.ID       = 1;
//  rotate_motor.speed_pid.kp = 10;
//  rotate_motor.speed_pid.ki = 0;
//  rotate_motor.angle_pid.kp = 1.5;
//  rotate_motor.angle_pid.ki = 0;
//  
//  //摩擦轮
//  rub_motor_L.mes.CAN      = ZN_CAN2;
//  rub_motor_L.mes.KIND     = M3508;
//  rub_motor_L.mes.ID       = 2;
//  rub_motor_L.speed_pid.kp = 1;
//  rub_motor_L.speed_pid.ki = 0.05;
//  rub_motor_L.angle_pid.kp = 0.8;
//  rub_motor_L.angle_pid.ki = 0.001; 
//  
//  rub_motor_R.mes.CAN      = ZN_CAN2;
//  rub_motor_R.mes.KIND     = M3508;
//  rub_motor_R.mes.ID       = 3;
//  rub_motor_R.speed_pid.kp = 1;
//  rub_motor_R.speed_pid.ki = 0.05;
//  rub_motor_R.angle_pid.kp = 0.8;
//  rub_motor_R.angle_pid.ki = 0.001;
  
//  Motor_message_Init(&yaw_motor);
//  Motor_message_Init(&pitch_motor);
  
//  Motor_message_Init(&rudder_FL);
//  Motor_message_Init(&rudder_FR);
//  Motor_message_Init(&rudder_BL);
//  Motor_message_Init(&rudder_BR);
//  
//  Motor_message_Init(&drive_FL);
//  Motor_message_Init(&drive_FR);
//  Motor_message_Init(&drive_BL);
//  Motor_message_Init(&drive_BR);  
  
//  Motor_message_Init(&rotate_motor);
//  Motor_message_Init(&rub_motor_L);
//  Motor_message_Init(&rub_motor_R);
}

//电机速度设定 3508 or 6020 or 2006
void Assign_To_Motor_speed(Motor_type_def *ptr,int16_t speed)
{
  Motor_speed_cal(ptr,speed);//速度环计算
  CAN_Set_Current(ptr,(int16_t)ptr->speed_pid.All_out);//电流输出
}

//6020角度设定
void Assign_To_M6020_angle(Motor_type_def *ptr,int16_t angle)
{
  M6020_angle_cal(ptr,angle);//角度环计算 
  Motor_speed_cal(ptr,(int16_t)ptr->angle_pid.All_out);//速度环计算
  CAN_Set_Current(ptr,(int16_t)ptr->speed_pid.All_out);//电流输出
}

//3508&2006角度设定
void Assign_To_M3508_angle(Motor_type_def *ptr,int16_t angle)
{
  M3508_angle_cal_angle(ptr,angle);//角度环计算 
  Motor_speed_cal(ptr,(int16_t)ptr->angle_pid.All_out);//速度环计算
  CAN_Set_Current(ptr,(int16_t)ptr->speed_pid.All_out);//电流输出
}

//电机信息初始化
void Motor_message_Init(Motor_type_def *ptr)
{
    if(ptr->mes.KIND == M3508)
    {
      if(ptr->mes.ID<=4)
      {
        ptr->mes.control_StdId = 0X200;
      }   
     else if(ptr->mes.ID>4)
      {
        ptr->mes.control_StdId = 0X1FF;
      }
     ptr->mes.back_StdId = 0X200 + ptr->mes.ID;
    }
    
    if(ptr->mes.KIND == M6020)
    {
      if(ptr->mes.ID<=4)
      {
      ptr->mes.control_StdId = 0X1FF;
      }  
      if(ptr->mes.ID>4)
      {
      ptr->mes.control_StdId = 0X2FF;
      }
      ptr->mes.back_StdId = 0X204 + ptr->mes.ID;
    }
}

//各种电机速度环计算方式相同，仅修改参数即可
float Motor_speed_cal(Motor_type_def *ptr,int16_t speed)
{
    ptr->speed_pid.error = speed - ptr->data.speed_rpm;
    ptr->speed_pid.error_all += ptr->speed_pid.error;
    
    //积分限幅
  //  Limit(ptr->speed_pid.error_all,ptr->speed_pid.IntegralLimit,-ptr->speed_pid.IntegralLimit);

    ptr->speed_pid.P_out = ptr->speed_pid.kp * ptr->speed_pid.error;//P_OUT
    ptr->speed_pid.I_out = ptr->speed_pid.ki * ptr->speed_pid.error_all;//I_OUT
    ptr->speed_pid.D_out = ptr->speed_pid.kd * (ptr->speed_pid.error - ptr->speed_pid.error_last);//D_OUT
   
    ptr->speed_pid.error_last = ptr->speed_pid.error;//数据更新
    
    ptr->speed_pid.All_out = ptr->speed_pid.P_out + ptr->speed_pid.I_out + ptr->speed_pid.D_out;
    
    //输出限幅
    
    if(ptr->mes.KIND == M6020)
    {
      
      Limit(ptr->speed_pid.All_out,4000, -4000);//30000
    }
    else if(ptr->mes.KIND == M3508)
    {
      Limit(ptr->speed_pid.All_out,10000, -10000);//16384
    }
      
    //死区限制
    LimitDeadBand(ptr->speed_pid.All_out,ptr->speed_pid.DeadBand);

    return ptr->speed_pid.All_out;
}

float M6020_angle_cal(Motor_type_def *ptr,int16_t angle)
{
  
    while(angle>8192)angle = angle - 8192;//圆周处理。上行限幅。
    if(angle<0){angle=8192+angle;} //下行限幅
    ptr->angle_pid.error = angle - ptr->data.angle;//偏差
     
    //过零处理,统一成劣弧
    if(ptr->angle_pid.error>( 8192/2))ptr->angle_pid.error = ptr->angle_pid.error - 8192;
    if(ptr->angle_pid.error<(-8192/2))ptr->angle_pid.error = ptr->angle_pid.error + 8192;    
    
    ptr->angle_pid.error_all += ptr->angle_pid.error;
    
    //积分限幅
  //  Limit(ptr->angle_pid.error_all,ptr->angle_pid.IntegralLimit,-ptr->angle_pid.IntegralLimit);

    ptr->angle_pid.P_out = ptr->angle_pid.kp * ptr->angle_pid.error;//P_OUT
    ptr->angle_pid.I_out = ptr->angle_pid.ki * ptr->angle_pid.error_all;//I_OUT
    ptr->angle_pid.D_out = ptr->angle_pid.kd * (ptr->angle_pid.error - ptr->angle_pid.error_last);//D_OUT
   
    ptr->angle_pid.error_last = ptr->angle_pid.error;//数据更新
    
    ptr->angle_pid.All_out = ptr->angle_pid.P_out + ptr->angle_pid.I_out + ptr->angle_pid.D_out;
    
    //输出限幅
  //  Limit(ptr->angle_pid.All_out,ptr->angle_pid.MaxOutput, -ptr->angle_pid.MaxOutput);
    //死区限制
    LimitDeadBand(ptr->angle_pid.All_out,ptr->angle_pid.DeadBand);

    return ptr->angle_pid.All_out;
}

float M3508_angle_cal_angle(Motor_type_def *ptr,int16_t angle)
{
  
    while(angle>8192)angle = angle - 8192;//圆周处理。上行限幅。
    if(angle<0){angle=8192+angle;} //下行限幅
    ptr->angle_pid.error = angle - ptr->data.angle_old;//偏差
     
    //过零处理,统一成劣弧
    if(ptr->angle_pid.error>( 8192/2))ptr->angle_pid.error = ptr->angle_pid.error - 8192;
    if(ptr->angle_pid.error<(-8192/2))ptr->angle_pid.error = ptr->angle_pid.error + 8192;    
    
    ptr->angle_pid.error_all += ptr->angle_pid.error;
    
    //积分限幅
  //  Limit(ptr->angle_pid.error_all,ptr->angle_pid.IntegralLimit,-ptr->angle_pid.IntegralLimit);

    ptr->angle_pid.P_out = ptr->angle_pid.kp * ptr->angle_pid.error;//P_OUT
    ptr->angle_pid.I_out = ptr->angle_pid.ki * ptr->angle_pid.error_all;//I_OUT
    ptr->angle_pid.D_out = ptr->angle_pid.kd * (ptr->angle_pid.error - ptr->angle_pid.error_last);//D_OUT
   
    ptr->angle_pid.error_last = ptr->angle_pid.error;//数据更新
    
    ptr->angle_pid.All_out = ptr->angle_pid.P_out + ptr->angle_pid.I_out + ptr->angle_pid.D_out;
    
    //输出限幅
  //  Limit(ptr->angle_pid.All_out,ptr->angle_pid.MaxOutput, -ptr->angle_pid.MaxOutput);
    //死区限制
    LimitDeadBand(ptr->angle_pid.All_out,ptr->angle_pid.DeadBand);

    return ptr->angle_pid.All_out;
}

//3508电机角度环计算  total_angle
float M3508_angle_cal(Motor_type_def *ptr,int16_t angle)
{ 
    ptr->angle_pid.error = angle - ptr->data.angle;//偏差
    ptr->angle_pid.error_all += ptr->angle_pid.error;//偏差积分
    
    //积分限幅
  //  Limit(ptr->angle_pid.error_all,ptr->angle_pid.IntegralLimit,-ptr->angle_pid.IntegralLimit);

    ptr->angle_pid.P_out = ptr->angle_pid.kp * ptr->angle_pid.error;//P_OUT
    ptr->angle_pid.I_out = ptr->angle_pid.ki * ptr->angle_pid.error_all;//I_OUT
    ptr->angle_pid.D_out = ptr->angle_pid.kd * (ptr->angle_pid.error - ptr->angle_pid.error_last);//D_OUT
   
    ptr->angle_pid.error_last = ptr->angle_pid.error;//数据更新
    
    ptr->angle_pid.All_out = ptr->angle_pid.P_out + ptr->angle_pid.I_out + ptr->angle_pid.D_out;
    
    //输出限幅
  //  Limit(ptr->angle_pid.All_out,ptr->angle_pid.MaxOutput, -ptr->angle_pid.MaxOutput);
    //死区限制
    LimitDeadBand(ptr->angle_pid.All_out,ptr->angle_pid.DeadBand);

    return ptr->angle_pid.All_out;

}

//获得3508电机total_angle
void total_angle_get(Motor_type_def *ptr)
{ 
  float res1=0,res2=0;
  float err,err_err;
  static float pos,pos_old;
  
  pos = (rotate_motor.data.angle/8092.0f*360.0f);
  
  err = pos - pos_old ;
   
  if(err>0)
  {
    res1 = err - 360;
    res2 = err;
  }
  else
  {
    res1 = err + 360;
    res2 = err;  
  }
  if(zn_abs(res1)<zn_abs(res2)) //不管正反转，肯定是转的角度小的那个是真的
  {
    err_err = res1;
  }
  else 
  {
    err_err = res2;
  }
   pos_old = pos;
   rotate_motor.data.total_angle += err_err;  
}