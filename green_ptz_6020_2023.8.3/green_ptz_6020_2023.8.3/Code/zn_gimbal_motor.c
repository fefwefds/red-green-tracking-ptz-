#include "zn_gimbal_motor.h"
#include "INS_task.h"

pid_type_def YAW_imu_angle ;//yaw��������ǽǶȻ�pid����
pid_type_def YAW_imu_speed ;
pid_type_def YAW_vision;
pid_type_def Pitch_vision;


float YAW_imuangle_cal(pid_type_def *ptr,float yaw_angle,float yaw_angle_set)
{ 
  ptr->error = yaw_angle_set - yaw_angle;

  //���㴦��,ͳһ���ӻ�
  if(ptr->error>=( 360/2))ptr->error = ptr->error - 360;
  if(ptr->error< (-360/2))ptr->error = ptr->error + 360;
  
  ptr->error_all += ptr->error;//ƫ�����
  
  //�����޷�
  
  ptr->P_out = ptr->kp * ptr->error;
  ptr->I_out = ptr->ki * ptr->error_all;
  ptr->D_out = ptr->kd * (ptr->error - ptr->error_last);
  
  ptr->error_last = ptr->error;
  
  ptr->All_out = ptr->P_out + ptr->I_out + ptr->D_out;
  
  //����޷�
  //��������
  
   return ptr->All_out;
}


//������ģʽ ����ٶ��ڻ�
float Motor_imuspeed_cal(pid_type_def *ptr,float imuspeed,float imuspeed_set)
{
  ptr->error = imuspeed_set - imuspeed;
  
  ptr->error_all += ptr->error;//ƫ�����
    //�����޷�
//  Limit(ptr->error_all,ptr->IntegralLimit,-ptr->IntegralLimit);
  
  ptr->P_out = ptr->kp * ptr->error;
  ptr->I_out = ptr->ki * ptr->error_all;
  ptr->D_out = ptr->kd * (ptr->error - ptr->error_last);

  ptr->error_last = ptr->error;
  
  ptr->All_out = ptr->P_out + ptr->I_out + ptr->D_out;
  
  //����޷�
  Limit(ptr->All_out,25000, -25000);//30000
  //��������
  
   return ptr->All_out;     
}

//����ģʽPID
float vision_PID(pid_type_def *ptr,float SetValue,float ActualValue)
{
  ptr->error = SetValue - ActualValue;
  
  ptr->P_out = ptr->kp * ptr->error;
  ptr->I_out = ptr->ki * ptr->error_last;
  ptr->D_out = ptr->kd * (ptr->error - ptr->error_last);
  
  Limit(ptr->I_out,100,-100);
  
  ptr->error_last = ptr->error;
  
  ptr->All_out = ptr->P_out + ptr->I_out + ptr->D_out;
 
  return ptr->All_out;
}

void YAW_Angle_Set(Motor_type_def *motor,pid_type_def *angle_pid,pid_type_def *speed_pid,float yaw_angle_set)
{
  YAW_imuangle_cal(angle_pid,can_message.angle[0],yaw_angle_set);
  Motor_imuspeed_cal(speed_pid,motor->data.speed_rpm,(int16_t)angle_pid->All_out);//�ٶȻ�����
  CAN_Set_Current(motor,(int16_t)speed_pid->All_out);//�������
}