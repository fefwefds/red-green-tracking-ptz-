#include "zn_pid.h"
#include "CAN_receive.h"



void pid_paragram_init(pid_type_def *ptr,float kp,float ki,float kd)
{
  ptr->kp = kp;
  ptr->kp = ki;
  ptr->kp = kd;

}


/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval ��ǰ���
  * @attention  
  */
float RAMP_float( float finala, float now, float ramp )
{
   float buffer = 0;

   buffer = finala - now;
	
    if (buffer > 0)
    {
      if (buffer > ramp)
      {  
       now += ramp;
      }   
      else
      {
        now += buffer;
      }
    }
    else
     {
       if (buffer < -ramp)
       {
          now += -ramp;
       }
       else
       {
          now += buffer;
       }
     }
		
    return now;
}

