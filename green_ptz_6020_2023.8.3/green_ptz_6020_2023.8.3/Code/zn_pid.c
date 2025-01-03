#include "zn_pid.h"
#include "CAN_receive.h"



void pid_paragram_init(pid_type_def *ptr,float kp,float ki,float kd)
{
  ptr->kp = kp;
  ptr->kp = ki;
  ptr->kp = kd;

}


/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
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

