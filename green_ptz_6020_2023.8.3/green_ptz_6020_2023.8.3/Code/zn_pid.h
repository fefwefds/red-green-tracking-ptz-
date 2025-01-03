#ifndef ZN_PID_H
#define ZN_PID_H

#include "main.h"
#include "struct_typedef.h"

//限幅
#define Limit(input, max,min)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < min) \
        {                      \
            input = min;      \
        }                      \
    }

//死区
#define LimitDeadBand(input, min)              \
        if (input <= min && input >= -min)       \
        {                                       \
            input = 0;                          \
        } 

#define zn_abs(x) ((x) > 0 ? (x) : (-x))


/*经典PID相关参数结构体*/
typedef struct _PID_TypeDef
{
	float  kp;                       //比例
	float  ki;                       //积分
	float  kd;                       //微分     
  
  	float  MaxOutput;		        //输出限幅
	float  IntegralLimit;		//积分限幅
	float  DeadBand;			//死区	
  
	float  target;		      //速度环目标值
        float   measure;		      //速度测量值
        
        float   error;		      //误差
	float   error_last;      	      //上次误差
        float   error_all;               //偏差积分
 
        float  P_out;                          //POUT
	float  I_out;                          //IOUT
	float  D_out;                          //DOUT
        float  All_out;                    //总输出
	
}pid_type_def; 

void pid_paragram_init(pid_type_def *ptr,float kp,float ki,float kd);

float RAMP_float( float finala, float now, float ramp );

#endif