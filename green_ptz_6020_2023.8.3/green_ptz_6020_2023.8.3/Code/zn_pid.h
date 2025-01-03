#ifndef ZN_PID_H
#define ZN_PID_H

#include "main.h"
#include "struct_typedef.h"

//�޷�
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

//����
#define LimitDeadBand(input, min)              \
        if (input <= min && input >= -min)       \
        {                                       \
            input = 0;                          \
        } 

#define zn_abs(x) ((x) > 0 ? (x) : (-x))


/*����PID��ز����ṹ��*/
typedef struct _PID_TypeDef
{
	float  kp;                       //����
	float  ki;                       //����
	float  kd;                       //΢��     
  
  	float  MaxOutput;		        //����޷�
	float  IntegralLimit;		//�����޷�
	float  DeadBand;			//����	
  
	float  target;		      //�ٶȻ�Ŀ��ֵ
        float   measure;		      //�ٶȲ���ֵ
        
        float   error;		      //���
	float   error_last;      	      //�ϴ����
        float   error_all;               //ƫ�����
 
        float  P_out;                          //POUT
	float  I_out;                          //IOUT
	float  D_out;                          //DOUT
        float  All_out;                    //�����
	
}pid_type_def; 

void pid_paragram_init(pid_type_def *ptr,float kp,float ki,float kd);

float RAMP_float( float finala, float now, float ramp );

#endif