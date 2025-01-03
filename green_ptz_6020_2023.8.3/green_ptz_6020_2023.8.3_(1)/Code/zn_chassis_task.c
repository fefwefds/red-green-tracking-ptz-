#include "zn_chassis_task.h"
#include "bsp_laser.h"
#include "ANO_DT.h"
#include "remote_control.h"
#include "math.h"
#include "zn_gimbal_task.h"
#include "zn_interaction.h"
#include "string.h"
#include "zn_chassis_motor.h"
#include "zn_vision_task.h"

yuntai_ yuntai;

void ZN_chassis_task(void const *pvParameters)
{	
   //Chassis_Mode_Change(Robot1.Status.actChassis);
   //���̵�����
   CHASSIS_Single_Loop_Out();
  /*code end*/
}

/**
  * @brief  �л����̿���ģʽ
  * @param  ģʽ�л���־λ
  * @retval none
  * @attention
  */
void Chassis_Mode_Change(eChassisAction mode)
{
    switch(mode) 
    {
    case CHASSIS_LOCK: //����
          Robot1.Movement.Chassis_speed_out.vx = 0;  //X
          Robot1.Movement.Chassis_speed_out.vy = 0;  //Y 
          Robot1.Movement.Chassis_speed_out.vw = 0;  //WZ    
          Absolute_Cal(&Robot1.Movement.Chassis_speed_out,0);//������������Ŀ���ٶ�    
          break;      
      
    case CHASSIS_SEPARATE://����
          Absolute_Cal(&Robot1.Movement.Chassis_speed_out,((yaw_motor.data.angle - Robot1.Mechanical.YAW_Motor_Middle)*0.043945f));//������������Ŀ���ٶ�    
          break;
          
    case CHASSIS_FOLLOW_GIMBAL : //����
         Robot1.Movement.Chassis_speed_out.vw  = Chassis_Follow(&chassis_follow);    //�涯pid         
         Absolute_Cal(&Robot1.Movement.Chassis_speed_out,0);//������������Ŀ���ٶ�    
          break;    
          
    case CHASSIS_GYROSCOPE : //С����
         Robot1.Movement.Chassis_speed_out.vw = Robot1.Movement.gyroscope_speed;//С�����ٶ�
//         Absolute_Cal(&Robot1.Movement.Chassis_speed_out,((yaw_motor.data.angle - Robot1.Mechanical.YAW_Motor_Middle)*0.043945f));//������������Ŀ���ٶ�    
          break;   
          
    default:
        break;          
    }
}

/**
  * @brief  ����̨����ת��Ϊ��������
  * @param  absolute_speed ����������Ҫ���ٶ� 
  * @param  angle ��̨����ڵ��̵ĽǶ�
  * @retval ƫ��ǣ��Ƕ���
  * @attention
  */
void Absolute_Cal(Chassis_Speed* absolute_speed, fp32 angle)
{
    fp32 angle_hd=angle* PI / 180;
    Chassis_Speed temp_speed;
    temp_speed.vw = absolute_speed->vw;
    temp_speed.vx = absolute_speed->vx * cos(angle_hd) - absolute_speed->vy * sin(angle_hd);
    temp_speed.vy = absolute_speed->vx * sin(angle_hd) + absolute_speed->vy * cos(angle_hd);
    
    AGV_angle_calc(&temp_speed,Robot1.Movement.chassis_6020_setangle);//��������ֽǶ�
    AGV_calc(&temp_speed,Robot1.Movement.chassis_3508_setspeed);//�����������ٶ�
}

/**
  * @brief  ����������������Ŀ���ٶ�
  * @param  speed ����������ٶ� 
  * @param  out_speed 3508Ŀ���ٶ�
  * @retval 
  * @attention
  */
int16_t drct[4]={1,1,1,1};//���������������ת
void AGV_calc(Chassis_Speed *speed, int16_t* out_speed) 
{
	  //3508Ŀ���ٶȼ���
    double wheel_rpm[4];
    fp32 wheel_rpm_ratio;
    wheel_rpm_ratio = 60.0f / (Robot1.Mechanical.WHEEL_PERIMETER * PI) * Robot1.Mechanical.CHASSIS_DECELE_RATIO ;
    
    wheel_rpm[FL_num] = sqrt(	pow(speed->vy - speed->vw * Robot1.Mechanical.Radius * 0.707107f,2)
                       +	pow(speed->vx - speed->vw * Robot1.Mechanical.Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio;
    wheel_rpm[FR_num] = sqrt(	pow(speed->vy - speed->vw * Robot1.Mechanical.Radius * 0.707107f,2)
                       +	pow(speed->vx + speed->vw * Robot1.Mechanical.Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio;
    wheel_rpm[BL_num] = sqrt(	pow(speed->vy + speed->vw * Robot1.Mechanical.Radius * 0.707107f,2)
                       +	pow(speed->vx - speed->vw * Robot1.Mechanical.Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio;
    wheel_rpm[BR_num] = sqrt(	pow(speed->vy + speed->vw * Robot1.Mechanical.Radius * 0.707107f,2)
                       +	pow(speed->vx + speed->vw * Robot1.Mechanical.Radius * 0.707107f,2) 
                       ) * wheel_rpm_ratio;
	  
    for(int i=0;i<4;i++)
       out_speed[i] = drct[i] * (int16_t)wheel_rpm[i];
}

/**
  * @brief  ������̺�������Ŀ��Ƕ�
  * @param  speed ����������ٶ� 
  * @param  out_angle 6020Ŀ��Ƕ�
  * @retval 
  * @attention
  */
void AGV_angle_calc(Chassis_Speed *speed, fp32* out_angle)
{
    fp32 wheel_angle[4]; //6020������Ŀ��Ƕ�
    static fp32 wheel_angle_last[4]={2700,5450,100,1320};//2800 5450 150 1320 
    fp64 atan_angle[4];
    
    //6020Ŀ��Ƕȼ���
    if(!(speed->vx == 0 && speed->vy == 0 && speed->vw == 0))//��ֹ����Ϊ��
    {
      atan_angle[FL_num]  = atan2((speed->vx + speed->vw*Robot1.Mechanical.Radius*0.707107f),(speed->vy + speed->vw*Robot1.Mechanical.Radius*0.707107f))*180.0f/PI;
      atan_angle[FR_num]  = atan2((speed->vx + speed->vw*Robot1.Mechanical.Radius*0.707107f),(speed->vy - speed->vw*Robot1.Mechanical.Radius*0.707107f))*180.0f/PI;
      atan_angle[BL_num]  = atan2((speed->vx - speed->vw*Robot1.Mechanical.Radius*0.707107f),(speed->vy + speed->vw*Robot1.Mechanical.Radius*0.707107f))*180.0f/PI;
      atan_angle[BR_num]  = atan2((speed->vx - speed->vw*Robot1.Mechanical.Radius*0.707107f),(speed->vy - speed->vw*Robot1.Mechanical.Radius*0.707107f))*180.0f/PI;	
    }  

      wheel_angle[FL_num] = Robot1.Mechanical.GM6020_init_position[0] + (fp32)(atan_angle[0]*22.75);
      wheel_angle[FR_num] = Robot1.Mechanical.GM6020_init_position[1] + (fp32)(atan_angle[1]*22.75);
      wheel_angle[BL_num] = Robot1.Mechanical.GM6020_init_position[2] + (fp32)(atan_angle[2]*22.75);
      wheel_angle[BR_num] = Robot1.Mechanical.GM6020_init_position[3] + (fp32)(atan_angle[3]*22.75);
        
      //�߼��Ż����ǶȲ�ϴ�ʱ�����ַ�ת            
      for(int i=0;i<4;i++)
      {
        AngleLoop_f(&wheel_angle[i],8192);//��ת����0~8192
        
           if(zn_abs(Find_min_Angle((int16_t)wheel_angle[i],wheel_angle_last[i])) >= 2840) 
        {
          wheel_angle[i] += 4096;
          drct[i] = -1;
        }
        else drct[i] = 1;
      }

     //ҡ�˻���ʱ�����ϴ�״̬
      if(speed->vx == 0 && speed->vy == 0 && speed->vw == 0)//ҡ�˻���ʱ
      {
        for(int i=0;i<4;i++)
         out_angle[i] = wheel_angle_last[i];
      }
      else
      {
        for(int i=0;i<4;i++)
        {
         out_angle[i] = wheel_angle[i];
         wheel_angle_last[i] = wheel_angle[i];
        }
      }
}


float speed_a1,speed_a2;
int yaw_motor_mid=-3360;
int pitch_motor_mid=-900;
int yaw_motor_out,pitch_motor_out;

Point location[4]=//�ڽ���4�����꣬˳ʱ��˳��
{
   {0,0},
   {0,0},
   {0,0},
   {0,0},
};

int paper_flag=0;//ҳ��״̬
int location_x;
int location_y;
//�ĵ���ʵ����ֵ��ֱ�����ü���
int location_x_1,location_y_1,
    location_x_2,location_y_2,
    location_x_3,location_y_3,
    location_x_4,location_y_4;

int err_x,err_y;
int err_x_speed,err_y_speed;
int num_num;
void state_control()
{
  err_x=location_x_1;err_y=location_y_1;
}

float led_gre_flow(float set_distance,float now_distance);
float led_gre_flow_one(float set_distance,float now_distance);
void key_read_pin();

float x_angle=0,y_angle=0;
float x_p=1,y_p=1;//�������
int flag,err_flag;

void CHASSIS_Single_Loop_Out()//�̹�
{

  state_control();num_num++;
  
  if(err_flag==0){x_angle=0;y_angle=0;}
  speed_a1=-led_gre_flow(0,x_angle+x_p*err_x);
  speed_a2=-led_gre_flow_one(0,y_angle+y_p*err_y); 
  
  if(speed_a1!=0){err_flag=1;}
  yaw_motor_out=speed_a1+yaw_motor_mid;
  pitch_motor_out=speed_a2+pitch_motor_mid;
  
//  yaw_motor_out=yaw_motor_mid;
//  pitch_motor_out=pitch_motor_mid;
//  RAMP_float
//  if(yaw_motor_out>=-2900){yaw_motor_out=-2900;}
//  if(yaw_motor_out<=-4600){yaw_motor_out=-4600;}
//  if(pitch_motor_out>=-3400){pitch_motor_out=-3400;}
//  if(pitch_motor_out<=-4000){pitch_motor_out=-4000;}

  if(flag==0)
  {
    Assign_To_M6020_angle(&yuntai_yaw,yaw_motor_out);
    Assign_To_M6020_angle(&yuntai_pitch,pitch_motor_out);
  }
   
   
}

//λ��ʽѰ���
float led_output;
float led_flow_P,led_flow_I,led_flow_D;
float led_Kp=-1.2,led_Ki=-0.015,led_Kd=0.2;
float led_err,led_last_err,led_last_last_err,led_err_all,led_err_err;
float led_gre_flow(float set_distance,float now_distance)
{
  led_err=set_distance-now_distance;

  led_err_all+=led_err;
  led_err_err=led_err-led_last_err;
  
//  if(led_err_err>=50){led_err_err=50;}
//  if(led_err_err<=-50){led_err_err=-50;}
  
  led_flow_P=led_Kp*led_err;
  led_flow_I=led_Ki*led_err_all;
  led_flow_D=led_Kd*led_err_err;
  
  led_last_last_err=led_last_err;
  led_last_err=led_err;
  
  led_output=led_flow_P+led_flow_I+led_flow_D;
  
//  if(led_output>=300){led_output=300;}
//  if(led_output<=-300){led_output=-300;}
  return led_output;
}

//λ��ʽѰ���
float led_output_one;
float led_flow_P_one,led_flow_I_one,led_flow_D_one;
float led_Kp_one=1.8,led_Ki_one=0.029,led_Kd_one=0;
float led_err_one,led_last_err_one,led_last_last_err_one,led_err_all_one,led_err_err_one;
float led_gre_flow_one(float set_distance,float now_distance)
{
  led_err_one=set_distance-now_distance;

  led_err_all_one+=led_err_one;
  led_err_err_one=led_err_one-led_last_err_one;
  
  led_flow_P_one=led_Kp_one*led_err_one;
  led_flow_I_one=led_Ki_one*led_err_all_one;
  led_flow_D_one=led_Kd_one*led_err_err_one;  
  
//  if(led_err_all_one>=50){led_err_all_one=50;}
//  if(led_err_all_one<=-50){led_err_all_one=-50;}
  
  led_last_last_err_one=led_last_err_one;
  led_last_err_one=led_err_one;
  
  led_output_one=led_flow_P_one+led_flow_I_one+led_flow_D_one;
//  if(led_output_one>=300){led_output_one=300;}
//  if(led_output_one<=-300){led_output_one=-300;}
  return led_output_one;
}

void location_caculate();

//����Э��
 static unsigned char port_data[11];
 static unsigned char report = 0;
//a1xxxxyyyyb
void serial_port_protocol(unsigned char data)
{
    port_data[report++] = data;
    if (port_data[0] != 0x61)  //���֡ͷ a->61
  {
    report = 0;
    return;
  }
    if (report < 11) { return; }  //���ֲ���11������
    if (port_data[10] != 0x62) 
  {
    report = 0;
    return;
  }
  
    else 
  {
    
    if(port_data[1]==0x30)//��Ļûֽ��ִ��������
    {
      paper_flag=0;
    }
    
    if(port_data[1]==0x31)//��Ļ��ֽ��ִ�вɵ����
    {
      paper_flag=1; //��һ��
    }
     if(port_data[1]==0x32)//��Ļ��ֽ��ִ�вɵ����
    {
      paper_flag=2; //�ڶ���
    }
     if(port_data[1]==0x33)//��Ļ��ֽ��ִ�вɵ����
    {
      paper_flag=3; //������
    }
     if(port_data[1]==0x34)//��Ļ��ֽ��ִ�вɵ����
    {
      paper_flag=4; //���ĵ�
    }
    
    location_caculate();
    
    report = 0;
  }  
}

//����ֵ����
void location_caculate()
{
    //��һ�����괦��   x����
      location[1].loc_1_x=port_data[3];
      location[1].loc_2_x=port_data[4];
      location[1].loc_3_x=port_data[5];
      location_x=(location[1].loc_1_x-48)*100+(location[1].loc_2_x-48)*10+location[1].loc_3_x-48;
      if(port_data[2]==0x30)//�ж�����
      {
        location_x=-location_x;
      }
      else if(port_data[2]==0x31)
      {
        location_x=location_x;
      } 
      //��һ������   y����
            
      location[1].loc_1_y=port_data[7];
      location[1].loc_2_y=port_data[8];
      location[1].loc_3_y=port_data[9];
      location_y=(location[1].loc_1_y-48)*100+(location[1].loc_2_y-48)*10+location[1].loc_3_y-48;
      if(port_data[6]==0x30)//�ж�����
      {
        location_y=-location_y;
      }
      else if(port_data[6]==0x31)
      {
        location_y=location_y;
      }
      
      //���긳ֵ
      if(paper_flag==1)
      {
        location_x_1=location_x;
        location_y_1=location_y;
      }
       if(paper_flag==2)
      {
        location_x_2=location_x;
        location_y_2=location_y;
      }
       if(paper_flag==3)
      {
        location_x_3=location_x;
        location_y_3=location_y;
      }
       if(paper_flag==4)
      {
        location_x_4=location_x;
        location_y_4=location_y;
      }    
}

int control_state;
void key_read_pin()
{
  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9);
  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
  HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14);
  
  if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9))//��ʼ����
  {
    control_state=1;
  }
  
}

/**
  * @brief  �ǶȻػ� ����
  * @param  void
  * @retval �Ƕ�ֵ�����Ƕ�ֵ
  * @attention 
  */
void AngleLoop_f (float* angle ,float max)
{
  while((*angle<-(max/2)) ||(*angle>(max/2)))
  {
    if(*angle<-(max/2))
    {
      *angle+=max;
    }
    else if(*angle>(max/2))
    {
      *angle-=max;
    }
  }
}

/**
  * @brief  �ǶȻػ� ����
  * @param  void
  * @retval �Ƕ�ֵ�����Ƕ�ֵ
  * @attention 
  */
void AngleLoop_int (int16_t* angle ,int16_t max)
{
  while((*angle<-(max/2)) ||(*angle>(max/2)))
  {
    if(*angle<-(max/2))
    {
      *angle+=max;
    }
    else if(*angle>(max/2))
    {
      *angle-=max;
    }
  }
}

/**
  * @brief  �ҳ����ǵĽ�С��ֵ
  * @param  ��1����2
  * @retval 
  * @attention 
  */
fp32 Find_min_Angle(int16_t angle1,fp32 angle2)
{
    fp32 err;
    err = (fp32)angle1 - angle2;
    if(fabs(err) > 4096)
    {
        err = 8192 - fabs(err);
    }
    return err;
}