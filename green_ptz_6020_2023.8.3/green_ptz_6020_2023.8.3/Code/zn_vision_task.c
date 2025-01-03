#include "zn_vision_task.h"
#include "usart.h"
#include "stm32f4xx_hal_def.h"
#include "zn_motor.h"
#include "zn_chassis_task.h"


Message RE_Message;

Gimbal_t gimbal_process=
{
  .yaw_set=4200,
  .pitch_set=4200,
  .ATTACK_OPEN=false,
  .SCOUT_OPEN=true,
  
};

unsigned char UART1_Rx_Buf;
unsigned char UART6_Rx_Buf;
void USART1_IRQHandler()
{
  if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE)!=RESET))
  {
    HAL_UART_IRQHandler(&huart1);  
    HAL_UART_Receive_IT(&huart1,&UART1_Rx_Buf,1);
    Uart_Protocol(UART1_Rx_Buf);
  }
}

void USART6_IRQHandler()
{
   if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_RXNE)!=RESET))
  {
    HAL_UART_IRQHandler(&huart6);  
    HAL_UART_Receive_IT(&huart6,&UART6_Rx_Buf,1);
    serial_port_protocol(UART6_Rx_Buf);
//    Uart_Protocol(UART6_Rx_Buf);
  }
}



#define uart_head1  0x61  //֡ͷ-a
#define uart_tail   0x62 //֡β-b

int angle_flag;   //�Ƕ�����

//�Ƕ�
int rx_angle;  //���սǶ�

//����
int rx_distance;//���վ���


void Uart_Protocol(unsigned char dat)
{
  static char data[10];
  static unsigned char TX_flag=0;
  data[TX_flag]=dat;
  TX_flag++;
  if(data[0] != uart_head1 )  //���֡ͷ
  {   
    TX_flag=0;
    return;
  }
  
  
  if(TX_flag<9)  //������ݸ�������  ��������֡β��һ��������Ȼֱ�ӵ������֡β������
    return;
  
  if(data[8] != uart_tail) //���֡β
  {
    TX_flag=0;
    return;
  }  
  else
  {
    
    //����д��ͬ���ݶ�Ӧ�Ĳ�ͬ����
    switch(data[1])
    {
      case 0x30: angle_flag=-1; break;
      case 0x31: angle_flag=1;break;             
    }
    
    rx_angle=angle_flag*((data[2]-48)*100 + (data[3]-48)*10 + (data[4]-48)*1);
    
    rx_distance=(data[5]-48)*100 + (data[6]-48)*10 + (data[7]-48)*1;
    
    TX_flag=0;  //���ǵ���0 
  }
}
int attack_flag=0;
void CopeSerialData(unsigned char ucData)                 //����̨�������ݽ��лص�
{
  static unsigned char ucRxBuffer[10];
  static unsigned char ucRxCnt = 0;	
  
  ucRxBuffer[ucRxCnt++]=ucData;
  
  if ((ucRxBuffer[0]!=0x61)) //����ͷ���ԣ������¿�ʼѰ�� '9' ����ͷ
  {
    ucRxCnt=0;
    return;
  }
  
  if (ucRxCnt<10) {return;}//���ݲ���10�����򷵻�
  if (ucRxBuffer[9]!=0x62)//����yiba���ԣ���ȥ '8'
  {
    ucRxCnt=0;
    return;
  } 
  else
  {
        
  }
}