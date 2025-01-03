/**
  ****************************(C) �� �� �� �� ս �� ****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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

#include "CAN_receive.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_TxHeaderTypeDef     TxMessage; 
CAN_RxHeaderTypeDef     RxMessage;

Communication can_message;

 Data Data_can1;
 Data Data_can2;//������


/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance==CAN1)
	{
             HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMessage,Data_can1.rxData);//�������������ջ�����       

             if(RxMessage.StdId == 0x4FF)
              {
                can_communication_rx(&can_message,&Data_can1);
              }
             
            get_moto_measure(&rudder_FL);
            get_moto_measure(&rudder_FR);
            get_moto_measure(&rudder_BL);
            get_moto_measure(&rudder_BR);
            get_moto_measure(&yuntai_yaw);
            get_moto_measure(&yuntai_pitch);
             
	}
	if(hcan->Instance==CAN2)
	{
             HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&RxMessage,Data_can2.rxData);//�������������ջ�����   
	
            get_moto_measure(&yaw_motor);
            get_moto_measure(&pitch_motor);
            
            get_moto_measure(&drive_FL);
            get_moto_measure(&drive_FR);
            get_moto_measure(&drive_BL);
            get_moto_measure(&drive_BR);
            
//            get_moto_measure(&rotate_motor);//����
//            get_moto_measure(&rub_motor_L);//Ħ����
//            get_moto_measure(&rub_motor_R);
        }  
}

/**
  * @brief          �ڶ�Ӧ��CAN�����϶�ȡ��Ӧ����Ϣ����ŵ�������ݽṹ����
  * @param[in]      ptr :  ����ṹ��
  * @retval         none
  */
void get_moto_measure(Motor_type_def *ptr)
{
    if(ptr->mes.CAN == ZN_CAN1 && RxMessage.StdId == ptr->mes.back_StdId)
    {
      copy_can_data(ptr,&Data_can1);
    }
    if(ptr->mes.CAN == ZN_CAN2 && RxMessage.StdId == ptr->mes.back_StdId)
    {
      copy_can_data(ptr,&Data_can2);
    }  
}

/**
  * @brief          ����������������ȡ����
  * @param[in]      ptr :  ����ṹ��
  * @param[in]      dat :  ���������ݽṹ��
  * @retval         none
  */
void copy_can_data(Motor_type_def *ptr,Data *dat)
{ 
   ptr->data.last_angle = ptr->data.angle;
   ptr->data.angle = (uint16_t)(dat->rxData[0]<<8 |dat->rxData[1]) ;
   ptr->data.speed_rpm  = (int16_t)(dat->rxData[2]<<8 | dat->rxData[3]);
   ptr->data.real_current = (dat->rxData[4]<<8 | dat->rxData[5])*5.f/16384.f;
   
   if(ptr->mes.KIND == M3508)//���� M3508 total_angle
   {
     total_angle_get(ptr);
   }
}

/**
  * @brief          ͨ��can���͵������ݸ����
  * @param[in]      ptr     : ����ṹ��
  * @param[in]      current : �趨����
  * @retval         none
  */

int16_t OUT_current[2][3][4];//�ֱ���0X200   0X1FF   0X2FF �ķ�������

void CAN_Set_Current(Motor_type_def *ptr,int16_t current)
{
    if(ptr->mes.CAN == ZN_CAN1)    
    {
      if(ptr->mes.control_StdId == 0X200)
      {
        OUT_current[0][0][ptr->mes.ID - 1] = current;
        can_message_sent(ZN_CAN1,0X200,OUT_current[0][0][0],OUT_current[0][0][1],OUT_current[0][0][2],OUT_current[0][0][3]);
      }
      if(ptr->mes.control_StdId == 0X1FF)
      {
        if(ptr->mes.KIND == M3508)
        {
        OUT_current[0][1][ptr->mes.ID - 5] = current;
        can_message_sent(ZN_CAN1,0X1FF,OUT_current[0][1][0],OUT_current[0][1][1],OUT_current[0][1][2],OUT_current[0][1][3]);        
        }
        else if(ptr->mes.KIND == M6020)
        {
        OUT_current[0][1][ptr->mes.ID - 1] = current;
        can_message_sent(ZN_CAN1,0X1FF,OUT_current[0][1][0],OUT_current[0][1][1],OUT_current[0][1][2],OUT_current[0][1][3]);        
        }
      }       
      if(ptr->mes.control_StdId == 0X2FF)
      {
        OUT_current[0][2][ptr->mes.ID - 5] = current;
        can_message_sent(ZN_CAN1,0X2FF,OUT_current[0][2][0],OUT_current[0][2][1],OUT_current[0][2][2],OUT_current[0][2][3]);
      }         
    } 
    else   
    {
      if(ptr->mes.control_StdId == 0X200)
      {
        OUT_current[1][0][ptr->mes.ID - 1] = current;
        can_message_sent(ZN_CAN2,0X200,OUT_current[1][0][0],OUT_current[1][0][1],OUT_current[1][0][2],OUT_current[1][0][3]);
      }
      if(ptr->mes.control_StdId == 0X1FF)
      {
        if(ptr->mes.KIND == M3508)
        {
        OUT_current[1][1][ptr->mes.ID - 5] = current;
        can_message_sent(ZN_CAN2,0X1FF,OUT_current[1][1][0],OUT_current[1][1][1],OUT_current[1][1][2],OUT_current[1][1][3]);        
        }
        else if(ptr->mes.KIND == M6020)
        {
        OUT_current[1][1][ptr->mes.ID - 1] = current;
        can_message_sent(ZN_CAN2,0X1FF,OUT_current[1][1][0],OUT_current[1][1][1],OUT_current[1][1][2],OUT_current[1][1][3]);        
        }
      }       
      if(ptr->mes.control_StdId == 0X2FF)
      {
        OUT_current[1][2][ptr->mes.ID - 5] = current;
        can_message_sent(ZN_CAN2,0X2FF,OUT_current[1][2][0],OUT_current[1][2][1],OUT_current[1][2][2],OUT_current[1][2][3]);
      }         
    }    
    
}

void can_message_sent(motor_CAN CAN,int STDID,int16_t mes1, int16_t mes2, int16_t mes3, int16_t mes4)
{
    uint8_t  send_data[8];   
    uint32_t send_mail_box;
  
    TxMessage.StdId = STDID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    send_data[0] = mes1 >> 8;
    send_data[1] = mes1;
    send_data[2] = mes2 >> 8;
    send_data[3] = mes2;
    send_data[4] = mes3 >> 8;
    send_data[5] = mes3;
    send_data[6] = mes4 >> 8;
    send_data[7] = mes4;

    if(CAN == ZN_CAN1)
    {
       HAL_CAN_AddTxMessage(&hcan1, &TxMessage, send_data, &send_mail_box);  
    }
      
    else if(CAN == ZN_CAN2)
    {
       HAL_CAN_AddTxMessage(&hcan2, &TxMessage, send_data, &send_mail_box);
    }
      
}
 
void can_communication_tx(uint8_t mes[8])
{
    uint8_t    M2006_send_data[8];  
    uint32_t send_mail_box;
  
    TxMessage.StdId = 0X3FF;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    M2006_send_data[0] = mes[0];
    M2006_send_data[1] = mes[1];
    M2006_send_data[2] = mes[2];
    M2006_send_data[3] = mes[3];
    M2006_send_data[4] = mes[4];
    M2006_send_data[5] = mes[5];
    M2006_send_data[6] = mes[6];
    M2006_send_data[7] = mes[7];

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, M2006_send_data, &send_mail_box);
}

void can_communication_rx(Communication *ptr,Data *dat)
{
  ptr->speed[0] = (int16_t)(dat->rxData[0]<<8 |dat->rxData[1]);
  ptr->speed[1] = (int16_t)(dat->rxData[2]<<8 |dat->rxData[3]);
  ptr->speed[2] = (int16_t)(dat->rxData[4]<<8 |dat->rxData[5]);
  ptr->speed[3] = (int16_t)(dat->rxData[6]<<8 |dat->rxData[7]); 
  
  ptr->angle[0] = (double)ptr->speed[0]/100;
  ptr->angle[1] = (double)ptr->speed[1]/100;
  ptr->angle[2] = (double)ptr->speed[2]/100;
  ptr->angle[3] = (double)ptr->speed[3]/100;  
}
