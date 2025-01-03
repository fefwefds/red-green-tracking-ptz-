#include "zn_shoot_task.h"
#include "cmsis_os.h"
#include "main.h"

#include "zn_motor.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "zn_interaction.h"
#include "ANO_DT.h"
#include "zn_chassis_motor.h"

uint8_t send_data[8];

void ZN_shoot_task(void const *pvParameters)
{
  static portTickType currentTime;
    
  while(1)
  {
   currentTime = xTaskGetTickCount();	//获取当前系统时间
    /*code begin*/
//   if(Robot1.Status.rub_flag == 1)
//   {
//     Robot1.Movement.rub_speed = 6000;
//   }
//   else
//   {
//     Robot1.Movement.rub_speed = 0;
//   }
   send_data[0] = DJI_RC.keyBoard.bit.X >> 8;
   send_data[1] = DJI_RC.keyBoard.bit.X;
   send_data[2] = DJI_RC.mouse.press_l   >> 8;
   send_data[3] = DJI_RC.mouse.press_l;
   
   can_communication_tx(send_data);
   
//   ANO_DT_Send_Senser( (int16_t)(can_message.angle[0]*100),
//                       (int16_t)(chassis_follow.All_out*100),
//                            0,
//                            0,
//                            0,
//                            0,
//                            0,
//                            0,
//                            0,
//                            0);
//   Assign_To_Motor_speed(&rub_motor_L,-Robot1.Movement.rub_speed);
//   Assign_To_Motor_speed(&rub_motor_R, Robot1.Movement.rub_speed);
     
  /*code end*/
  vTaskDelayUntil(&currentTime, 2);//绝对延时 ms
  }
}
