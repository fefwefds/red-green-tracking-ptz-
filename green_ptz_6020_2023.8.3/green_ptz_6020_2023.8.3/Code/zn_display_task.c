#include "zn_display_task.h"
#include "ANO_DT.h"
#include "math.h"
#include "zn_interaction.h"
#include "string.h"
#include "referee_usart_task.h"
#include "referee.h"
#include "INS_task.h"


void ZN_display_task()
{
  static portTickType currentTime;
    
  while(1)
  {
    /*code begin*/  
    
//        ANO_DT_Send_Senser( (int)power_heat_data_t.chassis_power,//底盘输出功率
//                            (int)power_heat_data_t.chassis_power_buffer,//缓冲功率
//                            (int)robot_state.remain_HP,//剩余血量
//                            40,
//                            0,
//                            0,
//                            0,
//                            0,
//                            0,
//                            0);       
  /*code end*/
  vTaskDelayUntil(&currentTime, 10);//绝对延时 ms
  }
}
