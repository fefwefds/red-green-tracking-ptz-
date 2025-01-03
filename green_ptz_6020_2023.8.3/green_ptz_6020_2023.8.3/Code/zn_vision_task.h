#ifndef ZN_VISION_TASK_H
#define ZN_VISION_TASK_H
#include "cmsis_os.h"
#include "main.h"
#include "stdbool.h"
#include "zn_chassis_task.h"
typedef struct
{
    int16_t  toutou;//??
    int16_t  fuhao1;
    int16_t  yaw;
    int16_t  fuhao2;
    int16_t  pitch;
    int16_t  yiba;//?¦Â
}Message;

extern Message RE_Message;

extern int angle_flag;

//½Ç¶È
extern int rx_angle;  

//¾àÀë
extern int rx_distance;

typedef struct 
{
    int16_t     yaw_set;	// YAWÖá
    int16_t     pitch_set;      // PITCHÖá
    bool        ATTACK_OPEN;    //ÊÇ·ñ¹¥»÷
    bool        SCOUT_OPEN;     //Ñ²º½×´Ì¬
} Gimbal_t;

extern Gimbal_t gimbal_process;
void vision_pid_init();
void CopeSerialData(unsigned char ucData);

void Uart_Protocol(unsigned char dat);
#endif