#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H
    /*
    keilû��stdint.h�ļ���������Ҫ������STRUCT_TYPEDEF_H�ļ�
    ��IAR��stdint.h�ļ����ʲ�ͬIDE��Ҫ�����
    ��������
    __CC_ARM   ��Ӧ   ARM RealView
    __ICCARM_  ��Ӧ   IAR EWARM
    __GNUC__   ��Ӧ   GNU Compiler Collection
    ���ں�Դ���������漰(---zmy)
    */
#include "stm32f4xx_hal.h"    
      
#if defined ( __ICCARM__   )
#include "stdint.h"
#else 
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;

#endif
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;


#endif



