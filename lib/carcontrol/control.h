//
// Created by YZP on 2021/7/21.
//

#ifndef BAN_CONTROL_H
#define BAN_CONTROL_H
#include "stm32Config.h"
#include "tim.h"
#define PWMA   TIM1->CCR1    //PA8
#define AIN2   PBout(15)
#define AIN1   PBout(14)
#define BIN1   PBout(12)
#define BIN2   PBout(13)
#define PWMB   TIM1->CCR4    //PA11

void Set_Pwm(int moto1,int moto2);
int myabs(int arg);
extern int speedL,speedR;
#endif //BAN_CONTROL_H
