#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdint.h>
#include <pthread.h>
#include <stdio.h>

#define LIDAR_DETECT_DIST 350 //35cm 기준으로 장애물 탐지
#define LIDAR_DETECT_ANGLE 82 //82도를 사용하기 때문에

#define LIDAR_DGREE_OFFSET 4 //약한 삐뚤어짐 4도정도
#define LIDAR_THETA_0_OFFSET ((LIDAR_DETECT_ANGLE/2)) //중앙 설정

typedef enum { // 자율주행 켜짐 꺼짐 -> autonomous_onOff_flag 여기에 저장
    AUTONOMOUS_ON_FLAG = 0,
    AUTONOMOUS_OFF_FLAG
}Autonomous_onOff_e;

extern pthread_mutex_t autonomous_onOff_mutex; //mutex설정
extern pthread_cond_t autonomous_onOff_cond; //조건 설정
extern int autonomous_onOff_flag; //현재 자율주행인가? 조이스틱인가?

extern int motor_left_cnt;


#endif