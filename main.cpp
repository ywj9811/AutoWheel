#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>

#include "motor_driver.h"
#include "rplidar_driver.h"
#include "joystick_reader.h"

int autonomous_onOff_flag;
pthread_mutex_t autonomous_onOff_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t autonomous_onOff_cond = PTHREAD_COND_INITIALIZER;

int motor_left_cnt = 0;

int main()
{
    pthread_t lidar_thread, joystick_thread, motor_thread; //쓰레드 3개 선언

    // 우선 모터 시작
    if(motor_init() != 0) {
        printf("motor init fail\n");
    }

    // motor_run 함수를 motor_thread 쓰레드에서 동작시키도록 생성
    if(pthread_create(&motor_thread, NULL, motor_run, NULL) != 0) {
        fprintf(stderr, "Thread lidar_thread creation failed\n");
        exit(EXIT_FAILURE);
    }

    // rplidar_run 함수를 lidar_thread 쓰레드에서 동작시키도록 생성
    if(pthread_create(&lidar_thread, NULL, rplidar_run, NULL) != 0) {
        fprintf(stderr, "Thread lidar_thread creation failed\n");
        exit(EXIT_FAILURE);
    }

    // joystick_run 함수를 joystick_thread 쓰레드에서 동작시키도록 생성
    if(pthread_create(&joystick_thread, NULL, joystick_run, NULL) != 0) {
        fprintf(stderr, "Thread joystick_thread creation failed\n");
        exit(EXIT_FAILURE);
    }

    // 위에서 생성한 쓰레드들이 종료될때까지 기다리도록 함 (return을 하지 않도록 하는 것과 병렬로 실행될 수 있도록)
    pthread_join(joystick_thread, NULL);
    pthread_join(lidar_thread, NULL);
    pthread_join(motor_thread, NULL);

    return 0;
}