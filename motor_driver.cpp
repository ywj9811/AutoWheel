#include <wiringPi.h>
// 라즈베리파이용 c언어 라이브러리

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>

#include "motor_driver.h"
#include "main.h"
// WiringPi 핀 번호, 라즈베리 파이의 GPIO 핀에 따라 변경해야 할 수 있습니다.
// 핀 번호

// 3개씩 각각 왼,오 모터임
#define IN1 4  
#define IN2 5 
#define EN1  1

#define IN3 22
#define IN4 23
#define EN2  24

#define SPEED_DEFAULT 100 //속도는 100으로 정의함

typedef struct { //모터 하나당 INPUT PIN2개, EN1개 필요함
    uint8_t IN[2];
    uint8_t EN;    
}motor_cfg_t;

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
}motor_type_e;

motor_cfg_t motor_cfg[2] = { // 하니씩 왼쪽, 오른쪽
    {
        .IN = {IN1, IN2},
        .EN = EN1
    },
    {
        .IN = {IN3, IN4},
        .EN = EN2
    },
};

typedef void (*motor_direction_func_ptr)();

pthread_mutex_t motor_direction_mutex = PTHREAD_MUTEX_INITIALIZER;
uint8_t motor_direction; //현재 진행 방향
motor_direction_func_ptr motor_dir_func_ptr[MOTOR_DIR_END] = {
    motor_streight, motor_right, motor_left, motor_back, motor_allStop
};
// motor_dir_func_ptr라는 함수 포인터 배열에 'MOTOR_DIR_END까지의 열거형 상수에 따라 크기가 결정
// 순서대로 직진, 오른쪽, 왼쪽, 뒤, 멈춤

void motor_set_direction(motor_direction_e direction) //모터 드라이버에 방향 지정하는 부분으로 direction에 따라 동작
{
    if(direction == motor_direction) { //현재 진행방향과 일치하면 그냥 진행(return)
        return;
    }
    // pthread_mutex_lock(&motor_direction_mutex);
    motor_direction = direction; //현재 진행방향과 일치하지 않는다면 진행 방향 지정
    // pthread_mutex_unlock(&motor_direction_mutex);
}

void motor_forward(motor_type_e type) { //그냥 말 그대로 전진하는 부분으로 왼, 오 바퀴 조절하여 좌우 회전도 할 수 있음(기본적으로 필요한 함수)
    digitalWrite(motor_cfg[type].IN[1], LOW);
    digitalWrite(motor_cfg[type].IN[0], HIGH);
    // pwmWrite(motor_cfg[type].EN, SPEED_DEFAULT); // PWM duty cycle 설정 (0~1024)
}

void motor_reverse(motor_type_e type) { //후진하는 부분
    digitalWrite(motor_cfg[type].IN[1], HIGH);
    digitalWrite(motor_cfg[type].IN[0], LOW);
    // pwmWrite(motor_cfg[type].EN, SPEED_DEFAULT); // PWM duty cycle 설정 (0~1024)
}
void motor_stop(motor_type_e type) { //멈추는 부분
    digitalWrite(motor_cfg[type].IN[0], LOW);
    digitalWrite(motor_cfg[type].IN[1], LOW);
}
void motor_allStop() {
    digitalWrite(motor_cfg[MOTOR_LEFT].IN[0], LOW);
    digitalWrite(motor_cfg[MOTOR_LEFT].IN[1], LOW);
    digitalWrite(motor_cfg[MOTOR_RIGHT].IN[0], LOW);
    digitalWrite(motor_cfg[MOTOR_RIGHT].IN[1], LOW);
}

void motor_left()
{
    // 왼쪽으로 꺾기 위해선 오른쪽 바퀴만 움직여야함
    // motor_forward에 오른쪽 바퀴를 주어 오른쪽 움직이게 하고 motor_stop에 왼쪽 바퀴를 주어 왼쪽을 멈추게 함
    motor_forward(MOTOR_RIGHT);
    motor_stop(MOTOR_LEFT);
}

void motor_right()
{
    // 오른쪽으로 꺾기 위해선 왼쪽 바퀴만 움직여야함
    // motor_forward에 왼쪽 바퀴를 주어 왼쪽 움직이게 하고 motor_stop에 오른쪽 바퀴를 주어 오른쪽을 멈추게 함
    motor_forward(MOTOR_LEFT);
    motor_stop(MOTOR_RIGHT);
}

void motor_streight()
{   
    // 왼, 오 모두 motor_forward에
    motor_forward(MOTOR_RIGHT);
    motor_forward(MOTOR_LEFT);
}

void motor_back()
{
    motor_reverse(MOTOR_RIGHT);
    motor_reverse(MOTOR_LEFT);
}

int motor_init() // main파일에서 시작할 때 호출함 올바르게 다 처리되면 0을 반환하도록 함
{
    if (wiringPiSetup() == -1) {
        printf("WiringPi setup failed.\n");
        return -1;
    }

    for(int i =0 ; i<2; i++)
    {
        // 왼,오른쪽 모터에 대해서 각각 어느것을 OUTPUT으로, 어느것을 PWM_OUTPUT으로 설정할 것인지 설정
        // OUTPUT이란 모터나 LED와 같이 출력을 위한 것에 대한 응답을 보내는 것 (INPUT은 센서에서 읽어오는 것)
        pinMode(motor_cfg[i].IN[0], OUTPUT);
        pinMode(motor_cfg[i].IN[1], OUTPUT);
        pinMode(motor_cfg[i].EN, PWM_OUTPUT);
    }

    pwmSetMode(PWM_MODE_MS);  // 마크-스페이스 모드로 설정
    // 기본의 경우 벨런스 모드인데, 이는 가변 주파수라 마크-스페이스 모드로 하여 고정 주파수 사용하는 듯

    pwmSetClock(1920);  // 클럭 설정 (기본 값은 1920, 변경 가능)
    pwmSetRange(200);  // 레인지 설정 (기본 값은 200, 변경 가능)
    //위의 경우 기본적인 값임

    // 모터 속도 조절
    pwmWrite(motor_cfg[MOTOR_RIGHT].EN, 5);
    pwmWrite(motor_cfg[MOTOR_LEFT].EN, 200);
    // 왼쪽과 오른쪽이 서로 반대로 달려있기 때문에 5와 200이 얼추 맞음 하지만 약간 차이가 있음

    return 0;
}

void* motor_run(void *arg) //thread로 불러서 작동하는 것
{
    motor_allStop();

    while(true)
    {
        for(int i =0 ; i< 10; i++){
            motor_dir_func_ptr[motor_direction](); //motor_direction은 현재 진행 방향
            delay(1);
            motor_dir_func_ptr[MOTOR_DIR_STOP]();
            // 왼,오 속도의 약간 차이가 있기 때문에 한번씩 다시 시작하는 방식임
        }

        // if((autonomous_onOff_flag == AUTONOMOUS_ON_FLAG) && ((motor_direction != MOTOR_DIR_STOP) || (motor_direction != MOTOR_DIR_BACK))) {
        //     motor_set_direction(MOTOR_DIR_FORD);
        // }
    }
}