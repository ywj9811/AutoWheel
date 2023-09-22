#include <iostream>
#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include "joystick_reader.h"
#include "motor_driver.h"
#include "main.h"

#define JOYSTICK_BTN_HORIZONTAL 6
#define JOYSTICK_BTN_VERTICAL 7
#define JOYSTICK_BTN_A 0

#define JOYSTIC_MAX_VALUE 32767
#define JOYSTIC_MIN_VALUE (-JOYSTIC_MAX_VALUE)

int joy_fd;

// 자율주행 모드인가, 조이스틱 모드인가 이것을 알려주기 위한 부분
static void notify_to_lidar(Autonomous_onOff_e onOff)
{   
    // 현재의 상태에 따라서 조건을 설정한다.
    pthread_mutex_lock(&autonomous_onOff_mutex); //autonomous_onOff_flag 설정 부분은 동시에 발생하면 안되니 lock~unlock까지 점유
    autonomous_onOff_flag = onOff;
    pthread_cond_signal(&autonomous_onOff_cond);

    pthread_mutex_unlock(&autonomous_onOff_mutex);
    
    motor_left_cnt = 0;
    motor_set_direction(MOTOR_DIR_STOP); // 모터는 멈춘다. (조이스틱으면 손으로 할거고, 자율주행이면 알아서 출발 할 것임)
}

// 조이스틱 모드에서 상,하,좌,우 조절하는 부분
static void joystick_ctrl_axis(__u8 number, __s16 value)
{   
    // 수평 방향을 조절하는 버튼을 눌렀냐(number)
    if(number == JOYSTICK_BTN_HORIZONTAL) {
        // 우선 자율주행을 하지 않는다는것을 lidar에 알려준다.
        printf("cause input joystic, autonomous OFF\n");
        notify_to_lidar(AUTONOMOUS_OFF_FLAG);
        
        // 수평의 경우 value가 양수면 오른쪽, 음수면 왼쪽, 0이면 스톱 (0~360도 돌아가는것으로 하면 다양한 숫자가 나오는데 우리는 방향키라)
        if(value > 0) {
            printf("MOTOR_DIR_RIGH\n");
            motor_set_direction(MOTOR_DIR_RIGH);
        } else if(value == 0) {
            printf("MOTOR_DIR_STOP\n");
            motor_set_direction(MOTOR_DIR_STOP);
        } else {
            printf("MOTOR_DIR_LEFT\n");
            motor_set_direction(MOTOR_DIR_LEFT);
        }
    } else if(number == JOYSTICK_BTN_VERTICAL) {
        // 우선 자율주행을 하지 않는다는것을 lidar에 알려준다.
        printf("cause input joystic, autonomous OFF\n");
        notify_to_lidar(AUTONOMOUS_OFF_FLAG);

        // 마찬가지로 수직(상,하) 음수면 뒤로, 양수면 앞으로, 0이면 멈춘다.
        if(value > 0) {
            printf("MOTOR_DIR_BACK\n");
            motor_set_direction(MOTOR_DIR_BACK);
        } else if(value == 0) {
            printf("MOTOR_DIR_STOP\n");
            motor_set_direction(MOTOR_DIR_STOP);
        } else {
            printf("MOTOR_DIR_FORD\n");
            motor_set_direction(MOTOR_DIR_FORD);
        }
    } else {
        // No Action
    }
}

// 버튼이 눌렸을 때 호출되는 함수로 조이스틱을 사용, 사용x를 판단한다.
static void joystick_press_button(__u8 number, __s16 value)
{
    if(number != JOYSTICK_BTN_A || value == 0) {
        return;
    }

    if(autonomous_onOff_flag == AUTONOMOUS_ON_FLAG) { // 만약 현재 자율주행이 켜져있었다면 자율주행을 꺼야함
        notify_to_lidar(AUTONOMOUS_OFF_FLAG);
        printf("cause input joystic, autonomous OFF\n");
    } else {
        notify_to_lidar(AUTONOMOUS_ON_FLAG); // 현재 조이스틱 모드였다면 조이스틱을 꺼야함
        printf("cause input joystic, autonomous ON\n");
    }
}

int joystick_init() {
    joy_fd = open("/dev/input/js0", O_RDONLY);
    if(joy_fd == -1) {
        std::cerr << "Failed to open joystick device." << std::endl;
        return -1;
    }

    autonomous_onOff_flag = AUTONOMOUS_OFF_FLAG;
    
    return 0;
}

// joystick_run이 thread로 호출되어 돌아감
void* joystick_run(void *arg)
{
    while(true) { //무한 반복
        while(true) {
            if(joystick_init() ==0) {
                break;
                // 이벤트 발생하면 동작해서 아래 진행할 수 있음
            }
            sleep(1);
        }
        // 조이스틱 시작 init()진행시 AUTONOMOUS_OFF_FLAG로 설정함 즉, 조이스틱을 키면 자율주행 꺼짐

        struct js_event js; //struct js_event 구조체의 정의는 주로 리눅스 커널의 헤더 파일에
        while(true) {
            if(read(joy_fd, &js, sizeof(struct js_event)) != sizeof(struct js_event)) {
                std::cerr << "Error reading joystick event." << std::endl;
                close(joy_fd);
                break;
            }

            // JS_EVENT_~~는 가져다 쓰는 것 (비트 플래그임)

            switch(js.type & ~JS_EVENT_INIT) { //JS_EVENT_INIT는 linux/joystick.h 헤더 파일에서 정의
                case JS_EVENT_BUTTON: // 버튼이 눌렸을 때 동작하는 것으로 시작, 종료를 나타낸다.

                    //std::cout << "Button " << (int)js.number << " " << (js.value ? "pressed" : "released") << std::endl;
                    joystick_press_button(js.number,js.value);
                    break;
                case JS_EVENT_AXIS: // 방향(상,하,좌,우) 눌렸을 때 작동

                    //std::cout << "Axis " << (int)js.number << " value: " << js.value << std::endl;
                    joystick_ctrl_axis(js.number,js.value);
                    break;
                default:
                    break;
            }
            usleep(1000);
        }
    }
}

void joystick_close()
{
    close(joy_fd);
}
