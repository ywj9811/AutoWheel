
#include <stdio.h>
#include <stdlib.h>

#include "autonomous.h"
#include "motor_driver.h"
#include "main.h"

//진행 조절(방향) 하는 부분
static void autonomous_motor_ctrl(motor_direction_e direction)
{
    motor_set_direction(direction); //모터드라이버에 방향 지정해줌(왼, 오 진행은 한만큼 체크해서 다시 돌려야 하니 --와 ++를 진행한다.)
    if(direction == MOTOR_DIR_RIGH) {
        motor_left_cnt--; // 오른쪽으로 진행하니 left_cnt--진행
    } else if(direction == MOTOR_DIR_LEFT) {
        motor_left_cnt++; // 왼쪽으로 진행하니 left_cnt++진행
    } else {
        // 직진은 원래 하던 일이니 카운트는 따로 필요 없음
        /* No Action */
    }
}

// 자율 주행을 하며 장애물을 감지하는 부분
static void autonomous_detect_obstacle(int angle, int obstacle_len)
{
    int obstacleStart, obstacleEnd;

    obstacleStart = angle; // angle을 받았은 것을 이용(start_index, count)
    obstacleEnd = obstacleStart + obstacle_len;

    if(obstacleStart < 0 || obstacleEnd > LIDAR_DETECT_ANGLE) {
        printf("invalid angle %d\n",angle);
        return;
    }
    // 각도가 잘못된 것은 오류기에 종료

    printf("Angle : %d, obstacle_len %d  : ",angle,obstacle_len);
    if(obstacleStart < 10 && obstacleEnd > (LIDAR_DETECT_ANGLE - 10)) {
        printf("Cannot avoid the obstacle. Stopping\n");
        autonomous_motor_ctrl(MOTOR_DIR_STOP);
        return;
    }
    if(obstacleEnd < LIDAR_THETA_0_OFFSET) { // 중앙을 중심으로 왼쪽에 있는지 체크하는 것임 (LIDAR_THETA_0_OFFSET가 중앙)
        printf("Autonomous right driving\n");
        autonomous_motor_ctrl(MOTOR_DIR_RIGH);
    } else if(obstacleStart > LIDAR_THETA_0_OFFSET) { //중앙을 중심으로 오른쪾에 존재하는지 체크
        printf("Autonomous left driving\n");
        autonomous_motor_ctrl(MOTOR_DIR_LEFT);
    } else {
        if((LIDAR_THETA_0_OFFSET - obstacleStart) >= (obstacleEnd - LIDAR_THETA_0_OFFSET)) { // 중앙이 걸려있을 때 -> 왼, 오 결정하는 부분
            printf("Autonomous right driving\n");
            autonomous_motor_ctrl(MOTOR_DIR_RIGH); //만약 조건이 참이면 이는 오른쪽으로 피하는 것이 합리적
        } else {
            printf("Autonomous left driving\n");
            autonomous_motor_ctrl(MOTOR_DIR_LEFT); //만약 조건이 참이면 이는 왼쪽 피하는 것이 합리적
        } 
    }
}

//왼/오른쪽으로 돈 만큼 다시 원상복구 하는 과정
static void autonomous_lane_centering() 
{
    if(motor_left_cnt > 0) { // 왼쪽으로 돌은 경우
        autonomous_motor_ctrl(MOTOR_DIR_RIGH); 
        // autonomous_motor_ctrl 여기서 들어온 매개변수에 따라 lect_cnt-- 혹은 ++를 진행함 따라서 cnt가 0이 되어 원복할때까지 진행
        printf("Autonomous lane centering turn right+\n");
    } else if(motor_left_cnt < 0) {
        autonomous_motor_ctrl(MOTOR_DIR_LEFT);
        // autonomous_motor_ctrl 여기서 들어온 매개변수에 따라 lect_cnt-- 혹은 ++를 진행함 따라서 cnt가 0이 되어 원복할때까지 진행
        printf("Autonomous lane centering turn left+\n");
    } else {
        autonomous_motor_ctrl(MOTOR_DIR_FORD);
        // 원복을 완료하여 cnt==0인 경우 직진으로 진행한다
        //printf("Autonomous lane centering go straight\n");
    }
}

// rplidar_driver.cpp에서 호출되어 처리하는 부분으로 장애물의 위치와 길이를 받음
void autonomous_determine_direction(int angle, int obstacle_len)
{
    static int force_straight = 0;
    if(obstacle_len == 0) {  
        // 장애물의 길이가 0 즉 없다면 직진 하다가(force_straight < 4) 다시 원래의 상태로 돌아감(else)
        if(force_straight < 4) {
            autonomous_motor_ctrl(MOTOR_DIR_FORD);
            force_straight++;
        } else {
            autonomous_lane_centering();
        }
    } else {
        force_straight = 0;
        autonomous_detect_obstacle(angle, obstacle_len);
    }
    
}
