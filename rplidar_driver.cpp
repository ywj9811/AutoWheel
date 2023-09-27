#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#include <unistd.h>
#include <pthread.h>

static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

#include "rplidar_driver.h"
#include "autonomous.h"
#include "main.h"
#include "timeStamp.h"

using namespace sl;

static bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#define LIDAR_DETECT_LEFT ((360-LIDAR_DGREE_OFFSET) - (LIDAR_DETECT_ANGLE/2))
#define LIDAR_DETECT_RIGHT (LIDAR_DETECT_ANGLE/2) - LIDAR_DGREE_OFFSET
#define LIDAR_DATA_BUF_OFFSET (LIDAR_DETECT_ANGLE/2) - LIDAR_DGREE_OFFSET
char data[LIDAR_DETECT_ANGLE+1] = {0};

ILidarDriver * drv;

void rplidar_finished()
{
    if(drv) {
        delete drv;
        drv = NULL;
    }
}

// 장애물 탐지했다면 동작하는 부분
void rplidar_detect_obstacle(bool find_obstacle)
{
    // '-'로 채워진 길이를 계산
    int count = 0;
    int space_count = 0; // 연속 공백 카운터
    int started = 0; // '-' 문자 발견 플래그
    int start_index = -1; // 시작 인덱스
    int end_index = -1; // 끝 인덱스
    
    for(int i = 0; i < LIDAR_DETECT_ANGLE; i++) { //총 82개로 왼쪽 ~ 중간(0) ~ 오른쪽 이렇게 판별한다.
        if(data[i] == '-') {
            if(!started) {
                start_index = i; // 시작 인덱스 설정
                started = 1; // '-' 문자 발견하여 started이미 저장됨을 표시
            }
            end_index = i; // 끝 인덱스 갱신
            count++;
            space_count = 0; // 공백 카운터 초기화
        }
        else if(started) {
            space_count++;
            count++; // 공백 카운트
            if(space_count >= 8) {
                break; // 연속 공백이 8번 나오면 루프를 종료
            }
        }
    }
    
    autonomous_determine_direction(start_index,count);
    // 이제 진행 방향을 결정할 수 있도록 함수 호출(어디에 장애물이 있고, 얼마나 있는지 보내줌)
}

// 라이다 시작하는 부분으로 정해진 방법
int rplidar_init()
{
    const char opt_channel_param_first[50] = "/dev/ttyUSB0";
	sl_u32         opt_channel_param_second = 115200;
    sl_result     op_result;
    IChannel* _channel;
     
    // create the driver instance
	drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        return -1;
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
    if (SL_IS_OK((drv)->connect(_channel))) {
        op_result = drv->getDeviceInfo(devinfo);

        if (SL_IS_OK(op_result)) 
        {
	        connectSuccess = true;
        }
        else{
            delete drv;
			drv = NULL;
        }
    }
    
    if (!connectSuccess) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_channel_param_first);	
	
        rplidar_finished();
        return -1;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);
    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        rplidar_finished();
        return -1;
    }

    drv->setMotorSpeed();

    // start scan...
    drv->startScan(0,1);
    // fetech result and print it out...
    
    return 0;
}

// rplidar_run이 쓰레드에서 호출되는 부분이다.(이게 시작됨)
void* rplidar_run(void *arg)
{
    sl_result     op_result;

    while(1) {    
        if(rplidar_init() != 0) {
            printf("rplidar init fail\n");
        } else {
            break;
        }

        sleep(3);
    }
    //rplidar시작시키는 부분

    //무한 루프를 돌리며 진행
    while (1) {
        
        pthread_mutex_lock(&autonomous_onOff_mutex);
        // 이 mutex_lock부터 unlock까지 자율 주행의 부분에 끼어들 수 없다. (다른 스레드와 공통적으로 다룰 수 있는 부분을 점유할 수 있음)

        while(autonomous_onOff_flag == AUTONOMOUS_OFF_FLAG) {
            pthread_cond_wait(&autonomous_onOff_cond,&autonomous_onOff_mutex);
            // wait을 호출하면 자동으로 unlock되며 진행되고 권한을 조이스틱에게 넘겨줌 
        } 
        //만약 조이스틱이 동작을 하는 경우에 기다림
        //조이스틱이 행위를 넘겨주는 순간 pthread_cond_wait가 깨어날 조건을 만족하면 깨어나고 자동적으로 lock진행

        sl_lidar_response_measurement_node_hq_t nodes[8192]; // 이 nodes가 라이다에서 받아오는 하나하나의 정보로 최대인 8192만큼
        size_t   count = _countof(nodes);
        
        op_result = drv->grabScanDataHq(nodes, count);
        // lidar에서 탐지한 결과를 가져온다. sl_lidar_response_measurement_node_hq_t 형태의 배열에 저장

        if (SL_IS_OK(op_result)) { // Lidar에서 스캔 데이터를 정상적으로 받아온 경우를 확인
            drv->ascendScanData(nodes, count);
            // Lidar 스캔 데이터를 처리하기 전에, 스캔 데이터를 각도 순으로 정렬하는 역할 이렇게 하면 데이터를 더 쉽게 처리할 수 있다.

            for(int i =0; i<LIDAR_DETECT_ANGLE; i++)
            {
                data[i] = ' ';
            } // 배열 초기화

            int theta_cnt = -1;
            bool find_obstacle = false;
            for (int pos = 0; pos < (int)count ; ++pos) { 
                int theta = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                // Lidar의 api에 있는 데이터인 angle_z_q14를 변환하여 실제 각도를 구함
                // 그렇게 실제 각도만 구해서 거기에 해당하는 내용만 아래 과정 (data[(theta + LIDAR_DATA_BUF_OFFSET)%360] = '-';)을 통해 data배열에 넣음
                
                if(theta <= theta_cnt) { 
                    //만약 아직 저장시점이 아니라면 continue
                    continue;
                } else {
                    theta_cnt++;
                    //저장한다면 다음 각도를 위해 theta_cnt를 늘려줌
                }

                if(theta >= LIDAR_DETECT_LEFT || theta <= LIDAR_DETECT_RIGHT){
                    // 만약 측정 범위 내부에 알맞게 들어와있다면 진행
                    int dist = nodes[pos].dist_mm_q2/4.0f;
                    // dist_mm_q2 또한 Lidar에서 제공해주는 거리 데이터 변수
                    if(dist > LIDAR_DETECT_DIST || dist == 0) { 
                        // 측정 범위보다 멀거나(현재 35cm) 아무것도 감지가 안된다면(0) 진행
                        continue;
                    } 
                    // 넘어오면 장애물이 존재하는 것 -> true로 함
                    find_obstacle = true;
                    data[(theta + LIDAR_DATA_BUF_OFFSET)%360] = '-';
                    // 장애물 표시
                }
            }
            data[LIDAR_DETECT_ANGLE] = '\0';
            
            printf("%s\n",data);
            rplidar_detect_obstacle(find_obstacle);
            // 감지했다면 rplidar_detect_obstacle함수 진행
                        
            memset(data,0,sizeof(data));
        }

        pthread_mutex_unlock(&autonomous_onOff_mutex);
        // 여기까지 임계구역 차지
        usleep(1000);
    }
    

    drv->stop();
	delay(200);
	
    drv->setMotorSpeed(0);
    // done!
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return NULL;
}