#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "timeStamp.h"

int print_time(int Line) {

    struct timeval tv;
    struct tm *timeinfo;
    char buffer[80];

    // 현재 시간을 얻습니다(초와 마이크로초 단위).
    gettimeofday(&tv, NULL);

    // 초 부분의 시간 정보를 얻습니다.
    timeinfo = localtime(&tv.tv_sec);

    // 시간 정보를 문자열로 변환합니다(마이크로초 부분 제외).
    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);

    // 밀리초 부분을 계산합니다.
    int milliseconds = tv.tv_usec / 1000;

    // 변환된 시간 문자열과 밀리초 부분을 함께 출력합니다.
    printf("%d: %s.%03d\n", Line, buffer, milliseconds);

    return 0;
}
