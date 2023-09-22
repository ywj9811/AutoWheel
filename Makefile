# 컴파일러 지정
CXX = g++

# 컴파일 플래그 지정 (예: -std=c++11, -Wall 등)
CXXFLAGS = -std=c++11 -Wall

# 추가적인 헤더 파일 경로
INCLUDES = -I../lidar/rplidar_sdk/sdk/include -I../lidar/rplidar_sdk/sdk/src

# 소스 파일(.cpp)을 찾기
SOURCES = $(wildcard *.cpp)

# 오브젝트 파일(.o) 지정
OBJECTS = $(SOURCES:.cpp=.o)

# 실행 파일 이름 지정
EXEC = Autonomous_Wheelchair

# 라이브러리 경로와 라이브러리 지정 (-L: 라이브러리 경로, -l: 라이브러리 이름)
LDFLAGS = -L../lidar/rplidar_sdk/output/Linux/Release -lsl_lidar_sdk -lm -lpthread -lwiringPi

# 기본 타겟은 실행 파일을 만드는 것
all: $(EXEC)

# 실행 파일을 만들기 위해 오브젝트 파일들을 링크
$(EXEC): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJECTS) -o $(EXEC) $(LDFLAGS) -g

# .o 파일 생성 규칙
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@ -g

# 'make clean' 명령을 위한 규칙
clean:
	rm -f $(OBJECTS) $(EXEC)

# .PHONY는 clean이 파일이 아닌 명령임을 나타냄
.PHONY: clean
