#ifndef CONTROL_MISSION_H
#define CONTROL_MISSION_H

#ifdef __cplusplus
extern "C" {
#endif

int DistanceSensor_cm(int channel);
//Return : 적외선 센서 값(cm)을 반환한다. (거리가 가까울수록 숫자높음)

int sensor_dist(int channel, int input);
// Return : 적외선 센서 값을 측정값 기준으로 맵핑하여 cm 단위로 반환한다.

int StopLine(int Lineflag);
// Return : Lineflag보다 1로 감지된 값이 더 크면 정지선을 감지한 것으로 판단하고 true를 반환한다.

int STOP_WhiteLine(int Lineflag);

void DesiredDistance(int SettingSpeed, int SettingDistance, int SettingSteering);
// Postcondition : SettingSpeed의 속도(양수는 전진, 음수는 후진)로 SettingDistance의 step 만큼 움직인다.

void onlyDistance(int SettingSpeed, int SettingDistance);
//ds

int RoundAbout_isStart(const int Distance1);
// Return : 앞에 차량이 지나갈 경우 일정 프레임 후 출발하며 1을 반환한다.

int RoundAbout_isDelay(const int Distance1);
// Return : 앞에 차량이 나타날 경우 우선 정지하며 1을 반환한다. 그 후 일정 프레임 후 출발하며 0을 반환한다.

int RoundAbout_isEnd(const int Distance1, const int Distance2);
// Return : 앞, 뒤 거리센서에 일정 간격만큼 아무것도 잡히지 않는다면 1을 반환한다.

//int Tunnel_isTunnel(const int Distance1, const int Distance2, const int Distance3, const int Distance4);

int Tunnel_isStart(const int Distance2, const int Distance6, const int Distance3, const int Distance5);

int Tunnel_isEnd(const int Distance1, const int Distance2, const int Distance3, const int Distance4);
// Return : 앞, 뒤 센서가 순서대로 감지되지 않으면 1을 반환한다.

int Tunnel_SteerVal(const int Distance1, const int Distance2);
// Return : 양 옆 센서의 거리차를 이용하여 알맞은 조향각을 반환한다.
int Tunnel_SteerVal2(const int Distance1, const int Distance2);

void frontLightOnOff(unsigned short lightFlag, int on);
// Postcondition : 인자가 1일경우 전조등 on, 0일경우 off

void rearLightOnOff(unsigned short lightFlag, int on);
// Postcondition : 인자가 1일경우 후미등 on, 0일경우 off

int auto_speedMapping(int steerVal, const int basicSpeed);
// Return :autoSteer 로 도출한 조향값에 따라 속도를 반환해준다.

void buzzer(int numOfTimes, int interval_us, int pulseWidth_us);
// numOfTimes		: 부저가 울리는 횟수
// interval_us		: 부저 사이의 시간간격
// pulseWidth_us	: 부저하나당 울리는 시간

#ifdef __cplusplus
}



#endif /* __cplusplus */

#endif //CONTROL_MISSION_H