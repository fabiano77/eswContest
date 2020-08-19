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

void DesiredDistance(int SettingSpeed, int SettingDistance);
// Postcondition : SettingSpeed의 속도(양수는 전진, 음수는 후진)로 SettingDistance의 step 만큼 움직인다.


#ifdef __cplusplus
}

int RoundAbout_isStart(const int Distance1);
// Return : 1 이면 다음 분기로 진행

int RoundAbout_isDelay(const int Distance1, int& speed);
// return 1 이면 정지

int RoundAbout_End(const int Distance1, const int Distance2);
// return 1 이면 분기 빠져나감

#endif /* __cplusplus */

#endif //CONTROL_MISSION_H