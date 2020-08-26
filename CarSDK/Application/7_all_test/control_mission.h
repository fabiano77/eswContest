#ifndef CONTROL_MISSION_H
#define CONTROL_MISSION_H

#ifdef __cplusplus
extern "C" {
#endif

int DistanceSensor_cm(int channel);
//Return : ���ܼ� ���� ��(cm)�� ��ȯ�Ѵ�. (�Ÿ��� �������� ���ڳ���)

int sensor_dist(int channel, int input);
// Return : ���ܼ� ���� ���� ������ �������� �����Ͽ� cm ������ ��ȯ�Ѵ�.

int StopLine(int Lineflag);
// Return : Lineflag���� 1�� ������ ���� �� ũ�� �������� ������ ������ �Ǵ��ϰ� true�� ��ȯ�Ѵ�.

int STOP_WhiteLine(int Lineflag);

void DesiredDistance(int SettingSpeed, int SettingDistance, int SettingSteering);
// Postcondition : SettingSpeed�� �ӵ�(����� ����, ������ ����)�� SettingDistance�� step ��ŭ �����δ�.

void onlyDistance(int SettingSpeed, int SettingDistance);
//ds

int RoundAbout_isStart(const int Distance1);
// Return : �տ� ������ ������ ��� ���� ������ �� ����ϸ� 1�� ��ȯ�Ѵ�.

int RoundAbout_isDelay(const int Distance1);
// Return : �տ� ������ ��Ÿ�� ��� �켱 �����ϸ� 1�� ��ȯ�Ѵ�. �� �� ���� ������ �� ����ϸ� 0�� ��ȯ�Ѵ�.

int RoundAbout_isEnd(const int Distance1, const int Distance2);
// Return : ��, �� �Ÿ������� ���� ���ݸ�ŭ �ƹ��͵� ������ �ʴ´ٸ� 1�� ��ȯ�Ѵ�.

//int Tunnel_isTunnel(const int Distance1, const int Distance2, const int Distance3, const int Distance4);

int Tunnel_isStart(const int Distance2, const int Distance6, const int Distance3, const int Distance5);

int Tunnel_isEnd(const int Distance1, const int Distance2, const int Distance3, const int Distance4);
// Return : ��, �� ������ ������� �������� ������ 1�� ��ȯ�Ѵ�.

int Tunnel_SteerVal(const int Distance1, const int Distance2);
// Return : �� �� ������ �Ÿ����� �̿��Ͽ� �˸��� ���Ⱒ�� ��ȯ�Ѵ�.
int Tunnel_SteerVal2(const int Distance1, const int Distance2);

void frontLightOnOff(unsigned short lightFlag, int on);
// Postcondition : ���ڰ� 1�ϰ�� ������ on, 0�ϰ�� off

void rearLightOnOff(unsigned short lightFlag, int on);
// Postcondition : ���ڰ� 1�ϰ�� �Ĺ̵� on, 0�ϰ�� off

int auto_speedMapping(int steerVal, const int basicSpeed);
// Return :autoSteer �� ������ ���Ⱚ�� ���� �ӵ��� ��ȯ���ش�.

void buzzer(int numOfTimes, int interval_us, int pulseWidth_us);
// numOfTimes		: ������ �︮�� Ƚ��
// interval_us		: ���� ������ �ð�����
// pulseWidth_us	: �����ϳ��� �︮�� �ð�

#ifdef __cplusplus
}



#endif /* __cplusplus */

#endif //CONTROL_MISSION_H