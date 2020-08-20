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

void DesiredDistance(int SettingSpeed, int SettingDistance);
// Postcondition : SettingSpeed�� �ӵ�(����� ����, ������ ����)�� SettingDistance�� step ��ŭ �����δ�.

int RoundAbout_isStart(const int Distance1);
// Return : �տ� ������ ������ ��� ���� ������ �� ����ϸ� 1�� ��ȯ�Ѵ�.

int RoundAbout_isDelay(const int Distance1);
// Return : �տ� ������ ��Ÿ�� ��� �켱 �����ϸ� 1�� ��ȯ�Ѵ�. �� �� ���� ������ �� ����ϸ� 0�� ��ȯ�Ѵ�.

int RoundAbout_isEnd(const int Distance1, const int Distance2);
// Return : ��, �� �Ÿ������� ���� ���ݸ�ŭ �ƹ��͵� ������ �ʴ´ٸ� 1�� ��ȯ�Ѵ�.

//int Tunnel_isTunnel(const int Distance1, const int Distance2, const int Distance3, const int Distance4);

int Tunnel_isEnd(const int Distance1, const int Distance2, const int Distance3, const int Distance4);
// Return : ��, �� ������ ������� �������� ������ 1�� ��ȯ�Ѵ�.

int Tunnel_SteerVal(const int Distance1, const int Distance2);
// Return : �� �� ������ �Ÿ����� �̿��Ͽ� �˸��� ���Ⱒ�� ��ȯ�Ѵ�.

#ifdef __cplusplus
}



#endif /* __cplusplus */

#endif //CONTROL_MISSION_H