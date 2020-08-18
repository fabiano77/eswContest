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





#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //CONTROL_MISSION_H