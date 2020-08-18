#ifndef _PARAM_HANDLE_
#define _PARAM_HANDLE_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */
#define OFF         0x00
#define ON          0xFF

// Light
#define ALL_OFF     0x00
#define REAR_ON     0x02
#define FRONT_ON    0x01
#define ALL_ON      0x03
#define RIGHT_ON    0x01
#define LEFT_ON     0x02

// Speed and position controller setting
#define UNCONTROL   0x00
#define CONTROL     0x01
#define CHECKSUMERROR 0xFEFE

/*******************************************************************************
 *  Functions
 *******************************************************************************
 */

/*******************************************************************************
 *  ��ü ��Ʈ��
 *******************************************************************************
 */ 
void CarControlInit(void);
    //PreCondition  : �ѹ��� ���� �� ���� ������Ѵ�.
    //PostCondition : ������ ����� �����Ͽ�, �����踦 ������ �� �ְԵȴ�.

void CarLight_Write(char status);   //������
    //PreCondition  : CarControllnit()�� ����Ǿ������. ���� ����Լ� ����.
    //PostCondition : status = (ALL_ON, FRONT_ON, REAR_ON, ALL_OFF)
    //                  ������, �Ĺ̵� ���¸� �����Ѵ�.

void Alarm_Write(char status);
    //PostCondition : status = (ON, OFF)
    //                  ������ �︰��.(off������ ��)

void Winker_Write(char status);
    //PostCondition : status = (ALL_ON, RIGHT_ON, LEFT_ON, ALL_OFF)
    //                  �������õ� ���¸� �����Ѵ�.

/*******************************************************************************
 *  ����(�ӵ�, ��ġ) ��Ʈ��
 *******************************************************************************
 */
char SpeedControlOnOff_Read(void);
    //Return : �ӵ�������¸� 0x01(CONTROL), 0x00(OFF)�� ��ȯ�Ѵ�.

void SpeedControlOnOff_Write(char status);
    //PostCondition : �ӵ���� Ȱ��ȭ(status = CONTROL)�Ѵ�.

signed short DesireSpeed_Read(void);
void DesireSpeed_Write(signed short speed);
    //PostCondition : write) ��ǥ �ӵ����� �����Ѵ�.
    //Return : read) ��ǥ �ӵ����� ��ȯ�Ѵ�.

/**********
* PID����� Propotional(���) Integral(����) Derivative(�̺�)�� ����
***********
*/
unsigned char SpeedPIDProportional_Read(void);
void SpeedPIDProportional_Write(unsigned char gain);

unsigned char SpeedPIDIntegral_Read(void);
void SpeedPIDIntegral_Write(unsigned char gain);

unsigned char SpeedPIDDifferential_Read(void);
void SpeedPIDDifferential_Write(unsigned char gain);

/**********
* ��ġ����� ���ڴ��� ����Ͽ� ������ ȸ����(195step = 1����)�� �����Ѵ�.
***********
*/
char PositionControlOnOff_Read(void);
    //Return : �ӵ�������¸� 0x01(CONTROL), 0x00(OFF)�� ��ȯ�Ѵ�.

void PositionControlOnOff_Write(char status);
    //PostCondition : �ӵ���� Ȱ��ȭ(status = CONTROL)�Ѵ�.

unsigned char PositionProportionPoint_Read(void);
void PositionProportionPoint_Write(unsigned char gain);

signed int DesireEncoderCount_Read(void);
void DesireEncoderCount_Write(signed int position);
    //PostCondition : write) ��ǥ ���ڴ����� �����Ѵ�.
    //                  ������� ���� ���ڴ��� 100, ��ǥ���ڴ��� 500�̸�
    //                  400step ��ŭ �����δ�.
    //Return : read) ��ǥ ���ڴ����� ��ȯ�Ѵ�.
                       

signed int EncoderCounter_Read(void);
void EncoderCounter_Write(signed int position);
    //PostCondition : write) ���� ���ڴ����� �����Ѵ�.
    //                  ������� ���� ���ڴ��� 100, ��ǥ���ڴ��� 500�̸�
    //                  400step ��ŭ �����δ�.
    //Return : read) ���� ���ڴ����� ��ȯ�Ѵ�.

/*******************************************************************************
 *  �������� ��Ʈ��
 *******************************************************************************
 */
signed short SteeringServoControl_Read(void);
signed short CameraXServoControl_Read(void);
signed short CameraYServoControl_Read(void);
    //Return : ���������� ����angle���� ��ȯ�Ѵ�

void SteeringServoControl_Write(signed short angle);
void CameraXServoControl_Write(signed short angle);
void CameraYServoControl_Write(signed short angle);
    //PostCondition : ���������� ������ �����Ѵ�
            //default == 1500
            //steering ���� (? ~ 1500 ~ ?)
            //cameraX ���� (? ~ 1500 ~ ?)
            //cameraY ���� (? ~ 1500 ~ ?)

/*******************************************************************************
 *  ���� ��Ʈ��
 *******************************************************************************
 */
unsigned char LineSensor_Read(void);
    //Return : �ϴ� ���ܼ� ���μ��� ���� ��ȯ�Ѵ�.

int DistanceSensor(int channel);
    //Return : ���ܼ� ���� ���� ��ȯ�Ѵ�. (�Ÿ��� �������� ���ڳ���)
    //ä�� 1~6(6��) ����(1 channel) ����(2, 3) �Ĺ�(4) ����(5, 6)

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

#endif


/*@}*/
