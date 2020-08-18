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
 *  차체 컨트롤
 *******************************************************************************
 */ 
void CarControlInit(void);
    //PreCondition  : 한번도 실행 된 적이 없어야한다.
    //PostCondition : 제어보드와 통신을 시작하여, 구동계를 제어할 수 있게된다.

void CarLight_Write(char status);   //ㅁㄴㅇ
    //PreCondition  : CarControllnit()이 실행되었어야함. 이하 모든함수 동일.
    //PostCondition : status = (ALL_ON, FRONT_ON, REAR_ON, ALL_OFF)
    //                  전조등, 후미등 상태를 변경한다.

void Alarm_Write(char status);
    //PostCondition : status = (ON, OFF)
    //                  부저가 울린다.(off전까지 쭉)

void Winker_Write(char status);
    //PostCondition : status = (ALL_ON, RIGHT_ON, LEFT_ON, ALL_OFF)
    //                  방향지시등 상태를 변경한다.

/*******************************************************************************
 *  바퀴(속도, 위치) 컨트롤
 *******************************************************************************
 */
char SpeedControlOnOff_Read(void);
    //Return : 속도제어상태를 0x01(CONTROL), 0x00(OFF)로 반환한다.

void SpeedControlOnOff_Write(char status);
    //PostCondition : 속도제어를 활성화(status = CONTROL)한다.

signed short DesireSpeed_Read(void);
void DesireSpeed_Write(signed short speed);
    //PostCondition : write) 목표 속도값을 설정한다.
    //Return : read) 목표 속도값을 반환한다.

/**********
* PID제어란 Propotional(비례) Integral(적분) Derivative(미분)의 약자
***********
*/
unsigned char SpeedPIDProportional_Read(void);
void SpeedPIDProportional_Write(unsigned char gain);

unsigned char SpeedPIDIntegral_Read(void);
void SpeedPIDIntegral_Write(unsigned char gain);

unsigned char SpeedPIDDifferential_Read(void);
void SpeedPIDDifferential_Write(unsigned char gain);

/**********
* 위치제어란 엔코더를 사용하여 바퀴의 회전수(195step = 1바퀴)를 조절한다.
***********
*/
char PositionControlOnOff_Read(void);
    //Return : 속도제어상태를 0x01(CONTROL), 0x00(OFF)로 반환한다.

void PositionControlOnOff_Write(char status);
    //PostCondition : 속도제어를 활성화(status = CONTROL)한다.

unsigned char PositionProportionPoint_Read(void);
void PositionProportionPoint_Write(unsigned char gain);

signed int DesireEncoderCount_Read(void);
void DesireEncoderCount_Write(signed int position);
    //PostCondition : write) 목표 엔코더값을 설정한다.
    //                  예를들어 현재 엔코더값 100, 목표엔코더값 500이면
    //                  400step 만큼 움직인다.
    //Return : read) 목표 엔코더값을 반환한다.
                       

signed int EncoderCounter_Read(void);
void EncoderCounter_Write(signed int position);
    //PostCondition : write) 현재 엔코더값을 설정한다.
    //                  예를들어 현재 엔코더값 100, 목표엔코더값 500이면
    //                  400step 만큼 움직인다.
    //Return : read) 현재 엔코더값을 반환한다.

/*******************************************************************************
 *  서보모터 컨트롤
 *******************************************************************************
 */
signed short SteeringServoControl_Read(void);
signed short CameraXServoControl_Read(void);
signed short CameraYServoControl_Read(void);
    //Return : 서보모터의 현재angle값을 반환한다

void SteeringServoControl_Write(signed short angle);
void CameraXServoControl_Write(signed short angle);
void CameraYServoControl_Write(signed short angle);
    //PostCondition : 서보모터의 각도를 조절한다
            //default == 1500
            //steering 범위 (? ~ 1500 ~ ?)
            //cameraX 범위 (? ~ 1500 ~ ?)
            //cameraY 범위 (? ~ 1500 ~ ?)

/*******************************************************************************
 *  센서 컨트롤
 *******************************************************************************
 */
unsigned char LineSensor_Read(void);
    //Return : 하단 적외선 라인센서 값을 반환한다.

int DistanceSensor(int channel);
    //Return : 적외선 센서 값을 반환한다. (거리가 가까울수록 숫자높음)
    //채널 1~6(6개) 전방(1 channel) 우측(2, 3) 후방(4) 좌측(5, 6)

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
#endif /* __cplusplus */

#endif


/*@}*/
