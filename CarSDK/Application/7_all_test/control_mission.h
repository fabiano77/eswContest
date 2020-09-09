#ifndef CONTROL_MISSION_H
#define CONTROL_MISSION_H

#ifdef __cplusplus
extern "C" {
#endif


	/******************** enumerator ********************/
	enum MissionState
	{
		NONE,
		READY,
		REMAIN,
		DONE
	};

	enum StartState
	{
		START_S,
		WAIT_S,
		FRONT_CLOSE_S,
		FRONT_OPEN_S
	};

	enum ParkingState
	{
		NONE_P,
		FIRST_WALL,
		DISTANCE_CHECK,
		SECOND_WALL,
		PARKING_START,
		DONE_P = NONE_P
	};

	enum HorizontalStep
	{
		FINISH,
		FIRST_BACKWARD,
		FIRST_FORWARD,
		SECOND_BACKWARD,
		SECOND_FORWARD,
		ESCAPE,
		ESCAPE_2,
		ESCAPE_3
	};

	enum VerticalStep
	{
		FINISH_V,
		FIRST_BACKWARD_V,
		SECOND_BACKWARD_V,
		OVER_STEER_V,
		UNDER_STEER_V,
		RIGHT_FRONT_V,
		LEFT_FRONT_V,
		FIRST_FORWARD_V,
		SECOND_FORWARD_V
	};

	enum OvertakeState
	{
		NONE_O,
		FRONT_DETECT,
		SIDE_ON,
		SIDE_OFF,
		DONE_O = NONE_O
	};

	enum RoundaboutState
	{
		NONE_R,
		WAIT_R,
		ROUND_GO_1,
		ROUND_STOP,
		ROUND_GO_2,
		DONE_R
	};

	enum CameraVerticalState
	{
		CAMERA_UP,	//��ֹ� �ν��� ���� �ø�����
		CAMERA_DOWN //���� ���� -->�� �κ� ���� �ʿ� MS
	};

	enum DirectionState
	{
		LEFT,
		RIGHT,
		STOP //�տ� ��ֹ��� �ִٸ� ��ž(overtaking�� on�� ����)
	};

	enum SignalLightState
	{
		DETECTION_FINISH,
		DETECT_RED,
		DETECT_YELLOW,
		DETECT_GREEN
	};

	/******************** mission struct ********************/
	struct Parking
	{
		bool frontRight;
		bool rearRight;
		bool bparking;		 // ���� �� �Ÿ� ���� ����� ���� ����
		bool verticalFlag;	 // ���� ���� Ȱ��ȭ�� ��Ÿ���� �÷���
		bool horizontalFlag; // ���� ���� Ȱ��ȭ�� ��Ÿ���� �÷���
	};

	struct Overtaking
	{
		bool sideSensorFlag;				   // ������ ���̵� Ž�� Ȱ��ȭ �÷���
		enum DirectionState headingDirection;  //������ �̵����� ����
		enum CameraVerticalState updownCamera; //ī�޶� ���� �ø��� ���� �����ϴ� �κ�
		char leftFlag;
		char rightFlag;
	};

	struct Finish
	{
		bool checkFront; //���� ���� ����� �ľ�
		int distEndLine; //��¼������� �Ÿ�
	};

	struct SignalLight
	{
		enum SignalLightState state;
		int Accumulation_greenVal;
		int ignore_frame;
		int finalDirection;
	};

	/******************** thread struct ********************/
	struct MissionData
	{
		uint32_t loopTime; // mission ������ ���� �ð�
		bool broundabout;
		bool btunnel;
		bool overtakingFlag; // �߿����� �÷��� ->MS ���� overtaking struct �߰��� ��
		bool changeMissionState;
		bool checkWhiteLineFlag;
		int frame_priority;
		int finish_distance;

		struct Parking parkingData;			// ������ �ʿ��� �÷��׸� ��� ����ü
		struct Overtaking overtakingData;	// �߿��� �ʿ��� �÷��� ��� ����ü
		struct SignalLight signalLightData; // ��ȣ� �ʿ��� ������ ��� ����ü
		struct Finish finishData;
		enum MissionState ms[9]; //
	};

	struct ControlData
	{
		int steerVal;
		int cameraY;
		int desireSpeedVal;
		int beforeSpeedVal;
		int settingSpeedVal;
		unsigned short lightFlag;
	};

	struct ImgProcessData
	{
		uint32_t loopTime;		// img ������ ���� �ð�
		bool dump_request;		// ������û
		bool bskip;
		bool bvideoRecord;		// ������ ��ȭ ����
		bool bvideoSave;		// ������ ���� ����
		bool bcalibration;		// Ķ���극�̼�
		bool bdebug;			// ����׸�� ON/OFF
		bool btopview;			// ž�� ON/OFF
		bool bmission;			// �̼����� ON/OFF (�����ν� ������� �ʰԵ�)
		bool bauto;				// �ڵ� ���� ON/OFF
		bool bspeedControl;		// �ڵ� ������ �ӵ����� ON/OFF
		bool bwhiteLine;		// �ڵ� ������ ��� �� Ž�� ON/OFF
		bool bprintString;		// �������̿� ���ڿ� ǥ�� ON/OFF
		bool bprintMission;		// �������̿� �̼����� ǥ�� ON/OFF
		bool bprintSensor;		// �������̿� ������ ǥ�� ON/OFF
		bool bprintTire;		// �������̿� �������� ǥ�� ON/OFF
		bool bdark;				// �ͳ� Ž�� ON/OFF
		bool bcheckPriority;	// �켱���� ǥ���� Ž�� ON/OFF
		bool bcheckSignalLight; // ��ȣ�� Ž�� ON/OFF
		bool bcheckFinishLine;	// �ǴϽö��� Ž�� ON/OFF
		char missionString[20]; // �������̿� ǥ���� ���ڿ�
		int topMode;			// ž�� ��� (0, 1, 2)
		int debugMode;			// ����� ���(0~ 7)
	};


int DistanceSensor_cm(int channel);
//Return : ���ܼ� ���� ��(cm)�� ��ȯ�Ѵ�. (�Ÿ��� �������� ���ڳ���)

int sensor_dist(int channel, int input);
// Return : ���ܼ� ���� ���� ������ �������� �����Ͽ� cm ������ ��ȯ�Ѵ�.

signed int Encoder_Read(void);
// return : ���� ���ڴ� ���� ��ȯ�Ѵ�

int StopLine(int Lineflag);
// Return : Lineflag���� 1�� ������ ���� �� ũ�� �������� ������ ������ �Ǵ��ϰ� true�� ��ȯ�Ѵ�.

int STOP_WhiteLine(int Lineflag);

void DesiredDistance(int SettingSpeed, int SettingDistance, int SettingSteering);
// Postcondition : SettingSpeed�� �ӵ�(����� ����, ������ ����)�� SettingDistance�� step ��ŭ �����δ�.

void onlyDistance(int SettingSpeed, int SettingDistance);
//ds

int RoundAbout_isStart(const int Distance1);
//

int Tunnel_isEnd(const int Distance1, const int Distance2, const int Distance3, const int Distance4);
// Return : ��, �� ������ ������� �������� ������ 1�� ��ȯ�Ѵ�.

int Tunnel_SteerVal(const int Distance1, const int Distance2);
// Return : �� �� ������ �Ÿ����� �̿��Ͽ� �˸��� ���Ⱒ�� ��ȯ�Ѵ�.

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

void manualControl(struct ControlData* cdata, char key);

uint32_t timeCheck(struct timeval* tempTime);

#ifdef __cplusplus
}



#endif /* __cplusplus */

#endif //CONTROL_MISSION_H