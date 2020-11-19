#ifndef CONTROL_MISSION_H
#define CONTROL_MISSION_H

#ifdef __cplusplus
extern "C"
{
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
		CAMERA_UP,	//?��?���? ?��?��?�� ?��?�� ?��린상?��
		CAMERA_DOWN //?��?�� ?��?�� -->?�� �?�? 조정 ?��?�� MS
	};

	enum DirectionState
	{
		LEFT,
		RIGHT,
		STOP //?��?�� ?��?��물이 ?��?���? ?��?��(overtaking?�� on?�� ?���?)
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
		bool bparking;		 // 주차 �? 거리 ?���? 출력?�� ?��?�� �??��
		bool verticalFlag;	 // ?���? 주차 ?��?��?���? ?��????��?�� ?��?���?
		bool horizontalFlag; // ?��?�� 주차 ?��?��?���? ?��????��?�� ?��?���?
	};

	struct Overtaking
	{
		bool sideSensorFlag;				   // 차량?�� ?��?��?�� ?���? ?��?��?�� ?��?���?
		enum DirectionState headingDirection;  //차량?�� ?��?��방향 결정
		enum CameraVerticalState updownCamera; //카메?���? ?���? ?��릴�?? 말�?? 결정?��?�� �?�?
		char leftFlag;
		char rightFlag;
	};

	struct Finish
	{
		int distEndLine; //결승?��까�???�� 거리
		bool checkFront; //?��?�� ?���? ?��????�� ?��?��
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
		struct Parking parkingData;			// 주차?�� ?��?��?�� ?��?��그�?? ?��?�� 구조�?
		struct Overtaking overtakingData;	// 추월?�� ?��?��?�� ?��?���? ?��?�� 구조�?
		struct SignalLight signalLightData; // ?��?��?��?�� ?��?��?�� �??���? ?��?�� 구조�?
		struct Finish finishData;
		enum MissionState ms[9];
		uint32_t loopTime; // mission ?��?��?�� 루프 ?���?

		int frame_priority;
		int finish_distance;
		bool broundabout;
		bool btunnel;
		bool overtakingFlag; // 추월차로 ?��?���? ->MS ?��?�� overtaking struct 추�???�� �?
		bool changeMissionState;
		bool checkWhiteLineFlag;
	};

	struct ControlData
	{
		int steerVal;		// 차량?�� ?��?�� ?��바�?? 조향�?
		int cameraY;		// 차량?�� ?��?�� 카메?��각도
		int desireSpeedVal; // 차량?�� ?��?�� ?��?��
		int beforeSpeedVal; // 차량?�� ?��?�� ?��?��
		int settingSpeedVal;
		unsigned short lightFlag; // 차량?�� ?��?�� ?��조등?��?��
	};

	struct ImgProcessData
	{
		char missionString[50]; // ?��버레?��?�� ?��?��?�� 문자?��
		int debugMode;			// ?��버그 모드(0~ 9)
		int topMode;			// ?���? 모드 (0, 1, 2)
		uint32_t loopTime;		// img ?��?��?�� 루프 ?���?
		bool dump_request;		// ?��?��?���?
		bool bskip;
		bool bvideoRecord;	// ?��?��?�� ?��?�� ?��?��
		bool bvideoSave;	// ?��?��?�� ?��?�� ????��
		bool bcalibration;	// 캘리브레?��?�� ON/OFF
		bool bdebug;		// ?��버그모드 ON/OFF
		bool btopview;		// ?���? ON/OFF
		bool bmission;		// 미션진입 ON/OFF (차선?��?�� ?��?��?���? ?��게됨)
		bool bauto;			// ?��?�� 조향 ON/OFF
		bool bspeedControl; // ?��?�� 조향?�� ?��?��개입 ON/OFF
		bool bwhiteLine;	// ?��?�� 조향?�� ?��?�� ?�� ?���? ON/OFF
		bool bprintString;	// ?��버레?��?�� 문자?�� ?��?�� ON/OFF
		bool bprintMission; // ?��버레?��?�� 미션?���? ?��?��
		bool bprintSensor;	// ?��버레?��?�� ?��?���? ?��?�� ON/OFF
		bool bprintTire;	// ?��버레?��?�� 바�?�각?�� ?��?��
		bool bdark;			// ?��?�� ?���? ON/OFF
		bool bcheckFrontWhite;
		bool bcheckPriority;	// ?��?��?���? ?���??�� ?���? ON/OFF
		bool bcheckSignalLight; // ?��?��?�� ?���? ON/OFF
		bool bcheckFinishLine;	// ?��?��?��?��?�� ?���? ON/OFF
	};

	// DistanceSensor_cm
	// ?��?��?�� ?��?��?��?�� 물체????�� 거리�? 측정?��주는 ?��?��
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ?��?��?�� ?��?��?��?�� 측정?�� 거리 값을 반환?��?��.
	int DistanceSensor_cm(int channel);

	// sensor_dist
	// ?��?��?�� ?��?��?��?�� 측정?�� 거리 값을 ?��규화?��?��?�� ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ?��?��?�� ?��?���? 측정?�� 거리 값을 ?��규화?��?�� 반환?��?��.
	int sensor_dist(int channel, int input);

	// Encoder_Read
	// 차량?�� ?��?��?�� 거리�? 측정?��주는 ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ?��?��까�?? 차량?�� ?��?��?�� 거리�? 반환?��?��.
	signed int Encoder_Read(void);

	// StopLine
	// ?��?�� ?��?��?�� ?��?���? ?��?�� ?��?��?���? ?��별해주는 ?��?��
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: 7개의 ?��?��?�� ?��?�� �? ?��?��?�� 감�???�� ?��?��?�� 개수�? Lineflag보다 많으�? 1, ?��?��?�� 0
	int StopLine(int Lineflag);

	// SteeringServo_Write_uart
	// 차량?�� ?�� 바�?��?? ?��?��?�� 각도(steerVal)�? 조향?��주는 ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: 차량?�� ?�� 바�?�는 angle�? 조향?��?�� ?���??��?��.
	//			uart ?��?�� ?��무�?? 감�???��?�� ?��?��?�� 간섭?���? ?��?���? ???기한?��.
	// Return 		: none
	void SteeringServo_Write_uart(signed short angle);

	// DesireSpeed_Write_uart
	// 차량?�� ?��?���? ?��?��?�� ?��?��(speed)�? 맞춰주는 ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: 차량?�� ?��?��?�� speed�? ?��?�� ?���??��?��.
	//			uart ?��?�� ?��무�?? 감�???��?�� ?��?��?�� 간섭?���? ?��?���? ???기한?��.
	// Return 		: none
	void DesireSpeed_Write_uart(signed short speed);

	// DesiredDistance
	// ?��?��?�� ?��?��(speed)??? 각도(steerVal)�? �?�?�? ?��?�� 거리(encoderVal)�? 주행?��?�� ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: SettingSpeed?�� ?��?��??? SettingSteering?�� 각도�? ?���??��채로
	//                    SettingDistance 만큼?�� 거리�? 주행?��?��.
	// Return 		: none
	void DesiredDistance(int SettingSpeed, int SettingDistance, int SettingSteering);

	// sleepDistance
	// ?��?�� 거리(encoderVal)�? 주행?��?��?��?�� ?���??��고있?�� ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: SettingDistance 만큼?�� 거리�? 주행?�� ?��?���? 종료?��?��(주행??? ?���?).
	// Return 		: none
	void sleepDistance(int SettingDistance);

	// onlyDistance
	// ?��?��?�� ?��?��(speed)�? �?�?�? ?��?�� 거리(encoderVal)�? 주행?��?�� ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: SettingSpeed?�� ?��?���? ?���??��채로 SettingDistance 만큼?�� 거리�? 주행?��?��.
	// Return 		: none
	void onlyDistance(int SettingSpeed, int SettingDistance);

	// RoundAbout_isStart
	// ?��?�� 교차로에?�� 차량?�� �??��갔는�? ?��?��?��주는 ?��?��?��?��.
	// PreCondition 	: ?��?�� 교차로의 ?���??��?��?�� ???기중?��?��?�� ?��?��.
	// PostCondition	: 2.5초�?? ???기한 ?��?�� 주행?�� ?��?��?��?��.
	// Return 		: ?��?�� 차량?�� ?��????��?���? ?��?��졌으�? 1, ?��?��?�� 0
	int RoundAbout_isStart(const int Distance1);

	// Tunnel_isEnd
	// ?��?��?��?�� ?��?��?���? ?��?��?��주는 ?��?��?��?��.
	// PreCondition 	: ?��?��?�� 진입?�� ?�� ?��?�� ?��?��?�� ?��?��.
	// PostCondition	: ?��?��?��?�� ?��?��?���?�? 기본 주행?�� ?��?��?��?��.
	// Return 		: �?,?�� ?��?���? ?��?�� 거리 ?��?��?�� 값을 �?�?�? 1, ?��?��?�� 0
	int Tunnel_isEnd(const int Distance1, const int Distance2, const int Distance3, const int Distance4);

	// Tunnel_SteerVal
	// ?��?��?��?�� 차량?�� ?�� 바�?��?? 조향?��주는 ?��?��?��?��.
	// PreCondition 	: ?��?��?�� 진입?�� ?�� ?��?�� ?��?��?�� ?��?��.
	// PostCondition	: 차량?�� ?�� 바�?�는 angle�? 조향?��?�� ?���??��?��.
	// Return 		: �?,?�� ?��?��?��?�� 측정?�� 거리?�� 차이�? ?��?�� ?��?��?�� angle?�� 반환?��?��.
	int Tunnel_SteerVal(const int Distance1, const int Distance2);

	// frontLightOnOff
	// 차량?�� ?���? ?��?��?���? On, Off ?��주는 ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: on?�� 값에 ?��?�� 차량?�� ?���? ?��?��?���? On ?��?�� Off�? ?��?��.
	// Return 		: none
	void frontLightOnOff(unsigned short lightFlag, int on);

	// rearLightOnOff
	// 차량?�� ?���? ?��?��?���? On, Off ?��주는 ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: on?�� 값에 ?��?�� 차량?�� ?���? ?��?��?���? On ?��?�� Off�? ?��?��.
	// Return 		: none
	void rearLightOnOff(unsigned short lightFlag, int on);

	// auto_speedMapping
	// 차량?�� ?�� 바�?�의 각도?�� ?��?�� ?��?���? 조절?��주는 ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: 직진 주행?��?���? ?��?��?���? basicSpeed�? ?���??��면서 주행?��?��.
	//                    곡선 주행?��?���? ?��?��?���? steerVal?�� ?��?�� basicSpeed?�� �?중치�? �??��?��?��.
	// Return 		: steerVal?�� ?��?�� 주행?��?�� ?�� 차량?�� ?��?���? 반환?��?��.
	int auto_speedMapping(int steerVal, const int basicSpeed);

	// buzzer
	// 차량?�� horn?�� ?��리게 ?��?�� ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: ?��?��?�� ?��간만?�� buzzer�? ?��리고, numOfTimes만큼 반복?��?��.
	// Return 		: none
	void buzzer(int numOfTimes, int interval_us, int pulseWidth_us);

	// manualControl
	// key�? ?��?��받아?�� ?��?��?�� ?��?��?�� ?���? ?��?�� ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: ?��?�� key?�� ?��?�� ?��?�� ?��?��?�� ?��?��?��?��.
	// Return 		: none
	void manualControl(struct ControlData *cdata, char key);

	// timeCheck
	// ?��?��?�� 구간 ?��?��?�� 걸리?�� ?��간을 구하?�� ?��?��?��?��.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ?��?��까�?? 측정?�� ?��간을 반환?��?��.
	uint32_t timeCheck(struct timeval *tempTime);

	// laneChange
	// PreCondition		: none
	// PostCondition	: 0?���? 좌회?��, 1?���? ?��?��?�� ?��?��?�� ?��?�� ?��?��.
	// Return 			: none
	void laneChange(int direction, int speed, int length);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif //CONTROL_MISSION_H
