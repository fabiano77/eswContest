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
		CAMERA_UP,		//장애물 인식을 위해 올린상태
		CAMERA_DOWN 	//원래 상태 -->이 부분 조정 필요 MS
	};

	enum DirectionState
	{
		LEFT,
		RIGHT,
		STOP	//앞에 장애물이 있다면 스탑(overtaking이 on일 때만)
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
		bool bparking;	 		// 주차 중 거리 정보 출력을 위한 변수
		bool verticalFlag;		// 수직 주차 활성화를 나타내는 플래그
		bool horizontalFlag; 	// 수평 주차 활성화를 나타내는 플래그
	};

	struct Overtaking
	{
		bool sideSensorFlag;					// 차량의 사이드 탐지 활성화 플래그			   
		enum DirectionState headingDirection; 	//차량의 이동방향 결정
		enum CameraVerticalState updownCamera; 	//카메라를 위로 올릴지 말지 결정하는 부분
		char leftFlag;
		char rightFlag;
	};

	struct Finish
	{
		int distEndLine; //결승선까지의 거리
		bool checkFront; //앞의 수직 노란선 파악
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
		struct Parking parkingData;				// 주차에 필요한 플래그를 담는 구조체
		struct Overtaking overtakingData;		// 추월에 필요한 플래그 담는 구조체
		struct SignalLight signalLightData; 	// 신호등에 필요한 변수를 담는 구조체
		struct Finish finishData;
		enum MissionState ms[9]; 
		uint32_t loopTime; 						// mission 스레드 루프 시간

		int frame_priority;
		int finish_distance;
		bool broundabout;
		bool btunnel;
		bool overtakingFlag; 					// 추월차로 플래그 ->MS 이후 overtaking struct 추가할 것
		bool changeMissionState;
		bool checkWhiteLineFlag;
	};

	struct ControlData
	{
		int steerVal;			// 차량의 현재 앞바퀴 조향각
		int cameraY;			// 차량의 현재 카메라각도
		int desireSpeedVal;		// 차량의 현재 속도
		int beforeSpeedVal;		// 차량의 이전 속도
		int settingSpeedVal;	
		unsigned short lightFlag;// 차량의 현재 전조등상태
	};

	struct ImgProcessData
	{
		char missionString[50]; 	// 오버레이에 표시할 문자열
		int debugMode;				// 디버그 모드(0~ 9)
		int topMode;				// 탑뷰 모드 (0, 1, 2)
		uint32_t loopTime; 			// img 스레드 루프 시간
		bool dump_request; 			// 덤프요청
		bool bskip;					
		bool bvideoRecord;			// 동영상 녹화 시작
		bool bvideoSave;			// 동영상 파일 저장
		bool bcalibration;			// 캘리브레이션 ON/OFF
		bool bdebug;				// 디버그모드 ON/OFF
		bool btopview;				// 탑뷰 ON/OFF
		bool bmission;				// 미션진입 ON/OFF (차선인식 사용하지 않게됨)
		bool bauto;					// 자동 조향 ON/OFF
		bool bspeedControl; 		// 자동 조향의 속도개입 ON/OFF
		bool bwhiteLine;			// 자동 조향의 흰색 선 탐지 ON/OFF
		bool bprintString;			// 오버레이에 문자열 표시 ON/OFF
		bool bprintMission; 		// 오버레이에 미션정보 표시
		bool bprintSensor;			// 오버레이에 센서값 표시 ON/OFF
		bool bprintTire;			// 오버레이에 바퀴각도 표시
		bool bdark;					// 터널 탐지 ON/OFF
		bool bcheckFrontWhite;		
		bool bcheckPriority;		// 우선정지 표지판 탐지 ON/OFF
		bool bcheckSignalLight; 	// 신호등 탐지 ON/OFF
		bool bcheckFinishLine;		// 피니시라인 탐지 ON/OFF
	};

	// DistanceSensor_cm
	// 적외선 센서에서 물체와의 거리를 측정해주는 함수
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: 적외선 센서에서 측정한 거리 값을 반환한다.
	int DistanceSensor_cm(int channel);

	// sensor_dist
	// 적외선 센서에서 측정한 거리 값을 정규화시키는 함수이다.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: 적외선 센서로 측정한 거리 값을 정규화하여 반환한다.
	int sensor_dist(int channel, int input);

	// Encoder_Read
	// 차량이 이동한 거리를 측정해주는 함수이다.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: 현재까지 차량이 이동한 거리를 반환한다.
	signed int Encoder_Read(void);

	// StopLine
	// 하단 적외선 센서로 흰색 라인인지 판별해주는 함수
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: 7개의 적외선 센서 중 흰색을 감지한 센서의 개수가 Lineflag보다 많으면 1, 아닐시 0
	int StopLine(int Lineflag);

	// SteeringServo_Write_uart
	// 차량의 앞 바퀴를 원하는 각도(steerVal)로 조향해주는 함수이다.
	// PreCondition 	: none
	// PostCondition	: 차량의 앞 바퀴는 angle로 조향되어 유지된다.
	//			uart 통신 유무를 감지하여 통신에 간섭되지 않도록 대기한다.
	// Return 		: none
	void SteeringServo_Write_uart(signed short angle);

	// DesireSpeed_Write_uart
	// 차량의 속도를 원하는 속도(speed)로 맞춰주는 함수이다.
	// PreCondition 	: none
	// PostCondition	: 차량의 속도는 speed가 되어 유지된다.
	//			uart 통신 유무를 감지하여 통신에 간섭되지 않도록 대기한다.
	// Return 		: none
	void DesireSpeed_Write_uart(signed short speed);

	// DesiredDistance
	// 원하는 속도(speed)와 각도(steerVal)를 가지고 일정 거리(encoderVal)를 주행하는 함수이다.
	// PreCondition 	: none
	// PostCondition	: SettingSpeed의 속도와 SettingSteering의 각도를 유지한채로
	//                    SettingDistance 만큼의 거리를 주행한다.
	// Return 		: none
	void DesiredDistance(int SettingSpeed, int SettingDistance, int SettingSteering);

	// onlyDistance
	// 원하는 속도(speed)를 가지고 일정 거리(encoderVal)를 주행하는 함수이다.
	// PreCondition 	: none
	// PostCondition	: SettingSpeed의 속도를 유지한채로 SettingDistance 만큼의 거리를 주행한다.
	// Return 		: none
	void onlyDistance(int SettingSpeed, int SettingDistance);

	// RoundAbout_isStart
	// 회전 교차로에서 차량이 지나갔는지 확인해주는 함수이다.
	// PreCondition 	: 회전 교차로의 정지선에서 대기중이여야 한다.
	// PostCondition	: 2.5초를 대기한 후에 주행을 시작한다.
	// Return 		: 앞에 차량이 나타났다가 없어졌으면 1, 아닐시 0
	int RoundAbout_isStart(const int Distance1);

	// Tunnel_isEnd
	// 터널에서 나왔는지 확인해주는 함수이다.
	// PreCondition 	: 터널에 진입을 한 적이 있어야 한다.
	// PostCondition	: 터널에서 나왔으므로 기본 주행을 해야한다.
	// Return 		: 좌,우 센서가 일정 거리 이상의 값을 가지면 1, 아닐시 0
	int Tunnel_isEnd(const int Distance1, const int Distance2, const int Distance3, const int Distance4);

	// Tunnel_SteerVal
	// 터널에서 차량의 앞 바퀴를 조향해주는 함수이다.
	// PreCondition 	: 터널에 진입을 한 적이 있어야 한다.
	// PostCondition	: 차량의 앞 바퀴는 angle로 조향되어 유지된다.
	// Return 		: 좌,우 센서에서 측정된 거리의 차이를 통해 적절한 angle을 반환한다.
	int Tunnel_SteerVal(const int Distance1, const int Distance2);

	// frontLightOnOff
	// 차량의 전방 라이트를 On, Off 해주는 함수이다.
	// PreCondition 	: none
	// PostCondition	: on의 값에 따라 차량의 전방 라이트가 On 또는 Off가 된다.
	// Return 		: none
	void frontLightOnOff(unsigned short lightFlag, int on);

	// rearLightOnOff
	// 차량의 후방 라이트를 On, Off 해주는 함수이다.
	// PreCondition 	: none
	// PostCondition	: on의 값에 따라 차량의 후방 라이트가 On 또는 Off가 된다.
	// Return 		: none
	void rearLightOnOff(unsigned short lightFlag, int on);

	// auto_speedMapping
	// 차량의 앞 바퀴의 각도에 따라 속도를 조절해주는 함수이다.
	// PreCondition 	: none
	// PostCondition	: 직진 주행이라고 판단하면 basicSpeed를 유지하면서 주행한다.
	//                    곡선 주행이라고 판단하면 steerVal에 따라 basicSpeed에 가중치가 부여된다.
	// Return 		: steerVal에 따라 주행해야 할 차량의 속도를 반환한다.
	int auto_speedMapping(int steerVal, const int basicSpeed);

	// buzzer
	// 차량의 horn을 울리게 하는 함수이다.
	// PreCondition 	: none
	// PostCondition	: 원하는 시간만큼 buzzer가 울리고, numOfTimes만큼 반복한다.
	// Return 		: none
	void buzzer(int numOfTimes, int interval_us, int pulseWidth_us);

	// manualControl
	// key를 입력받아서 원하는 동작을 하게 하는 함수이다.
	// PreCondition 	: none
	// PostCondition	: 입력 key에 따라 해당 동작이 실행된다.
	// Return 		: none
	void manualControl(struct ControlData *cdata, char key);

	// timeCheck
	// 원하는 구간 사이에 걸리는 시간을 구하는 함수이다.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: 현재까지 측정한 시간을 반환한다.
	uint32_t timeCheck(struct timeval *tempTime);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif //CONTROL_MISSION_H
