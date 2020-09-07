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
		CAMERA_UP,	//장애물 인식을 위해 올린상태
		CAMERA_DOWN //원래 상태 -->이 부분 조정 필요 MS
	};

	enum DirectionState
	{
		LEFT,
		RIGHT,
		STOP //앞에 장애물이 있다면 스탑(overtaking이 on일 때만)
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
		bool bparking;		 // 주차 중 거리 정보 출력을 위한 변수
		bool verticalFlag;	 // 수직 주차 활성화를 나타내는 플래그
		bool horizontalFlag; // 수평 주차 활성화를 나타내는 플래그
	};

	struct Overtaking
	{
		bool sideSensorFlag;				   // 차량의 사이드 탐지 활성화 플래그
		enum DirectionState headingDirection;  //차량의 이동방향 결정
		enum CameraVerticalState updownCamera; //카메라를 위로 올릴지 말지 결정하는 부분
		char leftFlag;
		char rightFlag;
	};

	struct Finish
	{
		bool checkFront; //앞의 수직 노란선 파악
		int distEndLine; //결승선까지의 거리
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
		uint32_t loopTime; // mission 스레드 루프 시간
		bool broundabout;
		bool btunnel;
		bool overtakingFlag; // 추월차로 플래그 ->MS 이후 overtaking struct 추가할 것
		bool changeMissionState;
		int frame_priority;
		int finish_distance;

		struct Parking parkingData;			// 주차에 필요한 플래그를 담는 구조체
		struct Overtaking overtakingData;	// 추월에 필요한 플래그 담는 구조체
		struct SignalLight signalLightData; // 신호등에 필요한 변수를 담는 구조체
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
		uint32_t loopTime;		// img 스레드 루프 시간
		bool dump_request;		// 덤프요청
		bool bskip;
		bool bvideoRecord;		// 동영상 녹화 시작
		bool bvideoSave;		// 동영상 파일 저장
		bool bcalibration;		// 캘리브레이션
		bool bdebug;			// 디버그모드 ON/OFF
		bool btopview;			// 탑뷰 ON/OFF
		bool bmission;			// 미션진입 ON/OFF (차선인식 사용하지 않게됨)
		bool bauto;				// 자동 조향 ON/OFF
		bool bspeedControl;		// 자동 조향의 속도개입 ON/OFF
		bool bwhiteLine;		// 자동 조향의 흰색 선 탐지 ON/OFF
		bool bprintString;		// 오버레이에 문자열 표시 ON/OFF
		bool bprintMission;		// 오버레이에 미션정보 표시 ON/OFF
		bool bprintSensor;		// 오버레이에 센서값 표시 ON/OFF
		bool bprintTire;		// 오버레이에 바퀴각도 표시 ON/OFF
		bool bdark;				// 터널 탐지 ON/OFF
		bool bcheckPriority;	// 우선정지 표지판 탐지 ON/OFF
		bool bcheckSignalLight; // 신호등 탐지 ON/OFF
		bool bcheckFinishLine;	// 피니시라인 탐지 ON/OFF
		char missionString[20]; // 오버레이에 표시할 문자열
		int topMode;			// 탑뷰 모드 (0, 1, 2)
		int debugMode;			// 디버그 모드(0~ 7)
	};

	struct thr_data
	{
		struct display* disp;
		struct v4l2* v4l2;
		struct vpe* vpe;
		struct buffer** input_bufs;
		struct ControlData controlData;
		struct MissionData missionData;
		struct ImgProcessData imgData;
		unsigned char img_data_buf[VPE_OUTPUT_IMG_SIZE];

		int msgq_id;

		bool bfull_screen;
		bool bstream_start;
		pthread_t threads[4]; //스레드개수
	};

int DistanceSensor_cm(int channel);
//Return : 적외선 센서 값(cm)을 반환한다. (거리가 가까울수록 숫자높음)

int sensor_dist(int channel, int input);
// Return : 적외선 센서 값을 측정값 기준으로 맵핑하여 cm 단위로 반환한다.

signed int Encoder_Read(void);
// return : 현재 인코더 값을 반환한다

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

static void manualControl(struct ControlData* cdata, char key);

static uint32_t timeCheck(struct timeval* tempTime);

#ifdef __cplusplus
}



#endif /* __cplusplus */

#endif //CONTROL_MISSION_H