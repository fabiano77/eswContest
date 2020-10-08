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
		CAMERA_UP,	
		CAMERA_DOWN 
	};

	enum DirectionState
	{
		LEFT,
		RIGHT,
		STOP
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
		bool bparking;	 
		bool verticalFlag;	
		bool horizontalFlag; 
	};

	struct Overtaking
	{
		bool sideSensorFlag;				   
		enum DirectionState headingDirection; 
		enum CameraVerticalState updownCamera; 
		char leftFlag;
		char rightFlag;
	};

	struct Finish
	{
		bool checkFront; 
		int distEndLine; 
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
		uint32_t loopTime; 
		bool broundabout;
		bool btunnel;
		bool overtakingFlag; 
		bool changeMissionState;
		bool checkWhiteLineFlag;
		int frame_priority;
		int finish_distance;

		struct Parking parkingData;			
		struct Overtaking overtakingData;	
		struct SignalLight signalLightData; 
		struct Finish finishData;
		enum MissionState ms[9]; 
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
		uint32_t loopTime; 
		bool dump_request; 
		bool bskip;
		bool bvideoRecord;	
		bool bvideoSave;	
		bool bcalibration;	
		bool bdebug;		
		bool btopview;		
		bool bmission;		
		bool bauto;			
		bool bspeedControl; 
		bool bwhiteLine;	
		bool bprintString;	
		bool bprintMission; 
		bool bprintSensor;	
		bool bprintTire;	
		bool bdark;			
		bool bcheckFrontWhite;
		bool bcheckPriority;	
		bool bcheckSignalLight; 
		bool bcheckFinishLine;	
		char missionString[30]; 
		int topMode;			
		int debugMode;			
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
	// PostCondition	: 
	// Return 		: none
	void manualControl(struct ControlData *cdata, char key);

	// timeCheck
	// 원하는 구간 사이에 걸리는 시간을 구하는 함수이다.
	// PreCondition 	: 
	// PostCondition	: 
	// Return 		: 
	uint32_t timeCheck(struct timeval *tempTime);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif //CONTROL_MISSION_H
