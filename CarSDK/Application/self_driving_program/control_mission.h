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
		CAMERA_UP,	//?₯? λ¬? ?Έ?? ??΄ ?¬λ¦°μ?
		CAMERA_DOWN //?? ?? -->?΄ λΆ?λΆ? μ‘°μ  ?? MS
	};

	enum DirectionState
	{
		LEFT,
		RIGHT,
		STOP //?? ?₯? λ¬Όμ΄ ??€λ©? ?€?(overtaking?΄ on?Ό ?λ§?)
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
		bool bparking;		 // μ£Όμ°¨ μ€? κ±°λ¦¬ ? λ³? μΆλ ₯? ?? λ³??
		bool verticalFlag;	 // ?μ§? μ£Όμ°¨ ??±?λ₯? ?????΄? ??κ·?
		bool horizontalFlag; // ?? μ£Όμ°¨ ??±?λ₯? ?????΄? ??κ·?
	};

	struct Overtaking
	{
		bool sideSensorFlag;				   // μ°¨λ? ?¬?΄? ?μ§? ??±? ??κ·?
		enum DirectionState headingDirection;  //μ°¨λ? ?΄?λ°©ν₯ κ²°μ 
		enum CameraVerticalState updownCamera; //μΉ΄λ©?Όλ₯? ?λ‘? ?¬λ¦΄μ?? λ§μ?? κ²°μ ?? λΆ?λΆ?
		char leftFlag;
		char rightFlag;
	};

	struct Finish
	{
		int distEndLine; //κ²°μΉ? κΉμ??? κ±°λ¦¬
		bool checkFront; //?? ?μ§? ?Έ????  ??
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
		struct Parking parkingData;			// μ£Όμ°¨? ??? ??κ·Έλ?? ?΄? κ΅¬μ‘°μ²?
		struct Overtaking overtakingData;	// μΆμ? ??? ??κ·? ?΄? κ΅¬μ‘°μ²?
		struct SignalLight signalLightData; // ? ?Έ?±? ??? λ³??λ₯? ?΄? κ΅¬μ‘°μ²?
		struct Finish finishData;
		enum MissionState ms[9];
		uint32_t loopTime; // mission ?€? ? λ£¨ν ?κ°?

		int frame_priority;
		int finish_distance;
		bool broundabout;
		bool btunnel;
		bool overtakingFlag; // μΆμμ°¨λ‘ ??κ·? ->MS ?΄? overtaking struct μΆκ???  κ²?
		bool changeMissionState;
		bool checkWhiteLineFlag;
	};

	struct ControlData
	{
		int steerVal;		// μ°¨λ? ??¬ ?λ°ν?? μ‘°ν₯κ°?
		int cameraY;		// μ°¨λ? ??¬ μΉ΄λ©?Όκ°λ
		int desireSpeedVal; // μ°¨λ? ??¬ ??
		int beforeSpeedVal; // μ°¨λ? ?΄?  ??
		int settingSpeedVal;
		unsigned short lightFlag; // μ°¨λ? ??¬ ? μ‘°λ±??
	};

	struct ImgProcessData
	{
		char missionString[50]; // ?€λ²λ ?΄? ???  λ¬Έμ?΄
		int debugMode;			// ?λ²κ·Έ λͺ¨λ(0~ 9)
		int topMode;			// ?λ·? λͺ¨λ (0, 1, 2)
		uint32_t loopTime;		// img ?€? ? λ£¨ν ?κ°?
		bool dump_request;		// ?€??μ²?
		bool bskip;
		bool bvideoRecord;	// ??? ?Ή? ??
		bool bvideoSave;	// ??? ??Ό ????₯
		bool bcalibration;	// μΊλ¦¬λΈλ ?΄? ON/OFF
		bool bdebug;		// ?λ²κ·Έλͺ¨λ ON/OFF
		bool btopview;		// ?λ·? ON/OFF
		bool bmission;		// λ―Έμμ§μ ON/OFF (μ°¨μ ?Έ? ?¬?©?μ§? ?κ²λ¨)
		bool bauto;			// ?? μ‘°ν₯ ON/OFF
		bool bspeedControl; // ?? μ‘°ν₯? ??κ°μ ON/OFF
		bool bwhiteLine;	// ?? μ‘°ν₯? ?°? ?  ?μ§? ON/OFF
		bool bprintString;	// ?€λ²λ ?΄? λ¬Έμ?΄ ?? ON/OFF
		bool bprintMission; // ?€λ²λ ?΄? λ―Έμ? λ³? ??
		bool bprintSensor;	// ?€λ²λ ?΄? ?Ό?κ°? ?? ON/OFF
		bool bprintTire;	// ?€λ²λ ?΄? λ°ν?΄κ°? ??
		bool bdark;			// ?°? ?μ§? ON/OFF
		bool bcheckFrontWhite;
		bool bcheckPriority;	// ?°? ? μ§? ?μ§?? ?μ§? ON/OFF
		bool bcheckSignalLight; // ? ?Έ?± ?μ§? ON/OFF
		bool bcheckFinishLine;	// ?Ό???Ό?Έ ?μ§? ON/OFF
	};

	// DistanceSensor_cm
	// ? ?Έ?  ?Ό??? λ¬Όμ²΄???? κ±°λ¦¬λ₯? μΈ‘μ ?΄μ£Όλ ?¨?
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ? ?Έ?  ?Ό??? μΈ‘μ ? κ±°λ¦¬ κ°μ λ°ν??€.
	int DistanceSensor_cm(int channel);

	// sensor_dist
	// ? ?Έ?  ?Ό??? μΈ‘μ ? κ±°λ¦¬ κ°μ ? κ·ν??€? ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ? ?Έ?  ?Ό?λ‘? μΈ‘μ ? κ±°λ¦¬ κ°μ ? κ·ν??¬ λ°ν??€.
	int sensor_dist(int channel, int input);

	// Encoder_Read
	// μ°¨λ?΄ ?΄?? κ±°λ¦¬λ₯? μΈ‘μ ?΄μ£Όλ ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ??¬κΉμ?? μ°¨λ?΄ ?΄?? κ±°λ¦¬λ₯? λ°ν??€.
	signed int Encoder_Read(void);

	// StopLine
	// ??¨ ? ?Έ?  ?Ό?λ‘? ?°? ?Ό?Έ?Έμ§? ?λ³ν΄μ£Όλ ?¨?
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: 7κ°μ ? ?Έ?  ?Ό? μ€? ?°?? κ°μ??? ?Ό?? κ°μκ°? Lineflagλ³΄λ€ λ§μΌλ©? 1, ??? 0
	int StopLine(int Lineflag);

	// SteeringServo_Write_uart
	// μ°¨λ? ? λ°ν?΄λ?? ??? κ°λ(steerVal)λ‘? μ‘°ν₯?΄μ£Όλ ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: μ°¨λ? ? λ°ν?΄λ angleλ‘? μ‘°ν₯??΄ ? μ§???€.
	//			uart ?΅?  ? λ¬΄λ?? κ°μ????¬ ?΅? ? κ°μ­?μ§? ??λ‘? ???κΈ°ν?€.
	// Return 		: none
	void SteeringServo_Write_uart(signed short angle);

	// DesireSpeed_Write_uart
	// μ°¨λ? ??λ₯? ??? ??(speed)λ‘? λ§μΆ°μ£Όλ ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: μ°¨λ? ??? speedκ°? ??΄ ? μ§???€.
	//			uart ?΅?  ? λ¬΄λ?? κ°μ????¬ ?΅? ? κ°μ­?μ§? ??λ‘? ???κΈ°ν?€.
	// Return 		: none
	void DesireSpeed_Write_uart(signed short speed);

	// DesiredDistance
	// ??? ??(speed)??? κ°λ(steerVal)λ₯? κ°?μ§?κ³? ?Ό?  κ±°λ¦¬(encoderVal)λ₯? μ£Όν?? ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: SettingSpeed? ????? SettingSteering? κ°λλ₯? ? μ§??μ±λ‘
	//                    SettingDistance λ§νΌ? κ±°λ¦¬λ₯? μ£Όν??€.
	// Return 		: none
	void DesiredDistance(int SettingSpeed, int SettingDistance, int SettingSteering);

	// sleepDistance
	// ?Ό?  κ±°λ¦¬(encoderVal)λ₯? μ£Όν???? ? μ§??κ³ μ? ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: SettingDistance λ§νΌ? κ±°λ¦¬λ₯? μ£Όν? ?¨?λ₯? μ’λ£??€(μ£Όν??? ? μ§?).
	// Return 		: none
	void sleepDistance(int SettingDistance);

	// onlyDistance
	// ??? ??(speed)λ₯? κ°?μ§?κ³? ?Ό?  κ±°λ¦¬(encoderVal)λ₯? μ£Όν?? ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: SettingSpeed? ??λ₯? ? μ§??μ±λ‘ SettingDistance λ§νΌ? κ±°λ¦¬λ₯? μ£Όν??€.
	// Return 		: none
	void onlyDistance(int SettingSpeed, int SettingDistance);

	// RoundAbout_isStart
	// ??  κ΅μ°¨λ‘μ? μ°¨λ?΄ μ§??κ°λμ§? ??Έ?΄μ£Όλ ?¨??΄?€.
	// PreCondition 	: ??  κ΅μ°¨λ‘μ ? μ§?? ?? ???κΈ°μ€?΄?¬?Ό ??€.
	// PostCondition	: 2.5μ΄λ?? ???κΈ°ν ?? μ£Όν? ????€.
	// Return 		: ?? μ°¨λ?΄ ?????¬?€κ°? ??΄μ‘μΌλ©? 1, ??? 0
	int RoundAbout_isStart(const int Distance1);

	// Tunnel_isEnd
	// ?°??? ???μ§? ??Έ?΄μ£Όλ ?¨??΄?€.
	// PreCondition 	: ?°?? μ§μ? ? ? ?΄ ??΄?Ό ??€.
	// PostCondition	: ?°??? ???Όλ―?λ‘? κΈ°λ³Έ μ£Όν? ?΄?Ό??€.
	// Return 		: μ’?,?° ?Ό?κ°? ?Ό?  κ±°λ¦¬ ?΄?? κ°μ κ°?μ§?λ©? 1, ??? 0
	int Tunnel_isEnd(const int Distance1, const int Distance2, const int Distance3, const int Distance4);

	// Tunnel_SteerVal
	// ?°??? μ°¨λ? ? λ°ν?΄λ?? μ‘°ν₯?΄μ£Όλ ?¨??΄?€.
	// PreCondition 	: ?°?? μ§μ? ? ? ?΄ ??΄?Ό ??€.
	// PostCondition	: μ°¨λ? ? λ°ν?΄λ angleλ‘? μ‘°ν₯??΄ ? μ§???€.
	// Return 		: μ’?,?° ?Ό??? μΈ‘μ ? κ±°λ¦¬? μ°¨μ΄λ₯? ?΅?΄ ? ? ? angle? λ°ν??€.
	int Tunnel_SteerVal(const int Distance1, const int Distance2);

	// frontLightOnOff
	// μ°¨λ? ? λ°? ?Ό?΄?Έλ₯? On, Off ?΄μ£Όλ ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: on? κ°μ ?°?Ό μ°¨λ? ? λ°? ?Ό?΄?Έκ°? On ?? Offκ°? ??€.
	// Return 		: none
	void frontLightOnOff(unsigned short lightFlag, int on);

	// rearLightOnOff
	// μ°¨λ? ?λ°? ?Ό?΄?Έλ₯? On, Off ?΄μ£Όλ ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: on? κ°μ ?°?Ό μ°¨λ? ?λ°? ?Ό?΄?Έκ°? On ?? Offκ°? ??€.
	// Return 		: none
	void rearLightOnOff(unsigned short lightFlag, int on);

	// auto_speedMapping
	// μ°¨λ? ? λ°ν?΄μ κ°λ? ?°?Ό ??λ₯? μ‘°μ ?΄μ£Όλ ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: μ§μ§ μ£Όν?΄?Όκ³? ??¨?λ©? basicSpeedλ₯? ? μ§??λ©΄μ μ£Όν??€.
	//                    κ³‘μ  μ£Όν?΄?Όκ³? ??¨?λ©? steerVal? ?°?Ό basicSpeed? κ°?μ€μΉκ°? λΆ??¬??€.
	// Return 		: steerVal? ?°?Ό μ£Όν?΄?Ό ?  μ°¨λ? ??λ₯? λ°ν??€.
	int auto_speedMapping(int steerVal, const int basicSpeed);

	// buzzer
	// μ°¨λ? horn? ?Έλ¦¬κ² ?? ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: ??? ?κ°λ§?Ό buzzerκ°? ?Έλ¦¬κ³ , numOfTimesλ§νΌ λ°λ³΅??€.
	// Return 		: none
	void buzzer(int numOfTimes, int interval_us, int pulseWidth_us);

	// manualControl
	// keyλ₯? ?? ₯λ°μ? ??? ??? ?κ²? ?? ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: ?? ₯ key? ?°?Ό ?΄?Ή ???΄ ?€???€.
	// Return 		: none
	void manualControl(struct ControlData *cdata, char key);

	// timeCheck
	// ??? κ΅¬κ° ?¬?΄? κ±Έλ¦¬? ?κ°μ κ΅¬ν? ?¨??΄?€.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ??¬κΉμ?? μΈ‘μ ? ?κ°μ λ°ν??€.
	uint32_t timeCheck(struct timeval *tempTime);

	// laneChange
	// PreCondition		: none
	// PostCondition	: 0?΄λ©? μ’ν? , 1?΄λ©? ?°??  ??? ?? ?¨?.
	// Return 			: none
	void laneChange(int direction, int speed, int length);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif //CONTROL_MISSION_H
