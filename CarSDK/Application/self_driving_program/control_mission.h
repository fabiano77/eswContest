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
		CAMERA_UP,	//?¥?• ë¬? ?¸?‹?„ ?œ„?•´ ?˜¬ë¦°ìƒ?ƒœ
		CAMERA_DOWN //?›?˜ ?ƒ?ƒœ -->?´ ë¶?ë¶? ì¡°ì • ?•„?š” MS
	};

	enum DirectionState
	{
		LEFT,
		RIGHT,
		STOP //?•?— ?¥?• ë¬¼ì´ ?ˆ?‹¤ë©? ?Š¤?ƒ‘(overtaking?´ on?¼ ?•Œë§?)
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
		bool bparking;		 // ì£¼ì°¨ ì¤? ê±°ë¦¬ ? •ë³? ì¶œë ¥?„ ?œ„?•œ ë³??ˆ˜
		bool verticalFlag;	 // ?ˆ˜ì§? ì£¼ì°¨ ?™œ?„±?™”ë¥? ?‚˜????‚´?Š” ?”Œ?˜ê·?
		bool horizontalFlag; // ?ˆ˜?‰ ì£¼ì°¨ ?™œ?„±?™”ë¥? ?‚˜????‚´?Š” ?”Œ?˜ê·?
	};

	struct Overtaking
	{
		bool sideSensorFlag;				   // ì°¨ëŸ‰?˜ ?‚¬?´?“œ ?ƒì§? ?™œ?„±?™” ?”Œ?˜ê·?
		enum DirectionState headingDirection;  //ì°¨ëŸ‰?˜ ?´?™ë°©í–¥ ê²°ì •
		enum CameraVerticalState updownCamera; //ì¹´ë©”?¼ë¥? ?œ„ë¡? ?˜¬ë¦´ì?? ë§ì?? ê²°ì •?•˜?Š” ë¶?ë¶?
		char leftFlag;
		char rightFlag;
	};

	struct Finish
	{
		int distEndLine; //ê²°ìŠ¹?„ ê¹Œì???˜ ê±°ë¦¬
		bool checkFront; //?•?˜ ?ˆ˜ì§? ?…¸????„  ?ŒŒ?•…
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
		struct Parking parkingData;			// ì£¼ì°¨?— ?•„?š”?•œ ?”Œ?˜ê·¸ë?? ?‹´?Š” êµ¬ì¡°ì²?
		struct Overtaking overtakingData;	// ì¶”ì›”?— ?•„?š”?•œ ?”Œ?˜ê·? ?‹´?Š” êµ¬ì¡°ì²?
		struct SignalLight signalLightData; // ?‹ ?˜¸?“±?— ?•„?š”?•œ ë³??ˆ˜ë¥? ?‹´?Š” êµ¬ì¡°ì²?
		struct Finish finishData;
		enum MissionState ms[9];
		uint32_t loopTime; // mission ?Š¤? ˆ?“œ ë£¨í”„ ?‹œê°?

		int frame_priority;
		int finish_distance;
		bool broundabout;
		bool btunnel;
		bool overtakingFlag; // ì¶”ì›”ì°¨ë¡œ ?”Œ?˜ê·? ->MS ?´?›„ overtaking struct ì¶”ê???•  ê²?
		bool changeMissionState;
		bool checkWhiteLineFlag;
	};

	struct ControlData
	{
		int steerVal;		// ì°¨ëŸ‰?˜ ?˜„?¬ ?•ë°”í?? ì¡°í–¥ê°?
		int cameraY;		// ì°¨ëŸ‰?˜ ?˜„?¬ ì¹´ë©”?¼ê°ë„
		int desireSpeedVal; // ì°¨ëŸ‰?˜ ?˜„?¬ ?†?„
		int beforeSpeedVal; // ì°¨ëŸ‰?˜ ?´? „ ?†?„
		int settingSpeedVal;
		unsigned short lightFlag; // ì°¨ëŸ‰?˜ ?˜„?¬ ? „ì¡°ë“±?ƒ?ƒœ
	};

	struct ImgProcessData
	{
		char missionString[50]; // ?˜¤ë²„ë ˆ?´?— ?‘œ?‹œ?•  ë¬¸ì?—´
		int debugMode;			// ?””ë²„ê·¸ ëª¨ë“œ(0~ 9)
		int topMode;			// ?ƒ‘ë·? ëª¨ë“œ (0, 1, 2)
		uint32_t loopTime;		// img ?Š¤? ˆ?“œ ë£¨í”„ ?‹œê°?
		bool dump_request;		// ?¤?”„?š”ì²?
		bool bskip;
		bool bvideoRecord;	// ?™?˜?ƒ ?…¹?™” ?‹œ?‘
		bool bvideoSave;	// ?™?˜?ƒ ?ŒŒ?¼ ????¥
		bool bcalibration;	// ìº˜ë¦¬ë¸Œë ˆ?´?…˜ ON/OFF
		bool bdebug;		// ?””ë²„ê·¸ëª¨ë“œ ON/OFF
		bool btopview;		// ?ƒ‘ë·? ON/OFF
		bool bmission;		// ë¯¸ì…˜ì§„ì… ON/OFF (ì°¨ì„ ?¸?‹ ?‚¬?š©?•˜ì§? ?•Šê²Œë¨)
		bool bauto;			// ??™ ì¡°í–¥ ON/OFF
		bool bspeedControl; // ??™ ì¡°í–¥?˜ ?†?„ê°œì… ON/OFF
		bool bwhiteLine;	// ??™ ì¡°í–¥?˜ ?°?ƒ‰ ?„  ?ƒì§? ON/OFF
		bool bprintString;	// ?˜¤ë²„ë ˆ?´?— ë¬¸ì?—´ ?‘œ?‹œ ON/OFF
		bool bprintMission; // ?˜¤ë²„ë ˆ?´?— ë¯¸ì…˜? •ë³? ?‘œ?‹œ
		bool bprintSensor;	// ?˜¤ë²„ë ˆ?´?— ?„¼?„œê°? ?‘œ?‹œ ON/OFF
		bool bprintTire;	// ?˜¤ë²„ë ˆ?´?— ë°”í?´ê°?„ ?‘œ?‹œ
		bool bdark;			// ?„°?„ ?ƒì§? ON/OFF
		bool bcheckFrontWhite;
		bool bcheckPriority;	// ?š°?„ ? •ì§? ?‘œì§??Œ ?ƒì§? ON/OFF
		bool bcheckSignalLight; // ?‹ ?˜¸?“± ?ƒì§? ON/OFF
		bool bcheckFinishLine;	// ?”¼?‹ˆ?‹œ?¼?¸ ?ƒì§? ON/OFF
	};

	// DistanceSensor_cm
	// ? ?™¸?„  ?„¼?„œ?—?„œ ë¬¼ì²´????˜ ê±°ë¦¬ë¥? ì¸¡ì •?•´ì£¼ëŠ” ?•¨?ˆ˜
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ? ?™¸?„  ?„¼?„œ?—?„œ ì¸¡ì •?•œ ê±°ë¦¬ ê°’ì„ ë°˜í™˜?•œ?‹¤.
	int DistanceSensor_cm(int channel);

	// sensor_dist
	// ? ?™¸?„  ?„¼?„œ?—?„œ ì¸¡ì •?•œ ê±°ë¦¬ ê°’ì„ ? •ê·œí™”?‹œ?‚¤?Š” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ? ?™¸?„  ?„¼?„œë¡? ì¸¡ì •?•œ ê±°ë¦¬ ê°’ì„ ? •ê·œí™”?•˜?—¬ ë°˜í™˜?•œ?‹¤.
	int sensor_dist(int channel, int input);

	// Encoder_Read
	// ì°¨ëŸ‰?´ ?´?™?•œ ê±°ë¦¬ë¥? ì¸¡ì •?•´ì£¼ëŠ” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ?˜„?¬ê¹Œì?? ì°¨ëŸ‰?´ ?´?™?•œ ê±°ë¦¬ë¥? ë°˜í™˜?•œ?‹¤.
	signed int Encoder_Read(void);

	// StopLine
	// ?•˜?‹¨ ? ?™¸?„  ?„¼?„œë¡? ?°?ƒ‰ ?¼?¸?¸ì§? ?Œë³„í•´ì£¼ëŠ” ?•¨?ˆ˜
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: 7ê°œì˜ ? ?™¸?„  ?„¼?„œ ì¤? ?°?ƒ‰?„ ê°ì???•œ ?„¼?„œ?˜ ê°œìˆ˜ê°? Lineflagë³´ë‹¤ ë§ìœ¼ë©? 1, ?•„?‹?‹œ 0
	int StopLine(int Lineflag);

	// SteeringServo_Write_uart
	// ì°¨ëŸ‰?˜ ?• ë°”í?´ë?? ?›?•˜?Š” ê°ë„(steerVal)ë¡? ì¡°í–¥?•´ì£¼ëŠ” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: ì°¨ëŸ‰?˜ ?• ë°”í?´ëŠ” angleë¡? ì¡°í–¥?˜?–´ ?œ ì§??œ?‹¤.
	//			uart ?†µ?‹  ?œ ë¬´ë?? ê°ì???•˜?—¬ ?†µ?‹ ?— ê°„ì„­?˜ì§? ?•Š?„ë¡? ???ê¸°í•œ?‹¤.
	// Return 		: none
	void SteeringServo_Write_uart(signed short angle);

	// DesireSpeed_Write_uart
	// ì°¨ëŸ‰?˜ ?†?„ë¥? ?›?•˜?Š” ?†?„(speed)ë¡? ë§ì¶°ì£¼ëŠ” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: ì°¨ëŸ‰?˜ ?†?„?Š” speedê°? ?˜?–´ ?œ ì§??œ?‹¤.
	//			uart ?†µ?‹  ?œ ë¬´ë?? ê°ì???•˜?—¬ ?†µ?‹ ?— ê°„ì„­?˜ì§? ?•Š?„ë¡? ???ê¸°í•œ?‹¤.
	// Return 		: none
	void DesireSpeed_Write_uart(signed short speed);

	// DesiredDistance
	// ?›?•˜?Š” ?†?„(speed)??? ê°ë„(steerVal)ë¥? ê°?ì§?ê³? ?¼? • ê±°ë¦¬(encoderVal)ë¥? ì£¼í–‰?•˜?Š” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: SettingSpeed?˜ ?†?„??? SettingSteering?˜ ê°ë„ë¥? ?œ ì§??•œì±„ë¡œ
	//                    SettingDistance ë§Œí¼?˜ ê±°ë¦¬ë¥? ì£¼í–‰?•œ?‹¤.
	// Return 		: none
	void DesiredDistance(int SettingSpeed, int SettingDistance, int SettingSteering);

	// sleepDistance
	// ?¼? • ê±°ë¦¬(encoderVal)ë¥? ì£¼í–‰?•˜?Š”?™?•ˆ ?œ ì§??•˜ê³ ìˆ?Š” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: SettingDistance ë§Œí¼?˜ ê±°ë¦¬ë¥? ì£¼í–‰?›„ ?•¨?ˆ˜ë¥? ì¢…ë£Œ?•œ?‹¤(ì£¼í–‰??? ?œ ì§?).
	// Return 		: none
	void sleepDistance(int SettingDistance);

	// onlyDistance
	// ?›?•˜?Š” ?†?„(speed)ë¥? ê°?ì§?ê³? ?¼? • ê±°ë¦¬(encoderVal)ë¥? ì£¼í–‰?•˜?Š” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: SettingSpeed?˜ ?†?„ë¥? ?œ ì§??•œì±„ë¡œ SettingDistance ë§Œí¼?˜ ê±°ë¦¬ë¥? ì£¼í–‰?•œ?‹¤.
	// Return 		: none
	void onlyDistance(int SettingSpeed, int SettingDistance);

	// RoundAbout_isStart
	// ?šŒ? „ êµì°¨ë¡œì—?„œ ì°¨ëŸ‰?´ ì§??‚˜ê°”ëŠ”ì§? ?™•?¸?•´ì£¼ëŠ” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: ?šŒ? „ êµì°¨ë¡œì˜ ? •ì§??„ ?—?„œ ???ê¸°ì¤‘?´?—¬?•¼ ?•œ?‹¤.
	// PostCondition	: 2.5ì´ˆë?? ???ê¸°í•œ ?›„?— ì£¼í–‰?„ ?‹œ?‘?•œ?‹¤.
	// Return 		: ?•?— ì°¨ëŸ‰?´ ?‚˜????‚¬?‹¤ê°? ?—†?–´ì¡Œìœ¼ë©? 1, ?•„?‹?‹œ 0
	int RoundAbout_isStart(const int Distance1);

	// Tunnel_isEnd
	// ?„°?„?—?„œ ?‚˜?™”?Š”ì§? ?™•?¸?•´ì£¼ëŠ” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: ?„°?„?— ì§„ì…?„ ?•œ ? ?´ ?ˆ?–´?•¼ ?•œ?‹¤.
	// PostCondition	: ?„°?„?—?„œ ?‚˜?™”?œ¼ë¯?ë¡? ê¸°ë³¸ ì£¼í–‰?„ ?•´?•¼?•œ?‹¤.
	// Return 		: ì¢?,?š° ?„¼?„œê°? ?¼? • ê±°ë¦¬ ?´?ƒ?˜ ê°’ì„ ê°?ì§?ë©? 1, ?•„?‹?‹œ 0
	int Tunnel_isEnd(const int Distance1, const int Distance2, const int Distance3, const int Distance4);

	// Tunnel_SteerVal
	// ?„°?„?—?„œ ì°¨ëŸ‰?˜ ?• ë°”í?´ë?? ì¡°í–¥?•´ì£¼ëŠ” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: ?„°?„?— ì§„ì…?„ ?•œ ? ?´ ?ˆ?–´?•¼ ?•œ?‹¤.
	// PostCondition	: ì°¨ëŸ‰?˜ ?• ë°”í?´ëŠ” angleë¡? ì¡°í–¥?˜?–´ ?œ ì§??œ?‹¤.
	// Return 		: ì¢?,?š° ?„¼?„œ?—?„œ ì¸¡ì •?œ ê±°ë¦¬?˜ ì°¨ì´ë¥? ?†µ?•´ ? ? ˆ?•œ angle?„ ë°˜í™˜?•œ?‹¤.
	int Tunnel_SteerVal(const int Distance1, const int Distance2);

	// frontLightOnOff
	// ì°¨ëŸ‰?˜ ? „ë°? ?¼?´?Š¸ë¥? On, Off ?•´ì£¼ëŠ” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: on?˜ ê°’ì— ?”°?¼ ì°¨ëŸ‰?˜ ? „ë°? ?¼?´?Š¸ê°? On ?˜?Š” Offê°? ?œ?‹¤.
	// Return 		: none
	void frontLightOnOff(unsigned short lightFlag, int on);

	// rearLightOnOff
	// ì°¨ëŸ‰?˜ ?›„ë°? ?¼?´?Š¸ë¥? On, Off ?•´ì£¼ëŠ” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: on?˜ ê°’ì— ?”°?¼ ì°¨ëŸ‰?˜ ?›„ë°? ?¼?´?Š¸ê°? On ?˜?Š” Offê°? ?œ?‹¤.
	// Return 		: none
	void rearLightOnOff(unsigned short lightFlag, int on);

	// auto_speedMapping
	// ì°¨ëŸ‰?˜ ?• ë°”í?´ì˜ ê°ë„?— ?”°?¼ ?†?„ë¥? ì¡°ì ˆ?•´ì£¼ëŠ” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: ì§ì§„ ì£¼í–‰?´?¼ê³? ?Œ?‹¨?•˜ë©? basicSpeedë¥? ?œ ì§??•˜ë©´ì„œ ì£¼í–‰?•œ?‹¤.
	//                    ê³¡ì„  ì£¼í–‰?´?¼ê³? ?Œ?‹¨?•˜ë©? steerVal?— ?”°?¼ basicSpeed?— ê°?ì¤‘ì¹˜ê°? ë¶??—¬?œ?‹¤.
	// Return 		: steerVal?— ?”°?¼ ì£¼í–‰?•´?•¼ ?•  ì°¨ëŸ‰?˜ ?†?„ë¥? ë°˜í™˜?•œ?‹¤.
	int auto_speedMapping(int steerVal, const int basicSpeed);

	// buzzer
	// ì°¨ëŸ‰?˜ horn?„ ?š¸ë¦¬ê²Œ ?•˜?Š” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: ?›?•˜?Š” ?‹œê°„ë§Œ?¼ buzzerê°? ?š¸ë¦¬ê³ , numOfTimesë§Œí¼ ë°˜ë³µ?•œ?‹¤.
	// Return 		: none
	void buzzer(int numOfTimes, int interval_us, int pulseWidth_us);

	// manualControl
	// keyë¥? ?…? ¥ë°›ì•„?„œ ?›?•˜?Š” ?™?‘?„ ?•˜ê²? ?•˜?Š” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: ?…? ¥ key?— ?”°?¼ ?•´?‹¹ ?™?‘?´ ?‹¤?–‰?œ?‹¤.
	// Return 		: none
	void manualControl(struct ControlData *cdata, char key);

	// timeCheck
	// ?›?•˜?Š” êµ¬ê°„ ?‚¬?´?— ê±¸ë¦¬?Š” ?‹œê°„ì„ êµ¬í•˜?Š” ?•¨?ˆ˜?´?‹¤.
	// PreCondition 	: none
	// PostCondition	: none
	// Return 		: ?˜„?¬ê¹Œì?? ì¸¡ì •?•œ ?‹œê°„ì„ ë°˜í™˜?•œ?‹¤.
	uint32_t timeCheck(struct timeval *tempTime);

	// laneChange
	// PreCondition		: none
	// PostCondition	: 0?´ë©? ì¢ŒíšŒ? „, 1?´ë©? ?š°?šŒ? „ ?™?‘?„ ?•˜?Š” ?•¨?ˆ˜.
	// Return 			: none
	void laneChange(int direction, int speed, int length);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif //CONTROL_MISSION_H
