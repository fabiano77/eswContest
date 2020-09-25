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
		CAMERA_UP,	//ï¿½ï¿½Ö¹ï¿? ï¿½Î½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½Ã¸ï¿½ï¿½ï¿½ï¿½ï¿½
		CAMERA_DOWN //ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ -->ï¿½ï¿½ ï¿½Îºï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½Ê¿ï¿½ MS
	};

	enum DirectionState
	{
		LEFT,
		RIGHT,
		STOP //ï¿½Õ¿ï¿½ ï¿½ï¿½Ö¹ï¿½ï¿½ï¿? ï¿½Ö´Ù¸ï¿½ ï¿½ï¿½Å¾(overtakingï¿½ï¿½ onï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½)
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
		bool bparking;		 // ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ ï¿½Å¸ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿? ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½
		bool verticalFlag;	 // ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ È°ï¿½ï¿½È­ï¿½ï¿½ ï¿½ï¿½Å¸ï¿½ï¿½ï¿½ï¿½ ï¿½Ã·ï¿½ï¿½ï¿½
		bool horizontalFlag; // ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ È°ï¿½ï¿½È­ï¿½ï¿½ ï¿½ï¿½Å¸ï¿½ï¿½ï¿½ï¿½ ï¿½Ã·ï¿½ï¿½ï¿½
	};

	struct Overtaking
	{
		bool sideSensorFlag;				   // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½Ìµï¿½ Å½ï¿½ï¿½ È°ï¿½ï¿½È­ ï¿½Ã·ï¿½ï¿½ï¿½
		enum DirectionState headingDirection;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½Ìµï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½
		enum CameraVerticalState updownCamera; //Ä«ï¿½Ş¶ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½Ã¸ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½Ï´ï¿½ ï¿½Îºï¿½
		char leftFlag;
		char rightFlag;
	};

	struct Finish
	{
		bool checkFront; //ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿? ï¿½Ä¾ï¿½
		int distEndLine; //ï¿½ï¿½Â¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿? ï¿½Å¸ï¿½
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
		uint32_t loopTime; // mission ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½Ã°ï¿½
		bool broundabout;
		bool btunnel;
		bool overtakingFlag; // ï¿½ß¿ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½Ã·ï¿½ï¿½ï¿½ ->MS ï¿½ï¿½ï¿½ï¿½ overtaking struct ï¿½ß°ï¿½ï¿½ï¿½ ï¿½ï¿½
		bool changeMissionState;
		bool checkWhiteLineFlag;
		int frame_priority;
		int finish_distance;

		struct Parking parkingData;			// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½Ê¿ï¿½ï¿½ï¿½ ï¿½Ã·ï¿½ï¿½×¸ï¿½ ï¿½ï¿½ï¿? ï¿½ï¿½ï¿½ï¿½Ã¼
		struct Overtaking overtakingData;	// ï¿½ß¿ï¿½ï¿½ï¿½ ï¿½Ê¿ï¿½ï¿½ï¿½ ï¿½Ã·ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿? ï¿½ï¿½ï¿½ï¿½Ã¼
		struct SignalLight signalLightData; // ï¿½ï¿½È£ï¿½î¿¡ ï¿½Ê¿ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿? ï¿½ï¿½ï¿½ï¿½Ã¼
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
		uint32_t loopTime; // img ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½Ã°ï¿½
		bool dump_request; // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ã»
		bool bskip;
		bool bvideoRecord;	// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½È­ ï¿½ï¿½ï¿½ï¿½
		bool bvideoSave;	// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½
		bool bcalibration;	// Ä¶ï¿½ï¿½ï¿½ê·¹ï¿½Ì¼ï¿½
		bool bdebug;		// ï¿½ï¿½ï¿½ï¿½×¸ï¿½ï¿½ ON/OFF
		bool btopview;		// Å¾ï¿½ï¿½ ON/OFF
		bool bmission;		// ï¿½Ì¼ï¿½ï¿½ï¿½ï¿½ï¿½ ON/OFF (ï¿½ï¿½ï¿½ï¿½ï¿½Î½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿? ï¿½Ê°Ôµï¿½)
		bool bauto;			// ï¿½Úµï¿½ ï¿½ï¿½ï¿½ï¿½ ON/OFF
		bool bspeedControl; // ï¿½Úµï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½Óµï¿½ï¿½ï¿½ï¿½ï¿½ ON/OFF
		bool bwhiteLine;	// ï¿½Úµï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿? ï¿½ï¿½ Å½ï¿½ï¿½ ON/OFF
		bool bprintString;	// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ì¿ï¿½ ï¿½ï¿½ï¿½Ú¿ï¿½ Ç¥ï¿½ï¿½ ON/OFF
		bool bprintMission; // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ì¿ï¿½ ï¿½Ì¼ï¿½ï¿½ï¿½ï¿½ï¿½ Ç¥ï¿½ï¿½ ON/OFF
		bool bprintSensor;	// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ì¿ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ Ç¥ï¿½ï¿½ ON/OFF
		bool bprintTire;	// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ì¿ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ Ç¥ï¿½ï¿½ ON/OFF
		bool bdark;			// ï¿½Í³ï¿½ Å½ï¿½ï¿½ ON/OFF
		bool bcheckFrontWhite;
		bool bcheckPriority;	// ï¿½ì¼±ï¿½ï¿½ï¿½ï¿½ Ç¥ï¿½ï¿½ï¿½ï¿½ Å½ï¿½ï¿½ ON/OFF
		bool bcheckSignalLight; // ï¿½ï¿½È£ï¿½ï¿½ Å½ï¿½ï¿½ ON/OFF
		bool bcheckFinishLine;	// ï¿½Ç´Ï½Ã¶ï¿½ï¿½ï¿½ Å½ï¿½ï¿½ ON/OFF
		char missionString[30]; // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ì¿ï¿½ Ç¥ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½Ú¿ï¿½
		int topMode;			// Å¾ï¿½ï¿½ ï¿½ï¿½ï¿? (0, 1, 2)
		int debugMode;			// ï¿½ï¿½ï¿½ï¿½ï¿? ï¿½ï¿½ï¿?(0~ 7)
	};

	int DistanceSensor_cm(int channel);
	//Return : ï¿½ï¿½ï¿½Ü¼ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½(cm)ï¿½ï¿½ ï¿½ï¿½È¯ï¿½Ñ´ï¿½. (ï¿½Å¸ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½Ú³ï¿½ï¿½ï¿½)

	int sensor_dist(int channel, int input);
	// Return : ï¿½ï¿½ï¿½Ü¼ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½Ï¿ï¿½ cm ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½È¯ï¿½Ñ´ï¿½.

	signed int Encoder_Read(void);
	// return : ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½Ú´ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½È¯ï¿½Ñ´ï¿½

	int StopLine(int Lineflag);
	// Return : Lineflagï¿½ï¿½ï¿½ï¿½ 1ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ Å©ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½Ç´ï¿½ï¿½Ï°ï¿½ trueï¿½ï¿½ ï¿½ï¿½È¯ï¿½Ñ´ï¿½.

	int STOP_WhiteLine(int Lineflag);

	void SteeringServo_Write_uart(signed short angle);
	//uart?†µ?‹ ?„?•˜?Š” ?‹¤ë¥? ?„¼?„œread()?•¨?ˆ˜??? ê°„ì„­?„ ?¼?œ¼?‚¤ì§? ?•Š?„ë¡? ë³´ì™„.

	void DesireSpeed_Write_uart(signed short speed);
	//uart?†µ?‹ ?„?•˜?Š” ?‹¤ë¥? ?„¼?„œread()?•¨?ˆ˜??? ê°„ì„­?„ ?¼?œ¼?‚¤ì§? ?•Š?„ë¡? ë³´ì™„.

	void DesiredDistance(int SettingSpeed, int SettingDistance, int SettingSteering);
	// Postcondition : SettingSpeedï¿½ï¿½ ï¿½Óµï¿½(ï¿½ï¿½ï¿½ï¿½ï¿? ï¿½ï¿½ï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½)ï¿½ï¿½ SettingDistanceï¿½ï¿½ step ï¿½ï¿½Å­ ï¿½ï¿½ï¿½ï¿½ï¿½Î´ï¿½.

	void onlyDistance(int SettingSpeed, int SettingDistance);
	//ds

	int RoundAbout_isStart(const int Distance1);
	//

	int Tunnel_isEnd(const int Distance1, const int Distance2, const int Distance3, const int Distance4);
	// Return : ï¿½ï¿½, ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿? ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ 1ï¿½ï¿½ ï¿½ï¿½È¯ï¿½Ñ´ï¿½.

	int Tunnel_SteerVal(const int Distance1, const int Distance2);
	// Return : ï¿½ï¿½ ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½Å¸ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½Ì¿ï¿½ï¿½Ï¿ï¿½ ï¿½Ë¸ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½â°¢ï¿½ï¿½ ï¿½ï¿½È¯ï¿½Ñ´ï¿½.

	void frontLightOnOff(unsigned short lightFlag, int on);
	// Postcondition : ï¿½ï¿½ï¿½Ú°ï¿½ 1ï¿½Ï°ï¿½ï¿? ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ on, 0ï¿½Ï°ï¿½ï¿? off

	void rearLightOnOff(unsigned short lightFlag, int on);
	// Postcondition : ï¿½ï¿½ï¿½Ú°ï¿½ 1ï¿½Ï°ï¿½ï¿? ï¿½Ä¹Ìµï¿½ on, 0ï¿½Ï°ï¿½ï¿? off

	int auto_speedMapping(int steerVal, const int basicSpeed);
	// Return :autoSteer ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½â°ªï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½Óµï¿½ï¿½ï¿½ ï¿½ï¿½È¯ï¿½ï¿½ï¿½Ø´ï¿½.

	void buzzer(int numOfTimes, int interval_us, int pulseWidth_us);
	// numOfTimes		: ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¸®ï¿½ï¿½ È½ï¿½ï¿½
	// interval_us		: ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½Ã°ï¿½ï¿½ï¿½ï¿½ï¿½
	// pulseWidth_us	: ï¿½ï¿½ï¿½ï¿½ï¿½Ï³ï¿½ï¿½ï¿½ ï¿½ï¸®ï¿½ï¿½ ï¿½Ã°ï¿½

	void manualControl(struct ControlData *cdata, char key);

	uint32_t timeCheck(struct timeval *tempTime);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif //CONTROL_MISSION_H