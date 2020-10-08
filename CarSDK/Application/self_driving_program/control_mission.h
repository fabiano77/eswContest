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
	// 
	// PreCondition 	: none
	// PostCondition	: 
	//                    
	// Return 			: 
	int DistanceSensor_cm(int channel);

	// sensor_dist
	// 
	// PreCondition 	: none
	// PostCondition	: 
	//                    
	// Return 			: 
	int sensor_dist(int channel, int input);

	// Encoder_Read
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: 
	signed int Encoder_Read(void);

	// StopLine
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: 
	int StopLine(int Lineflag);

	// SteeringServo_Write_uart
	// 
	// PreCondition 	:
	// PostCondition	: 
	//                    
	// Return 			: none
	void SteeringServo_Write_uart(signed short angle);

	// DesireSpeed_Write_uart
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: none
	void DesireSpeed_Write_uart(signed short speed);

	// DesiredDistance
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: none
	void DesiredDistance(int SettingSpeed, int SettingDistance, int SettingSteering);

	// onlyDistance
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: none
	void onlyDistance(int SettingSpeed, int SettingDistance);

	// RoundAbout_isStart
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: 
	int RoundAbout_isStart(const int Distance1);

	// Tunnel_isEnd
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: 
	int Tunnel_isEnd(const int Distance1, const int Distance2, const int Distance3, const int Distance4);

	// Tunnel_SteerVal
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: 
	int Tunnel_SteerVal(const int Distance1, const int Distance2);

	// frontLightOnOff
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: none
	void frontLightOnOff(unsigned short lightFlag, int on);

	// rearLightOnOff
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: none
	void rearLightOnOff(unsigned short lightFlag, int on);

	// auto_speedMapping
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: 
	int auto_speedMapping(int steerVal, const int basicSpeed);

	// buzzer
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: none
	void buzzer(int numOfTimes, int interval_us, int pulseWidth_us);

	// manualControl
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: none
	void manualControl(struct ControlData *cdata, char key);

	// timeCheck
	// 
	// PreCondition 	: 
	// PostCondition	: 
	//                    
	// Return 			: none
	uint32_t timeCheck(struct timeval *tempTime);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif //CONTROL_MISSION_H